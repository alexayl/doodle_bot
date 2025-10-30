from __future__ import annotations

import os, sys, time, math, threading, shutil
from pathlib import Path
from contextlib import contextmanager
from typing import List, Tuple, Optional

# Load environment variables from .env file
from dotenv import load_dotenv
load_dotenv()

import cv2
import numpy as np
from flask import Flask, Response, request, render_template, redirect, url_for
from werkzeug.utils import secure_filename
import faulthandler

from app.cv_core import CVPipeline
from app.control import PathRun, Waypoint
from app.camera import ThreadedCamera, CameraConfig
from app.path_parse import load_gcode_file, convert_pathfinding_gcode
from app.bt_link import BTLink
from app.utils import encode_jpeg, Metrics

# -----------------------------------------------------------------------------#
# Config
# -----------------------------------------------------------------------------#
USE_TURBOJPEG = True  # encode_jpeg handles fallback
CAMERA_SRC = int(os.getenv("CAMERA_SRC", "0"))
PATHFINDING_DIR = os.getenv(
    "PATHFINDING_DIR",
    os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "pathfinding")),
)
STREAM_JPEG_QUALITY = 75
_K_GAIN = float(os.getenv("CV_K_GAIN", "0.5"))
_MAX_CORR_MM = float(os.getenv("CV_MAX_CORR_MM", "5"))

# Pathfinding canvas size used by path2gcode.py (pixels/abstract units)
PATH_CANVAS_SIZE = (
    float(os.getenv("PATH_CANVAS_WIDTH", "575")),
    float(os.getenv("PATH_CANVAS_HEIGHT", "730")),
)

app = Flask(__name__, template_folder="../templates", static_folder="../static")
os.makedirs("uploads", exist_ok=True)

STATE = {"mode": "home", "target_image": None}
MET = Metrics(maxlen=240)

# -----------------------------------------------------------------------------#
# Camera
# -----------------------------------------------------------------------------#
_cam: Optional[ThreadedCamera] = None
def get_cam() -> Optional[ThreadedCamera]:
    global _cam
    if _cam is None:
        try:
            _cam = ThreadedCamera(CameraConfig(src=CAMERA_SRC))
        except Exception:
            _cam = None
    return _cam

# -----------------------------------------------------------------------------#
# CV + Control
# -----------------------------------------------------------------------------#
cvp = CVPipeline(cam=None)
RUN = PathRun()
bt = BTLink()  # Simplified - uses DEVICE_NAME from environment or default "DOO"
@contextmanager
def _in_dir(path: str):
    old = os.getcwd()
    os.makedirs(path, exist_ok=True)
    os.chdir(path)
    try:
        yield
    finally:
        os.chdir(old)

def _as_waypoints(seq) -> List[Waypoint]:
    out: List[Waypoint] = []
    if not seq:
        return out
    for item in seq:
        if isinstance(item, Waypoint):
            out.append(item)
        elif isinstance(item, (tuple, list)) and len(item) >= 2:
            out.append(Waypoint(x_mm=float(item[0]), y_mm=float(item[1])))
    return out

# -----------------------------------------------------------------------------#
# G-code helpers
# -----------------------------------------------------------------------------#
def _ops_to_normalized_wps(ops: List[tuple]) -> List[Waypoint]:
    """
    Convert ("move", dx, dy) incremental ops to [0..1] normalized waypoints.
    """
    x = y = 0.0
    pts: List[tuple[float, float]] = []
    for op in ops:
        if not op:
            continue
        if op[0] == "move":
            x += float(op[1]); y += float(op[2])
            pts.append((x, y))
    if not pts:
        return []
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    minx, maxx, miny, maxy = min(xs), max(xs), min(ys), max(ys)
    w = max(maxx - minx, 1e-6)
    h = max(maxy - miny, 1e-6)

    out: List[Waypoint] = []
    for px, py in pts:
        nx = (px - minx) / w
        ny0 = (py - miny) / h
        ny = ny0
        out.append(Waypoint(x_mm=float(nx), y_mm=float(ny)))
    return out

def _load_named_gcode_normalized(name: str) -> List[Waypoint]:
    bases = [
        os.path.join(PATHFINDING_DIR, f"{name}.gcode"),
        os.path.join(PATHFINDING_DIR, "gcode", f"{name}.gcode"),
    ]
    ops: List[tuple] = []
    for p in bases:
        if os.path.isfile(p):
            try:
                ops = load_gcode_file(p)
                break
            except Exception:
                pass
    if not ops:
        return []
    return _ops_to_normalized_wps(ops)

def _convert_image_to_gcode(img_path: str, out_gcode_path: str, canvas_size=(575, 730)) -> str:
    """Run the pathfinding stack and write G-code next to the uploaded image."""
    if PATHFINDING_DIR not in sys.path:
        sys.path.insert(0, PATHFINDING_DIR)
    from img2graph import img2graph
    from graph2path import graph2path
    from path2gcode import path2gcode

    img_path = os.path.abspath(img_path)
    out_gcode_path = os.path.abspath(out_gcode_path)
    png_dir = os.path.join(PATHFINDING_DIR, "png")
    os.makedirs(png_dir, exist_ok=True)

    name = Path(img_path).stem
    canonical_png = os.path.join(png_dir, f"{name}.png")

    ext = Path(img_path).suffix.lower()
    if ext == ".png":
        if os.path.abspath(img_path) != os.path.abspath(canonical_png):
            shutil.copyfile(img_path, canonical_png)
    else:
        im = cv2.imread(img_path, cv2.IMREAD_UNCHANGED)
        if im is None:
            raise RuntimeError(f"Uploaded image could not be read: {img_path}")
        cv2.imwrite(canonical_png, im)

    with _in_dir(PATHFINDING_DIR):
        graph, endpoints = img2graph(name, canvas_size=canvas_size)
        path = graph2path(graph, endpoints, canvas_size=canvas_size)
        os.makedirs(os.path.dirname(out_gcode_path), exist_ok=True)
        path2gcode(path, filename=out_gcode_path)

    return out_gcode_path

def _read_gcode_text(path_or_name: str) -> str:
    """Return G-code file contents as a UTF-8 string."""
    if os.path.isfile(path_or_name):
        p = path_or_name
    else:
        p = os.path.join(PATHFINDING_DIR, f"{path_or_name}.gcode")
        if not os.path.isfile(p):
            p = os.path.join(PATHFINDING_DIR, "gcode", f"{path_or_name}.gcode")
    if not os.path.isfile(p):
        raise FileNotFoundError(f"G-code not found: {path_or_name}")
    with open(p, "r", encoding="utf-8") as f:
        return f.read()

# -----------------------------------------------------------------------------#
# CV worker (kept for metrics/overlay). Optional legacy move streaming gated.
# -----------------------------------------------------------------------------#
_cv_thread: Optional[threading.Thread] = None

def _process_frame_once(frame) -> tuple[bool, float]:
    t0 = time.perf_counter()
    try:
        valid, _ = cvp.process_frame(frame)
    except Exception:
        try:
            st = cvp.get_state()
            if (not st["calibrated"]) or (st["mm_per_px"] is None) or ((time.monotonic() % 2.0) < 0.05):
                cvp.calibrate_board(frame)
            cvp.update_bot(frame)
            st2 = cvp.get_state()
            valid = bool(st2["calibrated"] and st2["bot_pose"] is not None)
        except Exception:
            valid = False
    return bool(valid), (time.perf_counter() - t0)

def _cv_worker():
    try:
        bt.connect()
    except Exception:
        pass

    cam = get_cam()
    # Legacy move streaming is removed; BLE now only receives full G-code packets.

    while True:
        # if not bt.client or not bt.client.is_connected:
        # try:
        #     bt.connect()
        # except Exception:
        #     time.sleep(1.0)
        #     continue
        if cam is None:
            time.sleep(0.02)
            cam = get_cam()
            continue

        frame = cam.get_latest_good()
        if frame is None:
            time.sleep(0.0004)
            continue

        valid, tproc = _process_frame_once(frame)
        MET.note(valid, tproc)

        time.sleep(0.004)

def _ensure_cv_thread():
    global _cv_thread
    if _cv_thread is None:
        _cv_thread = threading.Thread(target=_cv_worker, daemon=True)
        _cv_thread.start()

# -----------------------------------------------------------------------------#
# Overlay/Stream
# -----------------------------------------------------------------------------#
def overlay_fn():
    def _draw(frame):
        st = cvp.get_state()
        line1 = f"Mode: {STATE['mode'].upper()}"
        if st["calibrated"]:
            try:
                line1 += f" | mm/px: {st['mm_per_px']:.3f} | reproj: {st['board_reproj_err_px']:.2f}"
            except Exception:
                pass
        else:
            line1 += " | CALIBRATING..."
        run_txt = f"Run: {'ON' if RUN.active else 'OFF'} {RUN.follower.index()}/{RUN.follower.total()}"
        line2 = f"Rate: {MET.rate_hz():.1f} Hz | Median cam→pose: {MET.median_latency_ms():.0f} ms | Uptime: {MET.uptime_pct:.1f}% | {run_txt}"
        cv2.putText(frame, line1, (16, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(frame, line2, (16, 64), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2, cv2.LINE_AA)
        return frame
    return _draw

def gen_frames(overlay_factory, quality=65):
    _ensure_cv_thread()
    draw = overlay_factory() if callable(overlay_factory) else overlay_factory
    boundary = b"--frame\r\n"
    last_ts = 0.0

    while True:
        cam = get_cam()
        if cam is None:
            time.sleep(0.02); continue
        frame, ts = cam.get_latest_with_ts()
        if frame is None:
            time.sleep(0.005); continue
        if ts <= last_ts:
            time.sleep(0.002); continue
        last_ts = ts

        if draw:
            frame = draw(frame)

        jpg = encode_jpeg(frame, quality=max(40, min(int(quality), 90)))
        if jpg is None:
            continue
        yield boundary + b"Content-Type: image/jpeg\r\nCache-Control: no-store\r\n\r\n" + jpg + b"\r\n"

@app.after_request
def _no_buffer(resp):
    resp.headers["Cache-Control"] = "no-store, no-cache, must-revalidate, max-age=0"
    resp.headers["Pragma"] = "no-cache"
    resp.headers["X-Accel-Buffering"] = "no"
    return resp

def sanitize_for_firmware(gcode_text: str, force_relative: bool = True) -> str:
    """
    Keep: G90, G91, G0/G1 (with any args), M280.
    Strip: G21 and anything else.
    If force_relative and no G91 seen, prepend one.
    """
    out = []
    saw_g91 = False
    for raw in gcode_text.replace("\r\n", "\n").replace("\r", "\n").split("\n"):
        line = raw.split(";")[0].strip()
        if not line:
            continue
        u = line.upper()

        if u.startswith("G21"):
            continue
        if u.startswith("G91"):
            saw_g91 = True
            out.append("G91")
            continue
        if u.startswith("G90"):
            out.append("G90")
            continue
        if u.startswith(("G0", "G1")):
            out.append(line)
            continue
        if u.startswith("M280"):
            out.append(line)
            continue
        # drop others

    if force_relative and not saw_g91:
        out.insert(0, "G91")
    return "\n".join(out) + "\n"
# -----------------------------------------------------------------------------#
# Routes
# -----------------------------------------------------------------------------#
@app.route("/")
def home():
    _ensure_cv_thread()
    STATE["mode"] = "home"
    return render_template("index.html", page="home")

@app.route("/erase", methods=["GET", "POST"])
def erase():
    _ensure_cv_thread()
    STATE["mode"] = "erase"
    msg = None; error = None

    if request.method == "POST":
        action = request.form.get("action", "start")
        if action == "stop":
            RUN.stop()
            return redirect(url_for("erase"))

        # 1) Build normalized path for local preview/control (unchanged)
        raw_norm = _load_named_gcode_normalized("erase")
        if not raw_norm:
            error = "No erase path found in pathfinding directory."
        else:
            fitted = cvp.fit_path_to_board(raw_norm, margin_frac=0.10)
            if not fitted:
                error = "Board not calibrated yet."
            else:
                try:
                    gcode_str = _read_gcode_text("erase")
                    
                    # Normalize and scale to firmware G-code using current calibration
                    firmware_gcode = cvp.build_firmware_gcode(gcode_str, canvas_size=PATH_CANVAS_SIZE)
                    bt.send_gcode(sanitize_for_firmware(firmware_gcode, force_relative=False))
                except Exception as e:
                    error = f"Failed to send G-code: {e}"

                RUN.load(_as_waypoints(fitted))
                RUN.start()
                msg = f"Erase queued with {len(fitted)} vertices and sent over BLE."

        return render_template("index.html", page="erase", msg=msg, error=error)

    return render_template("index.html", page="erase", msg=msg, error=error)

@app.route("/draw", methods=["GET", "POST"])
def draw():
    _ensure_cv_thread()
    STATE["mode"] = "draw"
    msg = None; error = None

    if request.method == "POST":
        action = request.form.get("action", "start")
        if action == "stop":
            RUN.stop()
            return redirect(url_for("draw"))

        uploaded_file = request.files.get("file")
        raw_norm: List[Waypoint] = []
        gcode_path: Optional[str] = None

        if uploaded_file and uploaded_file.filename:
            fname = secure_filename(uploaded_file.filename)
            img_path = os.path.join("uploads", fname)
            uploaded_file.save(img_path)
            STATE["target_image"] = fname
            try:
                base = os.path.splitext(fname)[0]
                gcode_path = os.path.join("uploads", f"{base}.gcode")
                gcode_path = _convert_image_to_gcode(img_path, gcode_path, canvas_size=(575, 730))
                ops = load_gcode_file(gcode_path)
                raw_norm = _ops_to_normalized_wps(ops)
            except Exception as e:
                raw_norm = []
                error = f"Image conversion failed: {e}"

        if not raw_norm:
            fallback = _load_named_gcode_normalized("draw")
            if not fallback and error is None:
                error = "No draw path found and no valid uploaded image."
            raw_norm = fallback

        if raw_norm:
            if cvp.cal.board_pose and cvp.cal.board_pose.mm_per_px is None:
                cvp.scale.from_board_size()
            fitted = cvp.fit_path_to_board(raw_norm, margin_frac=0.10)
            if not fitted:
                error = "Board not calibrated yet (no homography or scale)."
            else:
                # Send raw G-code to MCU (use generated file if present, else named fallback)
                try:
                    src_for_gcode = gcode_path if gcode_path else "draw"
                    gcode_str = _read_gcode_text(src_for_gcode)
                    
                    # Normalize and scale to firmware G-code using current calibration
                    firmware_gcode = cvp.build_firmware_gcode(gcode_str, canvas_size=PATH_CANVAS_SIZE)
                    bt.send_gcode(sanitize_for_firmware(firmware_gcode, force_relative=False))
                except Exception as e:
                    error = f"Failed to send G-code: {e}"

                RUN.load(_as_waypoints(fitted))
                RUN.start()
                msg = f"Draw queued with {len(fitted)} vertices and sent over BLE."

        return render_template("index.html", page="draw",
                               has_target=bool(STATE["target_image"]),
                               target=STATE["target_image"],
                               msg=msg, error=error)

    return render_template("index.html", page="draw",
                           has_target=bool(STATE["target_image"]),
                           target=STATE["target_image"],
                           msg=msg, error=error)

@app.route("/stream")
def stream():
    q = int(request.args.get("q", STREAM_JPEG_QUALITY))
    q = max(40, min(q, 80))
    return Response(gen_frames(overlay_fn, quality=q),
                    mimetype="multipart/x-mixed-replace; boundary=frame")

def _shutdown():
    try:
        RUN.stop()
        bt.close()  # Use close() instead of stop()
    except Exception:
        pass
    cam = get_cam()
    if cam:
        cam.close()
    os._exit(0)

if __name__ == "__main__":
    try: faulthandler.enable()
    except Exception: pass
    _ensure_cv_thread()
    app.run(host="0.0.0.0", port=int(os.getenv("PORT", "5000")), threaded=True, debug=False)
