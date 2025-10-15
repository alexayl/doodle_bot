from __future__ import annotations

import os, sys, time, math, threading, shutil
from pathlib import Path
from contextlib import contextmanager
from typing import List, Tuple, Optional

import cv2
import numpy as np
from flask import Flask, Response, request, render_template, redirect, url_for
from werkzeug.utils import secure_filename
import faulthandler

from app.cv_core import CVPipeline
from app.control import PathRun, Waypoint
from app.camera import ThreadedCamera, CameraConfig
from app.path_parse import load_gcode_file
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

_K_GAIN = float(os.getenv("CV_K_GAIN", "0.5"))
_MAX_CORR_MM = float(os.getenv("CV_MAX_CORR_MM", "5"))

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
bt = BTLink(port=os.getenv("BT_PORT", "/dev/ttyUSB0"), baud=int(os.getenv("BT_BAUD", "115200")))

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
    Convert ("move", dx,dy, dz) incremental ops to [0..1] normalized waypoints.
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

# -----------------------------------------------------------------------------#
# CV worker; Bluetooth sending of (dx,dy,dz) packets)
# -----------------------------------------------------------------------------#
_cv_thread: Optional[threading.Thread] = None

def _clamp(v: float, m: float) -> float:
    return -m if v < -m else (m if v > m else v)
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
    batch_vertices = 12
    idle_timeout_s = 1.0

    # pen state we mirror to firmware via dz flags
    pen_is_down = False
    sent_initial_pen = False

    while True:
        if cam is None:
            time.sleep(0.02)
            cam = get_cam()
            continue

        frame = cam.get_latest_good()
        if frame is None:
            time.sleep(0.004)
            continue

        valid, tproc = _process_frame_once(frame)
        MET.note(valid, tproc)

        if RUN.active:
            bt.wait_idle(idle_timeout_s)

            pose = cvp.get_pose_mm()
            verts: List[Tuple[float, float, int]] = []  # (dx,dy,dz)
            for _ in range(batch_vertices):
                step = RUN.follower.step(pose)  # (fwd, left) in ROBOT frame
                if step is None:
                    if pen_is_down:
                        verts.append((0.0, 0.0, +1))
                        pen_is_down = False
                    RUN.stop()
                    sent_initial_pen = False
                    break

                # CV correction in BOARD mm, rotate ROBOT
                st = cvp.get_state()
                ok_conf = (
                    st["calibrated"]
                    and (st["board_confidence"] is None or st["board_confidence"] >= 0.60)
                    and st["bot_pose"] is not None
                    and st["bot_pose"]["confidence"] >= 0.60
                )
                if ok_conf and pose is not None:
                    err = RUN.follower.current_error(pose)
                    if err is not None:
                        ex, ey, _tgt = err
                        cx = _clamp(_K_GAIN * ex, _MAX_CORR_MM)
                        cy = _clamp(_K_GAIN * ey, _MAX_CORR_MM)
                        corr = cvp.corrected_delta_for_bt((cx, cy), rotate_into_bot_frame=True)
                        if corr is not None:
                            step = (step[0] + float(corr[0]), step[1] + float(corr[1]))

                dz = 0
                if not sent_initial_pen:
                    verts.append((0.0, 0.0, -1))
                    pen_is_down = True
                    sent_initial_pen = True

                verts.append((float(step[0]), float(step[1]), dz))

                # refresh pose for next iteration
                pose = cvp.get_pose_mm()

            if verts:
                try:
                    bt.send_moves_with_dz(verts)
                except Exception:
                    pass

        time.sleep(0.004)

def _ensure_cv_thread():
    global _cv_thread
    if _cv_thread is None:
        _cv_thread = threading.Thread(target=_cv_worker, daemon=True)
        _cv_thread.start()

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
            try: bt.stop()
            except: pass
            return redirect(url_for("erase"))

        raw_norm = _load_named_gcode_normalized("erase")
        if not raw_norm:
            error = "No erase path found in pathfinding directory."
        else:
            fitted = cvp.fit_path_to_board(raw_norm, margin_frac=0.10)
            if not fitted:
                error = "Board not calibrated yet."
            else:
                RUN.load(_as_waypoints(fitted))
                RUN.start()
                msg = f"Erase queued with {len(fitted)} vertices."
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
            try: bt.stop()
            except: pass
            return redirect(url_for("draw"))

        uploaded_file = request.files.get("file")
        raw_norm: List[Waypoint] = []

        if uploaded_file and uploaded_file.filename:
            fname = secure_filename(uploaded_file.filename)
            img_path = os.path.join("uploads", fname)
            uploaded_file.save(img_path)
            STATE["target_image"] = fname
            try:
                base = os.path.splitext(fname)[0]
                out_gcode = os.path.join("uploads", f"{base}.gcode")
                gcode_path = _convert_image_to_gcode(img_path, out_gcode, canvas_size=(575, 730))
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
                RUN.load(_as_waypoints(fitted))
                RUN.start()
                msg = f"Draw queued with {len(fitted)} vertices."

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
        bt.stop()
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
