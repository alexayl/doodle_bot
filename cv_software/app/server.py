from __future__ import annotations
from flask import Flask, Response, request, render_template, redirect, url_for
import cv2, os, time, threading, numpy as np
from typing import List, Tuple
from werkzeug.utils import secure_filename
import faulthandler

from app.cv_core import CVPipeline
from app.control import PathRun, Waypoint
from app.camera import ThreadedCamera, CameraConfig
from app.bt_link import BTLink
from app.path_parse import load_gcode_file
from app.utils import encode_jpeg, Metrics

USE_TURBOJPEG = True  # encode_jpeg handles fallback
CAMERA_SRC = int(os.getenv("CAMERA_SRC", "0"))
PATHFINDING_DIR = os.getenv(
    "PATHFINDING_DIR",
    os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "pathfinding"))
)

app = Flask(__name__, template_folder="../templates", static_folder="../static")
os.makedirs("uploads", exist_ok=True)

STATE = {"mode": "home", "target_image": None}
MET = Metrics(maxlen=240)

_cam = None
def get_cam() -> ThreadedCamera | None:
    global _cam
    if _cam is None:
        try:
            cfg = CameraConfig(src=CAMERA_SRC)
            _cam = ThreadedCamera(cfg)
        except Exception:
            _cam = None
    return _cam

cvp = CVPipeline(cam=None)
bt_link = BTLink()
RUN = PathRun()

_cv_thread = None
_K_GAIN = float(os.getenv("CV_K_GAIN", "0.5"))
_MAX_CORR_MM = float(os.getenv("CV_MAX_CORR_MM", "5")) 

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
    """Core closed-loop executor running in a background thread.

    Behavior:
      * Waits for firmware IDLE via BTLink.
      * Batches a few (fwd,left) steps from PathFollower (robot frame).
      * ALWAYS applies CV-based error nudging before sending vertices:
          - Error = current target (board mm) - current pose (board mm)
          - Proportional, saturated corrections rotated into robot frame
      * Sends batched vertices to firmware.

      - This is the one place where instructions are improved by CV.
      - If CV confidence is low, corrections are small; if pose is unavailable,
        it still streams the planned step (fails safe).
    """
    try:
        bt_link.connect()
    except Exception:
        pass

    cam = get_cam()
    batch_vertices = 8
    idle_timeout_s = 1.0

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

        if RUN.active and bt_link.wait_idle(idle_timeout_s):
            pose = cvp.get_pose_mm()
            vertices: List[tuple[float, float]] = []
            for _ in range(batch_vertices):
                step = RUN.follower.step(pose)  # (fwd, lef) in ROBOT frame
                if step is None:
                    RUN.stop()
                    break
                st = cvp.get_state()
                ok_conf = (
                    st["calibrated"]
                    and (st["board_confidence"] is None or st["board_confidence"] >= 0.60)
                    and st["bot_pose"] is not None
                    and st["bot_pose"]["confidence"] >= 0.60
                )

                if ok_conf and pose is not None:
                    err = RUN.follower.current_error(pose)  # single source of truth
                    if err is not None:
                        ex, ey, _tgt = err
                        cx = _clamp(_K_GAIN * ex, _MAX_CORR_MM)
                        cy = _clamp(_K_GAIN * ey, _MAX_CORR_MM)
                        corr = cvp.corrected_delta_for_bt((cx, cy), rotate_into_bot_frame=True)
                        if corr is not None:
                            step = (step[0] + float(corr[0]), step[1] + float(corr[1]))
                # -------------------------------------------------------------------

                vertices.append(step)
                pose = cvp.get_pose_mm()  # refresh pose for next iteration

            if vertices:
                try:
                    bt_link.send_waypoints(vertices)
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

        if st["bot_pose"] is not None and st["calibrated"]:
            bp = cvp.cal.board_pose
            if bp is not None:
                cx, cy = st["bot_pose"]["center_board_px"]
                Hinv = np.linalg.inv(bp.H_img2board)
                pt_img = cv2.perspectiveTransform(np.float32([[cx, cy]])[None], Hinv)[0][0]
                cv2.circle(frame, (int(pt_img[0]), int(pt_img[1])), 8, (0, 255, 255), -1)
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
            time.sleep(0.02)
            continue

        frame, ts = cam.get_latest_with_ts()
        if frame is None:
            time.sleep(0.005)
            continue

        if ts <= last_ts:
            time.sleep(0.002)
            continue
        last_ts = ts

        if draw:
            frame = draw(frame)

        jpg = encode_jpeg(frame, quality=quality)
        if jpg is None:
            continue

        yield boundary + b"Content-Type: image/jpeg\r\nCache-Control: no-store\r\n\r\n" + jpg + b"\r\n"

@app.after_request
def _no_buffer(resp):
    resp.headers["Cache-Control"] = "no-store, no-cache, must-revalidate, max-age=0"
    resp.headers["Pragma"] = "no-cache"
    resp.headers["X-Accel-Buffering"] = "no"
    return resp

def _ops_to_normalized_wps(ops: List[tuple]) -> List[Waypoint]:
    """
    Convert incremental G1 moves from parsed G-code into [0..1] normalized waypoints.

    Input
    -----
    ops : List[tuple]
        Output of `load_gcode_file(...)`, where each item looks like:
        ("move", dx, dy) in *incremental* (G91) millimeters. ("pen", bool) lines are ignored.

    Output
    ------
    List[Waypoint]
        Waypoints with x_mm,y_mm each in [0,1], preserving the original path’s
        aspect ratio and relative geometry. These are later scaled to board mm
        via `cvp.fit_path_to_board(...)`.
    """
    x = y = 0.0
    pts: List[tuple[float, float]] = []
    for op in ops:
        if op and op[0] == "move":
            x += float(op[1]); y += float(op[2])
            pts.append((x, y))
    if not pts:
        return []
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    minx, maxx, miny, maxy = min(xs), max(xs), min(ys), max(ys)
    w = max(maxx - minx, 1e-6)
    h = max(maxy - miny, 1e-6)
    norm: List[Waypoint] = []
    for px, py in pts:
        nx = (px - minx) / w
        ny = (py - miny) / h
        norm.append(Waypoint(x_mm=float(nx), y_mm=float(ny)))
    return norm


def _load_named_gcode_normalized(name: str) -> List[Waypoint]:
    """
    Load `pathfinding/<name>.gcode`, parse incremental moves, and return normalized waypoints.

    This is a thin wrapper around the G-code parser that:
      1) finds `<repo-root>/pathfinding/<name>.gcode`,
      2) parses it to ("move", dx, dy) ops,
      3) converts to [0..1] waypoints via `_ops_to_normalized_wps`.

    Returns an empty list if the file is missing or can’t be parsed.
    """
    bases = [os.path.join(PATHFINDING_DIR, f"{name}.gcode")]
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


def _emit_gcode(ops: List[tuple]) -> str:
    """ops: [("pen", bool)|("move", dx,dy,dz)]. Returns a G-code string."""
    lines, pen_down = ["G91"], False  # incremental
    for op in ops:
        if not op: 
            continue
        if op[0] == "pen":
            down = bool(op[1])
            lines.append("M3" if down else "M5")
            pen_down = down
        elif op[0] == "move":
            _, dx, dy, dz = op
            parts = ["G1"]
            if abs(dx) > 1e-9: parts.append(f"X{dx:.3f}")
            if abs(dy) > 1e-9: parts.append(f"Y{dy:.3f}")
            if abs(dz) > 1e-9: parts.append(f"Z{dz:.3f}")
            if len(parts) > 1: lines.append(" ".join(parts))
    if pen_down:
        lines.append("M5")
    lines.append("G90")  # back to absolute
    return "\n".join(lines) + "\n"

@app.route("/")
def home():
    _ensure_cv_thread()
    STATE["mode"] = "home"
    return render_template("index.html", page="home")

@app.route("/erase", methods=["GET", "POST"])
def erase():
    _ensure_cv_thread()
    STATE["mode"] = "erase"
    msg = None
    error = None

    if request.method == "POST":
        action = request.form.get("action", "start")
        if action == "stop":
            RUN.stop()
            try:
                bt_link.stop()
            except Exception:
                pass
            return redirect(url_for("erase"))

        raw_norm = _load_named_gcode_normalized("erase")
        if not raw_norm:
            error = "No erase path found in pathfinding directory."
        else:
            fitted = cvp.fit_path_to_board(raw_norm, margin_frac=0.10)
            if not fitted:
                error = "Board not calibrated yet."
            else:
                RUN.load(fitted)
                RUN.start()
                msg = f"Erase queued with {len(fitted)} vertices (firmware will smooth)."
        return render_template("index.html", page="erase", msg=msg, error=error)

    return render_template("index.html", page="erase", msg=msg, error=error)

@app.route("/draw", methods=["GET", "POST"])
def draw():
    _ensure_cv_thread()
    STATE["mode"] = "draw"
    msg = None
    error = None

    if request.method == "POST":
        action = request.form.get("action", "start")
        if action == "stop":
            RUN.stop()
            try:
                bt_link.stop()
            except Exception:
                pass
            return redirect(url_for("draw"))

        uploaded_file = request.files.get("file")
        raw_norm: List[Waypoint] = []

        if uploaded_file and uploaded_file.filename:
            fname = secure_filename(uploaded_file.filename)
            img_path = os.path.join("uploads", fname)
            uploaded_file.save(img_path)
            STATE["target_image"] = fname

            # need to finalize integrate pathfinding (image -> G-code)
            out_gcode = os.path.join("uploads", os.path.splitext(fname)[0] + ".gcode")
            try:
                ops = load_gcode_file(out_gcode)
                raw_norm = _ops_to_normalized_wps(ops)
            except Exception:
                raw_norm = []
                error = error or "Could not convert the uploaded image to a path."

        if not raw_norm:
            fallback = _load_named_gcode_normalized("draw")
            if not fallback and error is None:
                error = "No draw path found and no valid uploaded image."
            raw_norm = fallback

        if raw_norm:
            fitted = cvp.fit_path_to_board(raw_norm, margin_frac=0.10)
            if not fitted:
                error = "Board not calibrated yet."
            else:
                RUN.load(fitted)
                RUN.start()
                msg = f"Draw queued with {len(fitted)} vertices (firmware will smooth)."

        return render_template(
            "index.html",
            page="draw",
            has_target=bool(STATE["target_image"]),
            target=STATE["target_image"],
            msg=msg,
            error=error
        )

    return render_template(
        "index.html",
        page="draw",
        has_target=bool(STATE["target_image"]),
        target=STATE["target_image"],
        msg=msg,
        error=error
    )

@app.route("/stream")
def stream():
    try:
        q = int(request.args.get("q", "65"))
    except ValueError:
        q = 65
    q = max(40, min(q, 90))
    return Response(gen_frames(overlay_fn, quality=q), mimetype="multipart/x-mixed-replace; boundary=frame")

def _shutdown():
    try:
        RUN.stop()
        bt_link.stop()
    except Exception:
        pass
    cam = get_cam()
    if cam:
        cam.close()
    os._exit(0)

if __name__ == "__main__":
    try:
        faulthandler.enable()
    except Exception:
        pass
    _ensure_cv_thread()
    app.run(host="0.0.0.0", port=int(os.getenv("PORT", "5000")), threaded=True, debug=False)
