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

cv2.setNumThreads(1)

USE_TURBOJPEG = True
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

# -------- single shared JPEG encoder --------
_STREAM_Q = int(os.getenv("STREAM_Q", "50"))
_STREAM_FPS = float(os.getenv("STREAM_FPS", "6"))
_last_jpeg: bytes | None = None
_last_jpeg_ts: float = 0.0
_jpeg_thread = None
_jpeg_lock = threading.Lock()

def _jpeg_loop(overlay_factory):
    global _last_jpeg, _last_jpeg_ts
    draw = overlay_factory() if callable(overlay_factory) else overlay_factory
    period = 1.0 / max(1.0, float(_STREAM_FPS))
    next_t = time.perf_counter()
    while True:
        cam = get_cam()
        if cam is None:
            time.sleep(0.02)
            continue
        frame, ts = cam.get_latest_with_ts()
        if frame is None:
            time.sleep(0.01)
            continue
        now = time.perf_counter()
        if now < next_t:
            time.sleep(max(0.0, next_t - now))
            continue
        next_t = now + period
        if draw:
            frame = draw(frame)
        jpg = encode_jpeg(frame, quality=_STREAM_Q)
        if jpg is None:
            continue
        with _jpeg_lock:
            _last_jpeg = jpg
            _last_jpeg_ts = ts

def _ensure_jpeg_thread():
    global _jpeg_thread
    if _jpeg_thread is None:
        _jpeg_thread = threading.Thread(target=_jpeg_loop, args=(overlay_fn,), daemon=True)
        _jpeg_thread.start()

# --------------------------------------------

def _clamp(v: float, m: float) -> float:
    return -m if v < -m else (m if v > m else v)

def _process_frame_once(frame, capture_ts: float) -> tuple[bool, float, float]:
    t0 = time.monotonic()
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
    pose_ts = time.monotonic()
    return bool(valid), (pose_ts - t0), (pose_ts - capture_ts)

def _cv_worker():
    try:
        bt_link.connect()
    except Exception:
        pass

    cam = get_cam()
    batch_vertices = 8
    idle_timeout_s = 1.0

    IDLE_HZ   = float(os.getenv("CV_IDLE_HZ", "8"))
    ACTIVE_HZ = float(os.getenv("CV_ACTIVE_HZ", "20"))
    next_t = time.perf_counter()

    while True:
        if cam is None:
            time.sleep(0.02)
            cam = get_cam()
            continue

        target_hz = ACTIVE_HZ if RUN.active else IDLE_HZ
        period = 1.0 / target_hz
        now = time.perf_counter()
        if now < next_t:
            time.sleep(max(0.0, next_t - now))
            continue
        next_t = now + period

        frame, capture_ts = cam.get_latest_with_ts()
        if frame is None:
            continue

        valid, tproc, te2e = _process_frame_once(frame, capture_ts)
        MET.note(valid, tproc, te2e, ts_now=time.monotonic())

        if RUN.active and bt_link.wait_idle(idle_timeout_s):
            pose = cvp.get_pose_mm()
            vertices: List[tuple[float, float]] = []
            for _ in range(batch_vertices):
                step = RUN.follower.step(pose)
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
                    err = RUN.follower.current_error(pose)
                    if err is not None:
                        ex, ey, _tgt = err
                        cx = _clamp(_K_GAIN * ex, _MAX_CORR_MM)
                        cy = _clamp(_K_GAIN * ey, _MAX_CORR_MM)
                        corr = cvp.corrected_delta_for_bt((cx, cy), rotate_into_bot_frame=True)
                        if corr is not None:
                            step = (step[0] + float(corr[0]), step[1] + float(corr[1]))
                vertices.append(step)
                pose = cvp.get_pose_mm()
            if vertices:
                try:
                    bt_link.send_waypoints(vertices)
                except Exception:
                    pass

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
        line2 = f"Rate: {MET.rate_hz():.1f} Hz | Median cam→pose: {MET.median_e2e_ms():.0f} ms | Uptime: {MET.uptime_pct:.1f}% | {run_txt}"
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

@app.after_request
def _no_buffer(resp):
    resp.headers["Cache-Control"] = "no-store, no-cache, must-revalidate, max-age=0"
    resp.headers["Pragma"] = "no-cache"
    resp.headers["X-Accel-Buffering"] = "no"
    return resp

def _ops_to_normalized_wps(ops: List[tuple]) -> List[Waypoint]:
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
    lines, pen_down = ["G91"], False
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
    lines.append("G90")
    return "\n".join(lines) + "\n"

@app.route("/")
def home():
    _ensure_cv_thread()
    _ensure_jpeg_thread()
    STATE["mode"] = "home"
    return render_template("index.html", page="home")

@app.route("/erase", methods=["GET", "POST"])
def erase():
    _ensure_cv_thread()
    _ensure_jpeg_thread()
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
    _ensure_jpeg_thread()
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
    _ensure_cv_thread()
    _ensure_jpeg_thread()
    boundary = b"--frame\r\n"
    def _gen():
        last_seen = 0.0
        while True:
            with _jpeg_lock:
                jpg = _last_jpeg
                ts = _last_jpeg_ts
            if jpg is None or ts == last_seen:
                time.sleep(0.02)
                continue
            last_seen = ts
            yield (boundary +
                   b"Content-Type: image/jpeg\r\nCache-Control: no-store\r\n\r\n" +
                   jpg + b"\r\n")
    return Response(_gen(), mimetype="multipart/x-mixed-replace; boundary=frame")

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
    _ensure_jpeg_thread()
    app.run(host="0.0.0.0", port=int(os.getenv("PORT", "5000")), threaded=True, debug=False)
#run command STREAM_FPS=6 STREAM_Q=50 \
# CAMERA_W=640 CAMERA_H=360 CAMERA_FPS=10 CAMERA_USE_MJPEG=1 CAMERA_BUFFER_SIZE=1 \
# CV_IDLE_HZ=8 CV_ACTIVE_HZ=20 \
# gunicorn server:app -w 1 -k gevent -b 0.0.0.0:5000 --timeout 0