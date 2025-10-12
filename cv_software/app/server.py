from __future__ import annotations
from flask import Flask, Response, request, render_template, redirect, url_for
import cv2
import os
import time
import threading
from collections import deque
from typing import List
import numpy as np
from werkzeug.utils import secure_filename
from app.cv_core import CVPipeline
from app.control import PathRun, Waypoint
from app.camera import ThreadedCamera, CameraConfig
from app.bt_link import BTLink
from app.path_parse import load_gcode_file

USE_TURBOJPEG = os.getenv("DISABLE_TURBOJPEG", "0") not in ("1", "true", "True")
CAMERA_SRC = int(os.getenv("CAMERA_SRC", "0"))
PATHFINDING_DIR = os.getenv(
    "PATHFINDING_DIR",
    os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "pathfinding"))
)

app = Flask(__name__, template_folder="../templates", static_folder="../static")
os.makedirs("uploads", exist_ok=True)

STATE = {"mode": "home", "target_image": None}

_METRICS = {
    "t_hist": deque(maxlen=240),
    "ts_hist": deque(maxlen=240),
    "valid_frames": 0,
    "total_frames": 0,
    "uptime_pct": 0.0,
}

def _note_frame(valid: bool, t_proc_s: float):
    _METRICS["t_hist"].append(t_proc_s)
    _METRICS["ts_hist"].append(time.monotonic())
    _METRICS["total_frames"] += 1
    if valid:
        _METRICS["valid_frames"] += 1
    total = max(1, _METRICS["total_frames"])
    _METRICS["uptime_pct"] = 100.0 * _METRICS["valid_frames"] / total

def _rate_hz() -> float:
    ts = list(_METRICS["ts_hist"])
    if len(ts) < 3:
        return 0.0
    dur = ts[-1] - ts[0]
    if dur <= 1e-6:
        return 0.0
    return float(len(ts) - 1) / float(dur)

def _median_latency_ms() -> float:
    arr = sorted(_METRICS["t_hist"])
    if not arr:
        return 0.0
    n = len(arr)
    mid = n // 2
    if n % 2:
        med = arr[mid]
    else:
        med = 0.5 * (arr[mid - 1] + arr[mid])
    return 1000.0 * med

_jpeg = None

def _get_jpeg():
    global _jpeg
    if _jpeg is not None:
        return _jpeg
    if not USE_TURBOJPEG:
        _jpeg = False
        return _jpeg
    try:
        from turbojpeg import TurboJPEG
        _jpeg = TurboJPEG()
    except Exception:
        _jpeg = False
    return _jpeg

def _encode_jpeg(frame, quality=65):
    tj = _get_jpeg()
    if tj:
        try:
            return tj.encode(frame, quality=int(quality), jpeg_subsample=2, flags=512)
        except Exception:
            pass
    ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    if not ok:
        return None
    return buf.tobytes()

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

def _process_frame_once(frame) -> tuple[bool, float]:
    t0 = time.perf_counter()
    try:
        valid, _ = cvp.process_frame(frame)
        t_proc = time.perf_counter() - t0
        return bool(valid), t_proc
    except Exception:
        pass

    st = cvp.get_state()
    do_cal = not st["calibrated"] or st["mm_per_px"] is None or (time.monotonic() % 2.0) < 0.05
    if do_cal:
        try:
            cvp.calibrate_board(frame)
        except Exception:
            pass
    try:
        cvp.update_bot(frame)
    except Exception:
        pass

    st2 = cvp.get_state()
    valid = bool(st2["calibrated"] and st2["bot_pose"] is not None)
    t_proc = time.perf_counter() - t0
    return valid, t_proc

def _cv_worker():
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
        _note_frame(valid, tproc)

        if RUN.active and bt_link.wait_idle(idle_timeout_s):
            pose = cvp.get_pose_mm()
            vertices: List[tuple[float, float]] = []
            for _ in range(batch_vertices):
                step = RUN.follower.step(pose)
                if step is None:
                    RUN.stop()
                    break
                vertices.append(step)
                pose = cvp.get_pose_mm()
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
        line2 = f"Rate: {_rate_hz():.1f} Hz | Median cam→pose: {_median_latency_ms():.0f} ms | Uptime: {_METRICS['uptime_pct']:.1f}% | {run_txt}"
        cv2.putText(frame, line1, (16, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(frame, line2, (16, 64), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2, cv2.LINE_AA)

        if st["bot_pose"] is not None and st["calibrated"]:
            bp = cvp.cal.board_pose
            if bp is not None:
                cx, cy = st["bot_pose"]["center_board_px"]
                pt_board = np.float32([[cx, cy]])
                Hinv = np.linalg.inv(bp.H_img2board)
                pt_img = cv2.perspectiveTransform(pt_board[None], Hinv)[0][0]
                x = int(pt_img[0])
                y = int(pt_img[1])
                cv2.circle(frame, (x, y), 8, (0, 255, 255), -1)
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

        jpg = _encode_jpeg(frame, quality=quality)
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
    x = 0.0
    y = 0.0
    pts: List[tuple[float, float]] = []
    for op in ops:
        if not op:
            continue
        if op[0] == "move":
            x += float(op[1])
            y += float(op[2])
            pts.append((x, y))
    if not pts:
        return []
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    minx = min(xs)
    maxx = max(xs)
    miny = min(ys)
    maxy = max(ys)
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
        raw_norm = []

        if uploaded_file and uploaded_file.filename:
            os.makedirs("uploads", exist_ok=True)
            fname = secure_filename(uploaded_file.filename)
            img_path = os.path.join("uploads", fname)
            uploaded_file.save(img_path)
            STATE["target_image"] = fname

            out_gcode = os.path.join("uploads", os.path.splitext(fname)[0] + ".gcode")
            ok = True# need to integrate with pf
            if ok:
                try:
                    ops = load_gcode_file(out_gcode)
                    raw_norm = _ops_to_normalized_wps(ops)
                except Exception:
                    raw_norm = []
            else:
                error = "Could not convert the uploaded image to a path."

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
        import faulthandler
        faulthandler.enable()
    except Exception:
        pass
    _ensure_cv_thread()
    app.run(host="0.0.0.0", port=int(os.getenv("PORT", "5000")), threaded=True, debug=False)