# app/server.py
from __future__ import annotations
from flask import Flask, Response, request, jsonify, render_template, redirect, url_for
import cv2, os, time, threading, signal, math, platform
from collections import deque

# ---- CV + camera ----
from app.cv_core import CVPipeline
from app.control import parse_moves_payload
from app.camera import ThreadedCamera, CameraConfig
from app.bt_link import BTLink

# ---------- Config ----------
USE_TURBOJPEG = os.getenv("DISABLE_TURBOJPEG", "0") not in ("1", "true", "True")
CAMERA_SRC = int(os.getenv("CAMERA_SRC", "0"))

# ---------- App ----------
app = Flask(__name__, template_folder="../templates", static_folder="../static")
os.makedirs("uploads", exist_ok=True)

STATE = {"mode": "erase", "target_image": None}

# ---- Metrics state (rolling windows) ----
_METRICS = {
    "t_hist": deque(maxlen=240),   # processing times (s) ~12s at 20Hz
    "ts_hist": deque(maxlen=240),  # timestamps for Hz
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
    _METRICS["uptime_pct"] = 100.0 * _METRICS["valid_frames"] / max(1, _METRICS["total_frames"])

def _rate_hz() -> float:
    ts = list(_METRICS["ts_hist"])
    if len(ts) < 3:
        return 0.0
    dur = ts[-1] - ts[0]
    return 0.0 if dur <= 1e-6 else (len(ts) - 1) / dur

def _median_latency_ms() -> float:
    arr = sorted(_METRICS["t_hist"])
    if not arr:
        return 0.0
    n = len(arr); mid = n // 2
    med = arr[mid] if n % 2 else 0.5 * (arr[mid - 1] + arr[mid])
    return 1000.0 * med

# ---------- Lazy JPEG encoder ----------
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
            # Use fast DCT + 4:2:0 subsampling for speed
            return tj.encode(
                frame,
                quality=int(quality),
                jpeg_subsample=2,  # TurboJPEG.TJSAMP_420
                flags=512          # TurboJPEG.TJFLAG_FASTDCT
            )
        except Exception:
            pass
    ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    if not ok:
        return None
    return buf.tobytes()

# ---------- Threaded frame grabber (lazy singleton) ----------
_cam = None
def get_cam() -> ThreadedCamera | None:
    global _cam
    if _cam is None:
        try:
            cfg = CameraConfig(src=CAMERA_SRC)  # pulls the rest from env
            _cam = ThreadedCamera(cfg)
        except Exception:
            _cam = None
    return _cam

# ---------- Global CV + BLE wiring ----------
cvp = CVPipeline(cam=None)
bt_link = BTLink()

_cv_thread = None
_pose_thread = None
_pose_thread_stop = False

def _process_frame_once(frame) -> tuple[bool, float]:
    """
    Returns (valid_pose: bool, t_proc_seconds).
    Uses CVPipeline.process_frame if available; otherwise falls back to calibrate+track.
    """
    t0 = time.perf_counter()
    valid = False
    if hasattr(cvp, "process_frame"):
        try:
            valid, _t = cvp.process_frame(frame)
            t_proc = time.perf_counter() - t0
            return bool(valid), t_proc
        except Exception:
            pass
    # Fallback: periodic calibrate + track
    st = cvp.get_state()
    do_cal = (not st["calibrated"]) or (st["mm_per_px"] is None) or ((time.monotonic() % 2.0) < 0.05)
    if do_cal:
        try: cvp.calibrate_board(frame)
        except Exception: pass
    try:
        cvp.update_bot(frame)
    except Exception:
        pass
    st2 = cvp.get_state()
    valid = bool(st2["calibrated"] and st2["bot_pose"] is not None)
    t_proc = time.perf_counter() - t0
    return valid, t_proc

def _cv_worker():
    cam = get_cam()
    while True:
        if cam is None:
            time.sleep(0.02)
            cam = get_cam()
            continue

        # Prefer good/sharp frames; keep loop non-blocking
        frame = cam.get_latest_good()
        if frame is None:
            time.sleep(0.004)
            continue

        valid, tproc = _process_frame_once(frame)
        _note_frame(valid, tproc)

        time.sleep(0.004)

def _pose_publisher():
    global _pose_thread_stop
    while not _pose_thread_stop:
        pose = cvp.get_pose_mm()
        if pose is not None:
            (x_mm, y_mm), heading, conf = pose
            try:
                bt_link.send_pose(x_mm, y_mm, heading, conf)
            except Exception as e:
                print(f"[BT] send_pose error: {e}")
        time.sleep(0.08)  # ~12.5 Hz telemetry

def _ensure_cv_thread():
    global _cv_thread, _pose_thread
    if _cv_thread is None:
        _cv_thread = threading.Thread(target=_cv_worker, daemon=True)
        _cv_thread.start()
        try:
            if hasattr(bt_link, "connect"):
                bt_link.connect()
        except Exception as e:
            print(f"[BT] connect failed: {e}")
    if _pose_thread is None:
        _pose_thread = threading.Thread(target=_pose_publisher, daemon=True)
        _pose_thread.start()

# ---------- Overlay ----------
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

        line2 = f"Rate: {_rate_hz():.1f} Hz | Median cam→pose: {_median_latency_ms():.0f} ms | Uptime: {_METRICS['uptime_pct']:.1f}%"
        cv2.putText(frame, line1, (16, 32),  cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,255,0), 2, cv2.LINE_AA)
        cv2.putText(frame, line2, (16, 64),  cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,255,0), 2, cv2.LINE_AA)

        # Draw bot marker (project center back to image)
        if st["bot_pose"] is not None and st["calibrated"]:
            cx, cy = st["bot_pose"]["center_board_px"]
            bp = cvp.cal.board_pose
            if bp is not None:
                import numpy as np
                pt_board = np.float32([[cx, cy]])
                Hinv = np.linalg.inv(bp.H_img2board)
                pt_img = cv2.perspectiveTransform(pt_board[None], Hinv)[0][0]
                x, y = int(pt_img[0]), int(pt_img[1])
                cv2.circle(frame, (x, y), 8, (0, 255, 255), -1)
        return frame
    return _draw

# ---------- Streaming ----------
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

        yield (boundary +
               b"Content-Type: image/jpeg\r\n"
               b"Cache-Control: no-store\r\n\r\n" +
               jpg + b"\r\n")

@app.after_request
def _no_buffer(resp):
    resp.headers["Cache-Control"] = "no-store, no-cache, must-revalidate, max-age=0"
    resp.headers["Pragma"] = "no-cache"
    resp.headers["X-Accel-Buffering"] = "no"
    return resp

# ---------- Routes (UI) ----------
@app.route("/")
def home():
    _ensure_cv_thread()
    return render_template("index.html", page="home")

@app.route("/erase")
def erase():
    STATE["mode"] = "erase"
    STATE["target_image"] = None
    return render_template("index.html", page="erase")

@app.route("/draw", methods=["GET", "POST"])
def draw():
    if request.method == "POST":
        if "file" not in request.files or request.files["file"].filename == "":
            return render_template("index.html", page="draw", error="No file selected")
        f = request.files["file"]
        ts = int(time.time())
        save_path = os.path.join("uploads", f"{ts}_{f.filename}")
        f.save(save_path)
        STATE["mode"] = "draw"
        STATE["target_image"] = save_path
        return redirect(url_for("draw"))
    has_target = STATE["target_image"] is not None
    return render_template(
        "index.html", page="draw",
        has_target=has_target,
        target=os.path.basename(STATE["target_image"]) if has_target else None
    )

@app.route("/stream")
def stream():
    try:
        q = int(request.args.get("q", "65"))
    except ValueError:
        q = 65
    q = max(40, min(q, 90))
    return Response(
        gen_frames(overlay_fn, quality=q),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )

# ---------- Routes (API for firmware passthrough) ----------
@app.route("/api/moves", methods=["POST"])
def api_moves():
    _ensure_cv_thread()
    try:
        payload = request.get_json(force=True, silent=False)
        moves = parse_moves_payload(payload)  # list[MoveMM]
        path = [{"dx": m.dx_mm, "dy": m.dy_mm, **({"speed": m.speed_mm_s} if m.speed_mm_s else {})}
                for m in moves]
        if hasattr(bt_link, "send_path"):
            bt_link.send_path(path)
        return jsonify({"forwarded": len(path)})
    except Exception as e:
        return jsonify({"error": str(e)}), 400

@app.route("/api/pen", methods=["POST"])
def api_pen():
    data = request.get_json(force=True, silent=False) or {}
    down = bool(data.get("down", False))
    if hasattr(bt_link, "send_pen"):
        bt_link.send_pen(down)
    return jsonify({"msg": f"pen {'down' if down else 'up'}"})

@app.route("/api/stop", methods=["POST"])
def api_stop():
    try:
        if hasattr(bt_link, "stop"):
            bt_link.stop()
        return jsonify({"msg": "stop sent"})
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route("/health")
def health():
    cam = get_cam()
    st = cvp.get_state()
    cam_stats = cam.get_stats() if cam else {}
    return jsonify({
        "camera_ok": bool(cam and cam_stats.get("has_frame")),
        "turbojpeg": bool(_get_jpeg()),
        "camera_stats": cam_stats,
        "cv": {
            "calibrated": bool(st["calibrated"]),
            "mm_per_px": st["mm_per_px"],
            "reproj_err_px": st["board_reproj_err_px"],
            "bot_seen": bool(st["bot_pose"] is not None),
        }
    })

@app.route("/metrics.json")
def metrics():
    st = cvp.get_state()
    return jsonify({
        "rate_hz": round(_rate_hz(), 1),
        "median_latency_ms": round(_median_latency_ms(), 1),
        "uptime_pct": round(_METRICS["uptime_pct"], 1),
        "mm_per_px": st["mm_per_px"],
        "reproj_err_px": st["board_reproj_err_px"],
        "board_confidence": st["board_confidence"],
        "bot_seen": bool(st["bot_pose"] is not None),
        "platform": platform.platform(),
    })

# ---------- Graceful shutdown ----------
def _shutdown(*_):
    global _pose_thread_stop
    _pose_thread_stop = True
    try:
        if hasattr(bt_link, "close"):
            bt_link.close()
    except Exception:
        pass
    cam = get_cam()
    if cam:
        cam.close()
    os._exit(0)

signal.signal(signal.SIGINT, _shutdown)
signal.signal(signal.SIGTERM, _shutdown)

if __name__ == "__main__":
    try:
        import faulthandler; faulthandler.enable()
    except Exception:
        pass
    _ensure_cv_thread()
    # gunicorn -w1 --threads 8 -b 0.0.0.0:5000 app.server:app
    app.run(host="0.0.0.0", port=int(os.getenv("PORT", "5000")), threaded=True, debug=False)