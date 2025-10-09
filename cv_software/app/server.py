# app/server.py
from __future__ import annotations
from flask import Flask, Response, request, jsonify, render_template, redirect, url_for
import cv2, os, time, threading, signal, math

# ---- Imports from your control + CV code ----
from app.cv_core import CVPipeline
from app.control import InstructionQueue, Coordinator, parse_moves_payload, PenCmd
from app.camera import ThreadedCamera, CameraConfig


# ---------- Config ----------
USE_TURBOJPEG = os.getenv("DISABLE_TURBOJPEG", "0") not in ("1", "true", "True")
CAMERA_SRC = int(os.getenv("CAMERA_SRC", "0"))

# ---------- App ----------
app = Flask(__name__, template_folder="../templates", static_folder="../static")
os.makedirs("uploads", exist_ok=True)

STATE = {"mode": "erase", "target_image": None}

# ---------- Lazy JPEG encoder ----------
_jpeg = None
def _get_jpeg():
    global _jpeg
    if _jpeg is not None: return _jpeg
    if not USE_TURBOJPEG:
        _jpeg = False; return _jpeg
    try:
        from turbojpeg import TurboJPEG
        _jpeg = TurboJPEG()
    except Exception:
        _jpeg = False
    return _jpeg

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

# ---------- Encoding ----------
def _encode_jpeg(frame, quality=70):
    tj = _get_jpeg()
    if tj:
        try:
            return tj.encode(frame, quality=int(quality))
        except Exception:
            pass
    ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    if not ok: return None
    return buf.tobytes()

# ---------- Simple BT link (replace with your real one) ----------
class BTLink:
    def connect(self): pass
    def close(self): pass
    def stop(self): print("[BT] stop")
    def send_move(self, fwd_mm: float, left_mm: float): print(f"[BT] move fwd={fwd_mm:.1f} left={left_mm:.1f}")
    def send_rotate(self, rot_rad: float): print(f"[BT] rotate {math.degrees(rot_rad):.1f} deg")
    def send_pen(self, down: bool): print(f"[BT] pen {'DOWN' if down else 'UP'}")

# ---------- Global CV + Coordinator wiring ----------
cvp = CVPipeline(cam=None)
instr_q = InstructionQueue(maxsize=10000)
bt_link = BTLink()
coord = Coordinator(instr_q, cvp, bt_link)

_cv_thread = None
def _cv_worker():
    cam = get_cam()
    last_calib = 0.0
    while True:
        if cam is None:
            time.sleep(0.05); cam = get_cam(); continue

        # Prefer "good" (sharp) frames when available
        frame = cam.get_latest_good()
        if frame is None:
            time.sleep(0.01); continue

        # Calibrate occasionally until good, then keep updating bot pose
        st = cvp.get_state()
        now = time.time()
        if (not st["calibrated"]) or (st["mm_per_px"] is None) or ((now - last_calib) > 2.0):
            ok = cvp.calibrate_board(frame)
            if ok: last_calib = now

        cvp.update_bot(frame)

        time.sleep(0.02)  # ~50 Hz loop max

def _ensure_cv_thread():
    global _cv_thread
    if _cv_thread is None:
        _cv_thread = threading.Thread(target=_cv_worker, daemon=True)
        _cv_thread.start()
        coord.start()

# ---------- Overlay ----------
def overlay_fn():
    def _draw(frame):
        st = cvp.get_state()
        label = f"Mode: {STATE['mode'].upper()}"
        if st["calibrated"]:
            try:
                label += f" | mm/px: {st['mm_per_px']:.3f} | reproj: {st['board_reproj_err_px']:.2f}"
            except Exception:
                pass
        else:
            label += " | CALIBRATING..."
        cv2.putText(frame, label, (16, 32),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2, cv2.LINE_AA)
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
def gen_frames(overlay_factory, quality=70):
    _ensure_cv_thread()
    draw = overlay_factory() if callable(overlay_factory) else overlay_factory
    boundary = b"--frame\r\n"
    while True:
        cam = get_cam()
        if cam is None:
            time.sleep(0.05); continue
        # For streaming we can show latest (not necessarily "good") to keep UI lively
        frame = cam.get_latest()
        if frame is None:
            time.sleep(0.01); continue
        if draw: frame = draw(frame)
        jpg = _encode_jpeg(frame, quality=quality)
        if jpg is None: continue
        yield boundary + b"Content-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n"

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
    try: q = int(request.args.get("q", "70"))
    except ValueError: q = 70
    return Response(
        gen_frames(overlay_fn, quality=max(40, min(q, 95))),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )

# ---------- Routes (API for control) ----------
@app.route("/api/moves", methods=["POST"])
def api_moves():
    _ensure_cv_thread()
    try:
        payload = request.get_json(force=True, silent=False)
        moves = parse_moves_payload(payload)
        for m in moves:
            instr_q.put_many([m])
        return jsonify({"enqueued": len(moves)})
    except Exception as e:
        return jsonify({"error": str(e)}), 400

@app.route("/api/pen", methods=["POST"])
def api_pen():
    data = request.get_json(force=True, silent=False) or {}
    down = bool(data.get("down", False))
    instr_q.put_many([PenCmd(down=down)])
    return jsonify({"msg": f"pen {'down' if down else 'up'}"})

@app.route("/api/stop", methods=["POST"])
def api_stop():
    try:
        if hasattr(bt_link, "stop"): bt_link.stop()
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

# ---------- Graceful shutdown ----------
def _shutdown(*_):
    try:
        coord.stop()
    except Exception: pass
    cam = get_cam()
    if cam: cam.close()
    os._exit(0)

signal.signal(signal.SIGINT, _shutdown)
signal.signal(signal.SIGTERM, _shutdown)

if __name__ == "__main__":
    try:
        import faulthandler; faulthandler.enable()
    except Exception:
        pass
    _ensure_cv_thread()
    app.run(host="0.0.0.0", port=int(os.getenv("PORT", "5000")), threaded=True, debug=False)
