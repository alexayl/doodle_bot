from flask import Flask, Response, request, jsonify, render_template, redirect, url_for
import cv2, os, time

app = Flask(__name__, template_folder="../templates", static_folder="../static")
os.makedirs("uploads", exist_ok=True)

# Global state (simple for now)
STATE = {
    "mode": "erase",          # "erase" or "draw"
    "target_image": None,     # path to last uploaded target
}

# Camera
camera = cv2.VideoCapture(0)

def gen_frames(get_overlay):
    while True:
        ok, frame = camera.read()
        if not ok:
            break
        # Optional overlay (mode label, filename, etc.)
        frame = get_overlay(frame)
        _, buf = cv2.imencode(".jpg", frame)
        yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + buf.tobytes() + b"\r\n")

def overlay_fn():
    mode = STATE["mode"]
    fname = os.path.basename(STATE["target_image"]) if STATE["target_image"] else ""
    label = f"Mode: {mode.upper()}" + (f" | Target: {fname}" if fname else "")
    def _draw(frame):
        if label:
            cv2.putText(frame, label, (16, 32), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2, cv2.LINE_AA)
        return frame
    return _draw

@app.route("/")
def home():
    return render_template("index.html", page="home")

# ---------- ERASE MODE ----------
@app.route("/erase")
def erase():
    STATE["mode"] = "erase"
    STATE["target_image"] = None
    return render_template("index.html", page="erase")

# ---------- DRAW MODE ----------
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
        # (Later) call your pathfinding here with STATE["target_image"]
        return redirect(url_for("draw"))  # show stream after upload
    # GET -> show upload form (if none) or stream (if uploaded)
    has_target = STATE["target_image"] is not None
    return render_template("index.html", page="draw", has_target=has_target, target=os.path.basename(STATE["target_image"]) if has_target else None)

# ---------- SHARED STREAM ----------
@app.route("/stream")
def stream():
    return Response(gen_frames(overlay_fn()), mimetype="multipart/x-mixed-replace; boundary=frame")

# ---------- JSON API (optional) ----------
@app.route("/upload", methods=["POST"])
def upload():
    if "file" not in request.files or request.files["file"].filename == "":
        return jsonify({"error": "No file"}), 400
    f = request.files["file"]
    ts = int(time.time())
    save_path = os.path.join("uploads", f"{ts}_{f.filename}")
    f.save(save_path)
    STATE["mode"] = "draw"
    STATE["target_image"] = save_path
    return jsonify({"msg": "File received", "filename": os.path.basename(save_path)})

@app.route("/mode", methods=["POST"])
def mode():
    data = request.json or {}
    m = data.get("mode")
    if m in ("draw", "erase"):
        STATE["mode"] = m
        if m == "erase":
            STATE["target_image"] = None
        return jsonify({"msg": f"Mode set to {m}"})
    return jsonify({"error": "mode must be 'draw' or 'erase'"}), 400
