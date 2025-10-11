# app/camera.py
from __future__ import annotations
import os, time, threading, platform, subprocess
from dataclasses import dataclass
from typing import Optional, Tuple, Dict, Any

import cv2
import numpy as np

@dataclass
class CameraConfig:
    src: int | str = 0
    width: int = int(os.getenv("CAMERA_W", "1280"))
    height: int = int(os.getenv("CAMERA_H", "720"))
    fps: int = int(os.getenv("CAMERA_FPS", "30"))
    downscale: int = int(os.getenv("CAMERA_DOWNSCALE", "1"))
    use_mjpeg: bool = os.getenv("CAMERA_USE_MJPEG", "1") not in ("0", "false", "False")
    buffer_size: int = int(os.getenv("CAMERA_BUFFER_SIZE", "1"))
    min_sharpness: Optional[float] = (
        float(os.getenv("CAMERA_MIN_SHARPNESS", "0")) if os.getenv("CAMERA_MIN_SHARPNESS") else None
    )
    # ROI as fractions "x0,y0,x1,y1" in [0,1], e.g. "0.05,0.05,0.95,0.95"
    roi_frac: Optional[Tuple[float, float, float, float]] = None

    def __post_init__(self):
        rf = os.getenv("CAMERA_ROI_FRAC", "")
        if rf:
            x0, y0, x1, y1 = [float(v) for v in rf.split(",")]
            self.roi_frac = (x0, y0, x1, y1)

def _fourcc(code: str) -> int:
    return cv2.VideoWriter_fourcc(*code)

def _crop_roi(img: np.ndarray, frac_roi: Tuple[float, float, float, float] | None) -> np.ndarray:
    if not frac_roi:
        return img
    h, w = img.shape[:2]
    x0, y0, x1, y1 = frac_roi
    x0i, y0i = max(0, int(x0 * w)), max(0, int(y0 * h))
    x1i, y1i = min(w, int(x1 * w)), min(h, int(y1 * h))
    if x1i <= x0i or y1i <= y0i:
        return img
    return img[y0i:y1i, x0i:x1i]

def _sharpness_var_laplacian(gray: np.ndarray) -> float:
    # Simple blur detector: variance of Laplacian (higher = sharper)
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())

def apply_linux_controls(dev: str = "/dev/video0") -> None:
    """Optionally lock exposure/gain/WB/focus using v4l2-ctl on Linux."""
    if platform.system().lower() != "linux":
        return
    if os.getenv("CAM_LOCK_PARAMS", "0") not in ("1", "true", "True"):
        return
    cmds = [
        ["v4l2-ctl", "-d", dev, "--set-ctrl=exposure_auto=1"],
        ["v4l2-ctl", "-d", dev, "--set-ctrl=exposure_absolute=80"],
        ["v4l2-ctl", "-d", dev, "--set-ctrl=gain=0"],
        ["v4l2-ctl", "-d", dev, "--set-ctrl=white_balance_automatic=0"],
        ["v4l2-ctl", "-d", dev, "--set-ctrl=white_balance_temperature=4500"],
        ["v4l2-ctl", "-d", dev, "--set-ctrl=focus_auto=0"],
        ["v4l2-ctl", "-d", dev, "--set-ctrl=focus_absolute=0"],
    ]
    for c in cmds:
        try:
            subprocess.run(c, check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception:
            pass

class ThreadedCamera:
    """
    Threaded capture with:
      - V4L2 (Linux) / AVFoundation (macOS) backend for low latency
      - MJPEG (if enabled) to reduce encode time in camera
      - Single buffer (drop old frames)
      - ROI cropping (fractional)
      - Laplacian sharpness gating
      - Monotonic timestamps
    """
    def __init__(self, cfg: Optional[CameraConfig] = None):
        self.cfg = cfg or CameraConfig()

        # Try to apply Linux UVC controls if requested
        if platform.system().lower() == "linux":
            devnum = self.cfg.src if isinstance(self.cfg.src, int) else 0
            apply_linux_controls(f"/dev/video{devnum}")
        else:
            apply_linux_controls()

        # Pick a low-latency backend
        sys = platform.system().lower()
        if sys == "linux":
            backend = cv2.CAP_V4L2
        elif hasattr(cv2, "CAP_AVFOUNDATION"):
            backend = cv2.CAP_AVFOUNDATION
        else:
            backend = 0

        self.cap = cv2.VideoCapture(self.cfg.src, backend) if backend else cv2.VideoCapture(self.cfg.src)

        # Request MJPEG and tight buffer *immediately* after open
        if self.cfg.use_mjpeg:
            try: self.cap.set(cv2.CAP_PROP_FOURCC, _fourcc("MJPG"))
            except Exception: pass
        try: self.cap.set(cv2.CAP_PROP_BUFFERSIZE, int(self.cfg.buffer_size))
        except Exception: pass

        # Basic capture props
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.cfg.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.cfg.height)
        self.cap.set(cv2.CAP_PROP_FPS,          self.cfg.fps)

        self.downscale = max(1, int(self.cfg.downscale))
        self._lock = threading.Lock()
        self._latest: Optional[np.ndarray] = None
        self._latest_ts: float = 0.0
        self._latest_sharpness: float = 0.0
        self._last_good: Optional[np.ndarray] = None
        self._last_good_ts: float = 0.0
        self.running = True
        self._thr = threading.Thread(target=self._loop, daemon=True)
        self._thr.start()

    def _loop(self):
        while self.running:
            ok, frame = self.cap.read()
            if not ok:
                time.sleep(0.005)
                continue

            # ROI crop first
            frame = _crop_roi(frame, self.cfg.roi_frac)

            # Downscale if requested
            if self.downscale > 1:
                frame = cv2.resize(
                    frame,
                    (frame.shape[1] // self.downscale, frame.shape[0] // self.downscale),
                    interpolation=cv2.INTER_AREA
                )

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            sharp = _sharpness_var_laplacian(gray)

            ts = time.monotonic()
            with self._lock:
                self._latest = frame
                self._latest_ts = ts
                self._latest_sharpness = sharp
                if self.cfg.min_sharpness is None or sharp >= self.cfg.min_sharpness:
                    self._last_good = frame
                    self._last_good_ts = ts

    def get_latest(self) -> Optional[np.ndarray]:
        with self._lock:
            return None if self._latest is None else self._latest.copy()

    def get_latest_good(self) -> Optional[np.ndarray]:
        with self._lock:
            src = self._last_good if self._last_good is not None else self._latest
            return None if src is None else src.copy()

    def get_latest_with_ts(self) -> tuple[Optional[np.ndarray], float]:
        with self._lock:
            if self._latest is None:
                return None, 0.0
            return self._latest.copy(), float(self._latest_ts)

    def get_stats(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "latest_ts": self._latest_ts,
                "latest_sharpness": self._latest_sharpness,
                "last_good_ts": self._last_good_ts,
                "has_frame": self._latest is not None,
                "has_good_frame": self._last_good is not None,
            }

    def close(self):
        self.running = False
        try:
            self._thr.join(timeout=0.5)
        except Exception:
            pass
        try:
            self.cap.release()
        except Exception:
            pass