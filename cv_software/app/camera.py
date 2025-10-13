from __future__ import annotations
import os, time, threading, platform, subprocess
from dataclasses import dataclass
from typing import Optional, Tuple, Dict, Any
import cv2, numpy as np

# ----------------------- Config -----------------------

@dataclass
class CameraConfig:
    """Runtime-configurable camera settings and optional ROI/quality gating."""
    src: int | str = 0
    width: int = int(os.getenv("CAMERA_W", "1280"))
    height: int = int(os.getenv("CAMERA_H", "720"))
    fps: int = int(os.getenv("CAMERA_FPS", "30"))
    downscale: int = int(os.getenv("CAMERA_DOWNSCALE", "1"))
    use_mjpeg: bool = os.getenv("CAMERA_USE_MJPEG", "1").lower() not in ("0", "false")
    buffer_size: int = int(os.getenv("CAMERA_BUFFER_SIZE", "1"))
    min_sharpness: Optional[float] = (
        float(os.getenv("CAMERA_MIN_SHARPNESS")) if os.getenv("CAMERA_MIN_SHARPNESS") else None
    )
    roi_frac: Optional[Tuple[float, float, float, float]] = None  # (x0,y0,x1,y1), 0..1

    def __post_init__(self):
        """Parse ROI from env if present (x0,y0,x1,y1 in [0..1])."""
        rf = os.getenv("CAMERA_ROI_FRAC")
        if rf:
            x0, y0, x1, y1 = map(float, rf.split(","))
            self.roi_frac = (x0, y0, x1, y1)

# ----------------------- Helpers -----------------------

def _apply_linux_controls(dev: str) -> None:
    """Optionally lock v4l2 controls on Linux for stable exposure, WB, and focus."""
    if platform.system().lower() != "linux":
        return
    if os.getenv("CAM_LOCK_PARAMS", "0").lower() not in ("1", "true"):
        return
    for args in (
        ("--set-ctrl=exposure_auto=1",),
        ("--set-ctrl=exposure_absolute=80",),
        ("--set-ctrl=gain=0",),
        ("--set-ctrl=white_balance_automatic=0",),
        ("--set-ctrl=white_balance_temperature=4500",),
        ("--set-ctrl=focus_auto=0",),
        ("--set-ctrl=focus_absolute=0",),
    ):
        try:
            subprocess.run(["v4l2-ctl", "-d", dev, *args], check=False,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception:
            pass

def _crop_roi(img: np.ndarray, roi: Optional[Tuple[float, float, float, float]]) -> np.ndarray:
    """Crop a fractional ROI (x0,y0,x1,y1 in [0..1]) from an image if provided."""
    if not roi:
        return img
    h, w = img.shape[:2]
    x0, y0, x1, y1 = roi
    x0i, y0i = max(0, int(x0 * w)), max(0, int(y0 * h))
    x1i, y1i = min(w, int(x1 * w)), min(h, int(y1 * h))
    return img if (x1i <= x0i or y1i <= y0i) else img[y0i:y1i, x0i:x1i]

def _sharpness(gray: np.ndarray) -> float:
    """Variance of Laplacian sharpness metric (higher is sharper)."""
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())

# ----------------------- Camera -----------------------

class ThreadedCamera:
    """
    Background camera reader with optional ROI, downscale, and sharpness gating.
    Exposes:
      - get_latest_with_ts(): latest frame and monotonic timestamp
      - get_latest_good(): last frame that met min_sharpness (or latest if None)
      - get_stats(): small dict of timing/sharpness flags for UI/metrics
    """
    def __init__(self, cfg: Optional[CameraConfig] = None):
        self.cfg = cfg or CameraConfig()
        devnum = self.cfg.src if isinstance(self.cfg.src, int) else 0
        _apply_linux_controls(f"/dev/video{devnum}")

        backend = cv2.CAP_V4L2 if platform.system().lower() == "linux" else getattr(cv2, "CAP_AVFOUNDATION", 0)
        self.cap = cv2.VideoCapture(self.cfg.src, backend) if backend else cv2.VideoCapture(self.cfg.src)

        if self.cfg.use_mjpeg:
            try: self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            except Exception: 
                pass
        for prop, val in (
            (cv2.CAP_PROP_BUFFERSIZE, int(self.cfg.buffer_size)),
            (cv2.CAP_PROP_FRAME_WIDTH, self.cfg.width),
            (cv2.CAP_PROP_FRAME_HEIGHT, self.cfg.height),
            (cv2.CAP_PROP_FPS, self.cfg.fps),
        ):
            try: 
                self.cap.set(prop, val)
            except Exception: 
                pass

        self.downscale = max(1, int(self.cfg.downscale))
        self._lock = threading.Lock()
        self._latest: Optional[np.ndarray] = None
        self._latest_ts: float = 0.0
        self._latest_sharp: float = 0.0
        self._last_good: Optional[np.ndarray] = None
        self._last_good_ts: float = 0.0
        self.running = True

        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        """Capture loop that updates latest and last_good frames continuously."""
        while self.running:
            ok, frame = self.cap.read()
            if not ok:
                time.sleep(0.005)
                continue

            frame = _crop_roi(frame, self.cfg.roi_frac)
            if self.downscale > 1:
                frame = cv2.resize(frame, (frame.shape[1] // self.downscale, frame.shape[0] // self.downscale),
                                   interpolation=cv2.INTER_AREA)

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            sharp = _sharpness(gray)
            ts = time.monotonic()

            with self._lock:
                self._latest, self._latest_ts, self._latest_sharp = frame, ts, sharp
                if self.cfg.min_sharpness is None or sharp >= self.cfg.min_sharpness:
                    self._last_good, self._last_good_ts = frame, ts

    def get_latest_with_ts(self) -> Tuple[Optional[np.ndarray], float]:
        """Return a copy of the most recent frame and its timestamp (or (None,0.0))."""
        with self._lock:
            return (None, 0.0) if self._latest is None else (self._latest.copy(), self._latest_ts)

    def get_latest_good(self) -> Optional[np.ndarray]:
        """Return a copy of the last frame that met min_sharpness (or latest if threshold is None)."""
        with self._lock:
            src = self._last_good if self._last_good is not None else self._latest
            return None if src is None else src.copy()

    def get_stats(self) -> Dict[str, Any]:
        """Return small diagnostic stats useful for overlays and health checks."""
        with self._lock:
            return {
                "latest_ts": self._latest_ts,
                "latest_sharpness": self._latest_sharp,
                "last_good_ts": self._last_good_ts,
                "has_frame": self._latest is not None,
                "has_good_frame": self._last_good is not None,
            }

    def close(self):
        """Stop the capture loop and release the camera device."""
        self.running = False
        try: 
            self.cap.release()
        except Exception:
            pass
