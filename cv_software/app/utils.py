from __future__ import annotations
import time, cv2, numpy as np
from collections import deque
import math
# ---- JPEG ----
try:
    from turbojpeg import TurboJPEG
    _TJ = TurboJPEG()
except Exception:
    _TJ = None

def encode_jpeg(frame: np.ndarray, quality: int = 65) -> bytes | None:
    """
    Encodes a given image frame into JPEG format.

    Parameters:
    frame (np.ndarray): The image frame to be encoded, represented as a NumPy array.
    quality (int): The quality of the JPEG encoding, ranging from 0 to 100. Default is 65.

    Returns:
    bytes | None: The encoded JPEG image as bytes if successful, otherwise None.
    """
    if _TJ:
        try:
            return _TJ.encode(frame, quality=int(quality), jpeg_subsample=2, flags=512)
        except Exception:
            pass
    ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    return buf.tobytes() if ok else None
def board_to_robot(dx_board: float, dy_board: float, heading_rad: float) -> Tuple[float, float]:
    c = math.cos(-heading_rad)
    s = math.sin(-heading_rad)
    fwd = c * dx_board - s * dy_board
    lef = -(s * dx_board + c * dy_board)
    return float(fwd), float(lef)
# ---- Metrics ----
class Metrics:
    """Class to track and compute metrics related to processing frames."""
    
    def __init__(self, maxlen: int = 240):
        """Initialize the Metrics class with a maximum length for history."""
        self.t_hist = deque(maxlen=maxlen)
        self.ts_hist = deque(maxlen=maxlen)
        self.valid_frames = 0
        self.total_frames = 0
        self.uptime_pct = 0.0

    def note(self, valid: bool, t_proc_s: float):
        """Record the processing time and update frame counts."""
        self.t_hist.append(t_proc_s)
        self.ts_hist.append(time.monotonic())
        self.total_frames += 1
        if valid: self.valid_frames += 1
        self.uptime_pct = 100.0 * self.valid_frames / max(1, self.total_frames)

    def rate_hz(self) -> float:
        """Calculate the frame processing rate in Hertz."""
        ts = list(self.ts_hist)
        if len(ts) < 3: return 0.0
        dur = ts[-1] - ts[0]
        return 0.0 if dur <= 1e-6 else float(len(ts) - 1) / float(dur)

    def median_latency_ms(self) -> float:
        """Compute the median latency in milliseconds."""
        arr = sorted(self.t_hist)
        if not arr: return 0.0
        n = len(arr); mid = n // 2
        med = arr[mid] if (n % 2) else 0.5 * (arr[mid - 1] + arr[mid])
        return 1000.0 * med
