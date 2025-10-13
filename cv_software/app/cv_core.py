from __future__ import annotations
from dataclasses import dataclass
from typing import Optional, Tuple, Dict, List
import numpy as np
import cv2
import math
import time
from app.utils import board_to_robot
from app.geom import fit_path_to_board as _fit_to_board

_POSE_ALPHA = 0.25  #  factor for updating the pose estimate
_SCALE_ALPHA = 0.15  #  factor for updating the scale estimate
_CONF_MIN_BOARD = 0.60  # min confidence threshold for board detection
_CONF_MIN_BOT = 0.60  # min confidence threshold for bot detection

BOARD_CORNERS = {"TL": 0, "TR": 1, "BR": 2, "BL": 3}
BOT_MARKERS = (10, 11)

BOT_BASELINE_MM = 60.0
# testing on a board of this dims
KNOWN_BOARD_WIDTH_MM = 1000.0
KNOWN_BOARD_HEIGHT_MM = 1460.0

@dataclass
class CameraModel:
    """Optional camera intrinsics/distortion used to undistort frames before detection."""
    K: Optional[np.ndarray] = None
    dist: Optional[np.ndarray] = None

@dataclass
class BoardPose:
    """Image→board homography and quality metrics for the current calibration."""
    H_img2board: np.ndarray
    board_size_px: Tuple[int, int]
    mm_per_px: Optional[float] = None
    reproj_err_px: float = np.inf
    confidence: float = 0.0

@dataclass
class BotPose:
    """Robot pose in board pixels: center (px), heading (rad), inter-marker baseline (px), and confidence."""
    center_board_px: Tuple[float, float]
    heading_rad: float
    pixel_baseline: float
    confidence: float = 0.0

def _aruco_dict():
    return cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

def _aruco_params():
    if hasattr(cv2.aruco, "DetectorParameters"):
        p = cv2.aruco.DetectorParameters()
    else:
        p = cv2.aruco.DetectorParameters_create()
    if hasattr(p, "cornerRefinementMethod"):
        p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    if hasattr(p, "adaptiveThreshConstant"):
        p.adaptiveThreshConstant = 7
    return p

class _ArucoCompatDetector:
    """Wrapper that uses cv2.aruco’s APIs and returns {id: 4x2 corners}."""
    def __init__(self):
        self._dict = _aruco_dict()
        self._params = _aruco_params()
        self._new = hasattr(cv2.aruco, "ArucoDetector")
        self._det = cv2.aruco.ArucoDetector(self._dict, self._params) if self._new else None

    def detect(self, gray):
        if self._new:
            corners, ids, _ = self._det.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self._dict, parameters=self._params)
        id_map: Dict[int, np.ndarray] = {}
        if ids is not None:
            for c, i in zip(corners, ids.flatten()):
                id_map[int(i)] = c.reshape(-1, 2).astype(np.float32)
        return id_map

class BoardCalibrator:
    """Finds board corner markers, estimates image→board homography, and scores calibration quality."""
    def __init__(self, cam: Optional[CameraModel] = None):
        self.det = _ArucoCompatDetector()
        self.cam = cam or CameraModel()
        self.board_pose: Optional[BoardPose] = None

    def _undistort(self, frame_bgr):
        if self.cam.K is None or self.cam.dist is None:
            return frame_bgr
        h, w = frame_bgr.shape[:2]
        K_new, _ = cv2.getOptimalNewCameraMatrix(self.cam.K, self.cam.dist, (w, h), 1.0, (w, h))
        return cv2.undistort(frame_bgr, self.cam.K, self.cam.dist, None, K_new)

    def _collect_corners(self, id_map: Dict[int, np.ndarray]) -> Dict[str, np.ndarray]:
        have: Dict[str, np.ndarray] = {}
        for name, mid in BOARD_CORNERS.items():
            if mid in id_map:
                have[name] = id_map[mid].mean(axis=0)
        return have

    def _complete_quad(self, cs: Dict[str, np.ndarray]) -> Dict[str, np.ndarray]:
        out = dict(cs)
        have = set(out.keys())
        if len(have) < 3:
            return out
        if have >= {"TL", "TR", "BL"} and "BR" not in have:
            out["BR"] = out["TR"] + (out["BL"] - out["TL"])
        elif have >= {"TL", "BL", "BR"} and "TR" not in have:
            out["TR"] = out["TL"] + (out["BR"] - out["BL"])
        elif have >= {"TR", "BR", "TL"} and "BL" not in have:
            out["BL"] = out["TL"] + (out["BR"] - out["TR"])
        return out

    @staticmethod
    def _reproj_error(H: np.ndarray, img_pts: np.ndarray, dst_pts: np.ndarray) -> float:
        proj = cv2.perspectiveTransform(img_pts[None].astype(np.float32), H)[0]
        return float(np.mean(np.linalg.norm(proj - dst_pts.astype(np.float32), axis=1)))

    def calibrate(self, frame_bgr) -> Optional[BoardPose]:
        """Compute and cache image→board homography from a BGR frame; returns the new pose or last good one."""
        frame_bgr = self._undistort(frame_bgr)
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)

        id_map = self.det.detect(gray)
        cs = self._collect_corners(id_map)
        if len(cs) < 3:
            return self.board_pose

        cs = self._complete_quad(cs)
        if not all(k in cs for k in ("TL", "TR", "BR", "BL")):
            return self.board_pose

        tl, tr, br, bl = cs["TL"], cs["TR"], cs["BR"], cs["BL"]
        img_pts = np.array([tl, tr, br, bl], dtype=np.float32)

        width_px  = max(float(np.linalg.norm(tr - tl)), 1e-6)
        height_px = max(float(np.linalg.norm(bl - tl)), 1e-6)
        board_rect = np.array(
            [[0.0, 0.0], [width_px, 0.0], [width_px, height_px], [0.0, height_px]],
            dtype=np.float32
        )

        H = cv2.getPerspectiveTransform(img_pts.astype(np.float32), board_rect.astype(np.float32))
        if H is None or abs(H[2, 2]) < 1e-12:
            return self.board_pose
        H = H / H[2, 2]

        reproj = self._reproj_error(H, img_pts, board_rect)
        condH = float(np.linalg.cond(H))
        if (reproj > 1.0) and (condH > 1e7):
            return self.board_pose

        mm_per_px_prev = getattr(self.board_pose, "mm_per_px", None)
        self.board_pose = BoardPose(
            H_img2board=H,
            board_size_px=(int(width_px), int(height_px)),
            mm_per_px=mm_per_px_prev,
            reproj_err_px=float(reproj),
            confidence=float(np.clip((6.0 - reproj) / 6.0, 0.0, 1.0)),
        )
        return self.board_pose

class BotTracker:
    """Detects robot ArUco markers and outputs center/heading in the board coordinate frame."""
    def __init__(self, cal: BoardCalibrator):
        self.det = _ArucoCompatDetector()
        self.cal = cal

    @staticmethod
    def _center(c4):
        return c4.mean(axis=0)

    @staticmethod
    def _warp(H, pts_xy):
        return cv2.perspectiveTransform(pts_xy[None, :, :].astype(np.float32), H)[0]

    def track(self, frame_bgr) -> Optional[BotPose]:
        """Detect markers 10/11, transform to board space, compute center, heading, and confidence."""
        if self.cal.board_pose is None:
            return None

        frame_bgr = self.cal._undistort(frame_bgr)
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        id_map = self.det.detect(gray)
        if BOT_MARKERS[0] not in id_map or BOT_MARKERS[1] not in id_map:
            return None

        H = self.cal.board_pose.H_img2board
        c0 = self._center(id_map[BOT_MARKERS[0]])
        c1 = self._center(id_map[BOT_MARKERS[1]])
        pair_board = self._warp(H, np.vstack([c0, c1]))

        v = pair_board[1] - pair_board[0]
        center = 0.5 * (pair_board[0] + pair_board[1])
        heading = float(np.arctan2(v[1], v[0]))  # [-pi, pi]
        pix_base = float(np.linalg.norm(v))
        conf = float(np.clip((pix_base / 40.0), 0.0, 1.0))

        return BotPose(
            center_board_px=(center[0], center[1]),
            heading_rad=heading,
            pixel_baseline=pix_base,
            confidence=conf,
        )

class ScaleEstimator:
    """Computes mm/px scale from either known board size or known robot marker baseline."""
    def __init__(self, cal: BoardCalibrator):
        self.cal = cal

    def from_board_size(self) -> Optional[float]:
        """Estimate mm/px using known physical board dimensions."""
        bp = self.cal.board_pose
        if bp is None:
            return None
        Wpx, Hpx = bp.board_size_px
        sx = KNOWN_BOARD_WIDTH_MM / float(Wpx)
        sy = KNOWN_BOARD_HEIGHT_MM / float(Hpx)
        mm_per_px = 0.5 * (sx + sy)
        self.cal.board_pose.mm_per_px = mm_per_px
        return mm_per_px

    def from_bot_baseline(self, bot: BotPose) -> Optional[float]:
        """Estimate mm/px using the two-Tag baseline on the robot."""
        if bot.pixel_baseline <= 1e-6:
            return None
        mm_per_px = BOT_BASELINE_MM / bot.pixel_baseline
        if self.cal.board_pose:
            self.cal.board_pose.mm_per_px = mm_per_px
        return mm_per_px

class CVPipeline:
    """Top-level CV pipeline: board calibration, bot tracking, scale smoothing, and API to query state."""
    def __init__(self, cam: Optional[CameraModel] = None):
        self.cal = BoardCalibrator(cam)
        self.trk = BotTracker(self.cal)
        self.scale = ScaleEstimator(self.cal)
        self._bot: Optional[BotPose] = None
        self._mm_per_px_smooth: Optional[float] = None
        self._pose_smooth: Optional[BotPose] = None

    @staticmethod
    def _ema(old, new, a):
        """Simple exponential moving average helper (returns new if old is None)."""
        if old is None:
            return new
        return a * old + (1.0 - a) * new
    def _ema_angle(prev: float, new: float, a: float) -> float:
        # move a fraction (1-a) toward new, with proper wrap-around
        d = math.atan2(math.sin(new - prev), math.cos(new - prev))
        h = prev + (1.0 - a) * d
        return math.atan2(math.sin(h), math.cos(h))  # wrap to [-pi, pi]
    def calibrate_board(self, frame_bgr) -> bool:
        """Run a single-board calibration step from a frame; returns True if a pose is available."""
        return self.cal.calibrate(frame_bgr) is not None

    def update_bot(self, frame_bgr) -> bool:
        """Track the robot for the given frame and update smoothed pose and mm/px scale if available."""
        bot = self.trk.track(frame_bgr)
        if bot is None:
            return False
        self._bot = bot

        if self.cal.board_pose and self.cal.board_pose.mm_per_px is None:
            self.scale.from_board_size()

        if self.cal.board_pose and self.cal.board_pose.mm_per_px is not None:
            self._mm_per_px_smooth = self._ema(self._mm_per_px_smooth, float(self.cal.board_pose.mm_per_px), _SCALE_ALPHA)
            self.cal.board_pose.mm_per_px = self._mm_per_px_smooth

        if self._pose_smooth is None:
            self._pose_smooth = bot
        else:
            cx = self._ema(self._pose_smooth.center_board_px[0], bot.center_board_px[0], _POSE_ALPHA)
            cy = self._ema(self._pose_smooth.center_board_px[1], bot.center_board_px[1], _POSE_ALPHA)
            hd = _ema_angle(self._pose_smooth.heading_rad, bot.heading_rad, _POSE_ALPHA)

            self._pose_smooth = BotPose(
                center_board_px=(cx, cy),
                heading_rad=hd,
                pixel_baseline=bot.pixel_baseline,
                confidence=max(self._pose_smooth.confidence, bot.confidence),
            )
        return True

    def process_frame(self, frame_bgr) -> tuple[bool, float]:
        """One-shot update: ensure calibration, update bot, and return (valid_pose, latency_s)."""
        t0 = time.monotonic()
        st = self.get_state()
        needs_cal = not st["calibrated"] or st["mm_per_px"] is None
        if needs_cal or (st["board_confidence"] is not None and st["board_confidence"] < _CONF_MIN_BOARD):
            self.calibrate_board(frame_bgr)
        _ = self.update_bot(frame_bgr)
        st = self.get_state()
        valid = bool(
            st["calibrated"]
            and st["mm_per_px"] is not None
            and st["bot_pose"] is not None
            and (st["board_confidence"] is None or st["board_confidence"] >= _CONF_MIN_BOARD)
            and st["bot_pose"]["confidence"] >= _CONF_MIN_BOT
        )
        return valid, time.monotonic() - t0

    def get_state(self):
        """Return a serializable snapshot of current calibration, scale, and (smoothed) bot pose."""
        bp = self.cal.board_pose
        bot = self._pose_smooth or self._bot
        return {
            "calibrated": bp is not None,
            "mm_per_px": None if bp is None else bp.mm_per_px,
            "board_size_px": None if bp is None else bp.board_size_px,
            "board_reproj_err_px": None if bp is None else bp.reproj_err_px,
            "board_confidence": None if bp is None else bp.confidence,
            "bot_pose": None if bot is None else {
                "center_board_px": bot.center_board_px,
                "heading_rad": bot.heading_rad,
                "pixel_baseline": bot.pixel_baseline,
                "confidence": bot.confidence,
            },
        }

    def board_size_mm(self) -> Optional[Tuple[float, float]]:
        """Board physical size in mm using current mm/px; None if not calibrated or unscaled."""
        st = self.get_state()
        if not st["calibrated"] or st["mm_per_px"] is None or st["board_size_px"] is None:
            return None
        mpp = float(st["mm_per_px"])
        Wpx, Hpx = st["board_size_px"]
        return mpp * float(Wpx), mpp * float(Hpx)

    def fit_path_to_board(self, wps, margin_frac: float = 0.10):
        """Scale/center normalized waypoints to the calibrated board."""
        sz = self.board_size_mm()
        if not sz:
            return []
        return _fit_to_board(wps, sz, margin_frac)

    def corrected_delta_for_bt(self, desired_delta_board_mm, rotate_into_bot_frame=False):
        """Optionally rotate (dx,dy) in BOARD mm into ROBOT frame using the latest smoothed heading."""
        bp = self.cal.board_pose
        if bp is None or bp.mm_per_px is None:
            return None
        dx, dy = float(desired_delta_board_mm[0]), float(desired_delta_board_mm[1])
        if not rotate_into_bot_frame:
            return dx, dy
        bot = self._pose_smooth or self._bot
        if bot is None:
            return dx, dy
        return board_to_robot(dx, dy, bot.heading_rad)

    def get_pose_mm(self):
        """Smoothed robot pose in mm and heading/confidence, or None if not ready."""
        st = self.get_state()
        if not st["calibrated"] or st["bot_pose"] is None or st["mm_per_px"] is None:
            return None
        px = st["bot_pose"]["center_board_px"]
        mpp = float(st["mm_per_px"])
        return (px[0] * mpp, px[1] * mpp), float(st["bot_pose"]["heading_rad"]), float(st["bot_pose"]["confidence"])
