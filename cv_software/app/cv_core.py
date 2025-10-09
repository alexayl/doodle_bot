# app/cv_core.py
from __future__ import annotations
from dataclasses import dataclass
from typing import Optional, Tuple, Dict
import numpy as np
import cv2
import math

# ---------------- Tunables ----------------
_POSE_ALPHA  = 0.25     # EMA for bot center/heading
_SCALE_ALPHA = 0.15     # EMA for mm/px

_REPROJ_GOOD_PX   = 1.0     # accept if reprojection ≤ 1px regardless of cond(H)
_COND_HARD_LIMIT  = 1e7     # only reject truly pathological homographies

# ---------------- Board/Bot IDs ----------------
BOARD_CORNERS = {"TL": 0, "TR": 1, "BR": 2, "BL": 3}
BOT_MARKERS   = (10, 11)    # two markers along robot body axis

# ---------------- Physical refs ----------------
BOT_BASELINE_MM = 60.0
KNOWN_BOARD_WIDTH_MM  = 1000.0
KNOWN_BOARD_HEIGHT_MM = 1460.0

# ---------------- Data classes ----------------
@dataclass
class CameraModel:
    K: Optional[np.ndarray] = None
    dist: Optional[np.ndarray] = None

@dataclass
class BoardPose:
    H_img2board: np.ndarray
    board_size_px: Tuple[int, int]
    mm_per_px: Optional[float] = None
    reproj_err_px: float = np.inf
    confidence: float = 0.0

@dataclass
class BotPose:
    center_board_px: Tuple[float, float]
    heading_rad: float
    pixel_baseline: float
    confidence: float = 0.0

# ---------------- ArUco compat ----------------
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
    """
    detect(gray) -> Dict[int, (4,2) float32 corners]
    (id_map only — corners/ids arrays are not returned)
    """
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
                id_map[int(i)] = c.reshape(-1, 2).astype(np.float32)  # (4,2)
        return id_map

# ---------------- Board calibration ----------------
class BoardCalibrator:
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
                have[name] = id_map[mid].mean(axis=0)  # (4,2)->(2,)
        return have

    def _complete_quad(self, cs: Dict[str, np.ndarray]) -> Dict[str, np.ndarray]:
        out = dict(cs); have = set(cs.keys())
        if len(have) < 3: return out
        if have >= {"TL", "TR", "BL"} and "BR" not in have:
            out["BR"] = cs["TR"] + (cs["BL"] - cs["TL"])
        elif have >= {"TL", "BL", "BR"} and "TR" not in have:
            out["TR"] = cs["TL"] + (cs["BR"] - cs["BL"])
        elif have >= {"TR", "BR", "TL"} and "BL" not in have:
            out["BL"] = cs["TL"] + (cs["BR"] - cs["TR"])
        elif have >= {"BL", "TL", "TR"} and "BR" not in have:
            out["BR"] = cs["TR"] + (cs["BL"] - cs["TL"])
        return out

    @staticmethod
    def _reproj_error(H: np.ndarray, img_pts: np.ndarray, dst_pts: np.ndarray) -> float:
        proj = cv2.perspectiveTransform(img_pts[None].astype(np.float32), H)[0]
        return float(np.mean(np.linalg.norm(proj - dst_pts.astype(np.float32), axis=1)))

    def calibrate(self, frame_bgr) -> Optional[BoardPose]:
        frame_bgr = self._undistort(frame_bgr)
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        id_map = self.det.detect(gray)

        cs = self._collect_corners(id_map)
        if len(cs) < 3:
            return self.board_pose  # keep previous if we had one

        cs = self._complete_quad(cs)
        if not all(k in cs for k in ("TL", "TR", "BR", "BL")):
            return self.board_pose

        img_pts = np.float32([cs["TL"], cs["TR"], cs["BR"], cs["BL"]])
        width_px  = float(np.linalg.norm(cs["TR"] - cs["TL"]))
        height_px = float(np.linalg.norm(cs["BL"] - cs["TL"]))
        width_px  = max(width_px, 1e-6)
        height_px = max(height_px, 1e-6)
        board_rect = np.float32([[0,0],[width_px,0],[width_px,height_px],[0,height_px]])

        H = cv2.getPerspectiveTransform(img_pts.astype(np.float32),
                                        board_rect.astype(np.float32))
        if H is None or abs(H[2,2]) < 1e-12:
            return self.board_pose
        H = H / H[2,2]

        condH = float(np.linalg.cond(H))
        reproj = self._reproj_error(H, img_pts, board_rect)
        if reproj > _REPROJ_GOOD_PX and condH > _COND_HARD_LIMIT:
            return self.board_pose  # reject truly bad H

        bp = BoardPose(
            H_img2board=H,
            board_size_px=(int(width_px), int(height_px)),
            mm_per_px=getattr(self.board_pose, "mm_per_px", None),
            reproj_err_px=reproj,
            confidence=float(np.clip((6.0 - reproj)/6.0, 0.0, 1.0)),
        )
        self.board_pose = bp
        return self.board_pose

# ---------------- Bot tracker ----------------
class BotTracker:
    def __init__(self, cal: BoardCalibrator):
        self.det = _ArucoCompatDetector()
        self.cal = cal

    @staticmethod
    def _center(c4): return c4.mean(axis=0)
    @staticmethod
    def _warp(H, pts_xy):
        return cv2.perspectiveTransform(pts_xy[None, :, :].astype(np.float32), H)[0]

    def track(self, frame_bgr) -> Optional[BotPose]:
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
        heading = float(np.arctan2(v[1], v[0]))
        heading = float(np.arctan2(np.sin(heading), np.cos(heading)))  # wrap [-pi, pi]
        pix_base = float(np.linalg.norm(v))
        conf = float(np.clip((pix_base / 40.0), 0.0, 1.0))
        return BotPose(center_board_px=(center[0], center[1]),
                       heading_rad=heading,
                       pixel_baseline=pix_base,
                       confidence=conf)

# ---------------- Scale estimator ----------------
class ScaleEstimator:
    def __init__(self, cal: BoardCalibrator):
        self.cal = cal

    def from_board_size(self) -> Optional[float]:
        bp = self.cal.board_pose
        if bp is None:
            return None
        Wpx, Hpx = bp.board_size_px
        sx = KNOWN_BOARD_WIDTH_MM  / float(Wpx)
        sy = KNOWN_BOARD_HEIGHT_MM / float(Hpx)
        mm_per_px = 0.5 * (sx + sy)
        self.cal.board_pose.mm_per_px = mm_per_px
        return mm_per_px

    def from_bot_baseline(self, bot: BotPose) -> Optional[float]:
        if bot.pixel_baseline <= 1e-6:
            return None
        mm_per_px = BOT_BASELINE_MM / bot.pixel_baseline
        if self.cal.board_pose:
            self.cal.board_pose.mm_per_px = mm_per_px
        return mm_per_px

# ---------------- Helpers ----------------
def board_pixels_to_mm(xy_board_px: np.ndarray, mm_per_px: float) -> np.ndarray:
    return xy_board_px.astype(np.float32) * float(mm_per_px)

def warp_points_board2img(H_img2board: np.ndarray, pts_xy_board: np.ndarray) -> np.ndarray:
    """Map points from board plane back to image pixels."""
    H = np.linalg.inv(H_img2board)
    return cv2.perspectiveTransform(pts_xy_board[None].astype(np.float32), H)[0]

def correct_delta_mm(desired_delta_mm, bot_heading_rad):
    dx, dy = desired_delta_mm
    c, s = np.cos(-bot_heading_rad), np.sin(-bot_heading_rad)
    fwd =  c*dx - s*dy
    lef =  s*dx + c*dy
    lef = -lef  # board y-down to left+
    return float(fwd), float(lef)

# ---------------- Facade ----------------
class CVPipeline:
    def __init__(self, cam: Optional[CameraModel] = None):
        self.cal = BoardCalibrator(cam)
        self.trk = BotTracker(self.cal)
        self.scale = ScaleEstimator(self.cal)
        self._bot: Optional[BotPose] = None
        self._mm_per_px_smooth: Optional[float] = None
        self._pose_smooth: Optional[BotPose] = None

    @staticmethod
    def _ema(old, new, a):
        return new if old is None else (a*old + (1.0-a)*new)

    def calibrate_board(self, frame_bgr) -> bool:
        return self.cal.calibrate(frame_bgr) is not None

    def update_bot(self, frame_bgr) -> bool:
        bot = self.trk.track(frame_bgr)
        if bot is None:
            return False
        self._bot = bot

        # mm/px from board size or bot baseline
        if self.cal.board_pose and self.cal.board_pose.mm_per_px is None:
            if self.scale.from_board_size() is None:
                self.scale.from_bot_baseline(bot)

        if self.cal.board_pose and self.cal.board_pose.mm_per_px is not None:
            self._mm_per_px_smooth = self._ema(self._mm_per_px_smooth,
                                               float(self.cal.board_pose.mm_per_px),
                                               _SCALE_ALPHA)
            self.cal.board_pose.mm_per_px = self._mm_per_px_smooth

        if self._pose_smooth is None:
            self._pose_smooth = bot
        else:
            cx = self._ema(self._pose_smooth.center_board_px[0], bot.center_board_px[0], _POSE_ALPHA)
            cy = self._ema(self._pose_smooth.center_board_px[1], bot.center_board_px[1], _POSE_ALPHA)
            dh = math.atan2(math.sin(bot.heading_rad - self._pose_smooth.heading_rad),
                            math.cos(bot.heading_rad - self._pose_smooth.heading_rad))
            hd = self._pose_smooth.heading_rad + (1.0 - _POSE_ALPHA) * dh
            self._pose_smooth = BotPose(center_board_px=(cx, cy),
                                        heading_rad=float(math.atan2(math.sin(hd), math.cos(hd))),
                                        pixel_baseline=bot.pixel_baseline,
                                        confidence=max(self._pose_smooth.confidence, bot.confidence))
        return True

    def get_state(self):
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

    def corrected_delta_for_bt(self, desired_delta_board_mm, rotate_into_bot_frame=False):
        bp = self.cal.board_pose
        if bp is None or bp.mm_per_px is None:
            return None
        dx, dy = float(desired_delta_board_mm[0]), float(desired_delta_board_mm[1])
        if not rotate_into_bot_frame:
            return (dx, dy)
        bot = (self._pose_smooth or self._bot)
        if bot is None:
            return (dx, dy)
        return correct_delta_mm((dx, dy), bot.heading_rad)