from __future__ import annotations
from dataclasses import dataclass
from typing import Optional, Tuple, Dict, List, Callable
import numpy as np
import cv2
import math
import time
from app.utils import board_to_robot
from app.geom import fit_path_to_board as _fit_to_board
import os
from app.path_parse import (
    load_gcode_file,
    convert_pathfinding_gcode,
    scale_gcode_to_board,
)

# Bounds margin configuration (can be overridden via environment)
# INCREASED margins to keep robot well within marker zone
BOARD_MARGIN_MM = float(os.getenv("BOARD_MARGIN_MM", "67"))
BOARD_MARGIN_FRAC = float(os.getenv("BOARD_MARGIN_FRAC", "0.20"))
_POSE_ALPHA = 0.7  # factor for updating the pose estimate - higher = faster response
_SCALE_ALPHA = 0.15  # factor for updating the scale estimate
_CONF_MIN_BOARD = 0.60  # min confidence threshold for board detection
_CONF_MIN_BOT = 0.30  # relaxed min confidence threshold for bot detection (fallback tolerant)

# One-time deprecation warnings registry
_DEPRECATION_WARNED: set[str] = set()

def _warn_deprecated(name: str, alt: str = "cvp.correction.compute_correction"):
    try:
        if name not in _DEPRECATION_WARNED:
            print(f"DEPRECATED: {name} will be removed in a future release; use {alt} instead.")
            _DEPRECATION_WARNED.add(name)
    except Exception:
        pass

# CV Correction tuning
CV_CORRECTION_RATE_HZ = float(os.getenv("CV_CORRECTION_RATE_HZ", "7.0"))
CV_MIN_ERROR_MM = float(os.getenv("CV_MIN_ERROR_MM", "2.5"))
CV_LOOKAHEAD_COUNT = int(os.getenv("CV_LOOKAHEAD_COUNT", "2"))
CV_CROSS_TRACK_GAIN = float(os.getenv("CV_CROSS_TRACK_GAIN", "2.0"))
CV_ADAPTIVE_STEP = bool(int(os.getenv("CV_ADAPTIVE_STEP", "1")))

BOARD_CORNERS = {"TL": 0, "TR": 1, "BR": 2, "BL": 3}
BOT_MARKERS = (10, 11)

# Options: "4x4_50", "5x5_100", "6x6_250", "apriltag_36h11"
MARKER_DICT = os.getenv("MARKER_DICT", "4x4_100").upper()  # Changed from 5x5_100 to 4x4_100

BOT_BASELINE_MM = 100.0

# Nominal board dimensions used to define the homography coordinate frame.
# We now treat these as actual physical board dimensions.
KNOWN_BOARD_WIDTH_MM = float(800)
KNOWN_BOARD_HEIGHT_MM = float(400)

_MAX_REPROJ_ERR_PX = 1.2
_MAX_SIZE_JUMP_FRAC = 0.15
_MAX_CENTER_JUMP_PX = 40.0

# Thresholds expected by tests
_REPROJ_GOOD_PX = 0.5
_COND_HARD_LIMIT = 5e7
_COND_WARN_LIMIT = 3e4

# Robust heading estimation parameters (used in BotTracker)
_HEADING_RANSAC_MAX_DEG = float(os.getenv("HEADING_RANSAC_MAX_DEG", "10.0"))
_HEADING_RANSAC_MIN_SAMPLES = int(os.getenv("HEADING_RANSAC_MIN_SAMPLES", "5"))
_HEADING_HISTORY_SIZE = int(os.getenv("HEADING_HISTORY_SIZE", "30"))


def correct_delta_mm(delta_board_mm: tuple[float, float], heading_rad: float) -> tuple[float, float]:
    """Rotate a board-frame delta (dx,dy) into robot forward/left components."""
    dx = float(delta_board_mm[0]); dy = float(delta_board_mm[1])
    c = math.cos(float(heading_rad)); s = math.sin(float(heading_rad))
    fwd = dx * c + dy * s
    left = -dx * s + dy * c
    return fwd, left


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
    corners_board_px: Optional[Dict[str, Tuple[float, float]]] = None


@dataclass
class BotPose:
    """Robot pose in board units (treated as mm): center, heading, baseline, and confidence."""
    center_board_mm: Tuple[float, float]
    heading_rad: float
    baseline_mm: float
    confidence: float = 0.0


def _aruco_dict():
    """Get ArUco/AprilTag dictionary based on MARKER_DICT setting"""
    dict_name = MARKER_DICT.replace("X", "x")

    dict_map = {
        "4x4_50": cv2.aruco.DICT_4X4_50,
        "4x4_100": cv2.aruco.DICT_4X4_100,
        "4x4_250": cv2.aruco.DICT_4X4_250,
        "4x4_1000": cv2.aruco.DICT_4X4_1000,
        "5x5_50": cv2.aruco.DICT_5X5_50,
        "5x5_100": cv2.aruco.DICT_5X5_100,
        "5x5_250": cv2.aruco.DICT_5X5_250,
        "5x5_1000": cv2.aruco.DICT_5X5_1000,
        "6x6_50": cv2.aruco.DICT_6X6_50,
        "6x6_100": cv2.aruco.DICT_6X6_100,
        "6x6_250": cv2.aruco.DICT_6X6_250,
        "6x6_1000": cv2.aruco.DICT_6X6_1000,
        "7x7_50": cv2.aruco.DICT_7X7_50,
        "7x7_100": cv2.aruco.DICT_7X7_100,
        "7x7_250": cv2.aruco.DICT_7X7_250,
        "7x7_1000": cv2.aruco.DICT_7X7_1000,
    }

    if hasattr(cv2.aruco, 'DICT_APRILTAG_36h11'):
        dict_map.update({
            "apriltag_36h11": cv2.aruco.DICT_APRILTAG_36h11,
            "apriltag_36h10": cv2.aruco.DICT_APRILTAG_36h10,
            "apriltag_25h9": cv2.aruco.DICT_APRILTAG_25h9,
            "apriltag_16h5": cv2.aruco.DICT_APRILTAG_16h5,
        })

    dict_type = dict_map.get(dict_name.lower(), cv2.aruco.DICT_5X5_100)
    return cv2.aruco.getPredefinedDictionary(dict_type)


def _aruco_params():
    """Improved ArUco detector parameters for high-angle camera views."""
    if hasattr(cv2.aruco, "DetectorParameters"):
        p = cv2.aruco.DetectorParameters()
    else:
        p = cv2.aruco.DetectorParameters_create()

    if hasattr(p, "cornerRefinementMethod"):
        p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    if hasattr(p, "cornerRefinementMaxIterations"):
        p.cornerRefinementMaxIterations = 50
    if hasattr(p, "cornerRefinementMinAccuracy"):
        p.cornerRefinementMinAccuracy = 0.01

    if hasattr(p, "minMarkerPerimeterRate"):
        p.minMarkerPerimeterRate = 0.01
    if hasattr(p, "maxMarkerPerimeterRate"):
        p.maxMarkerPerimeterRate = 8.0

    if hasattr(p, "adaptiveThreshWinSizeMin"):
        p.adaptiveThreshWinSizeMin = 5
    if hasattr(p, "adaptiveThreshWinSizeMax"):
        p.adaptiveThreshWinSizeMax = 55
    if hasattr(p, "adaptiveThreshWinSizeStep"):
        p.adaptiveThreshWinSizeStep = 10
    if hasattr(p, "adaptiveThreshConstant"):
        p.adaptiveThreshConstant = 7

    if hasattr(p, "polygonalApproxAccuracyRate"):
        p.polygonalApproxAccuracyRate = 0.05

    if hasattr(p, "minCornerDistanceRate"):
        p.minCornerDistanceRate = 0.01
    if hasattr(p, "maxErroneousBitsInBorderRate"):
        p.maxErroneousBitsInBorderRate = 0.6

    if hasattr(p, "minOtsuStdDev"):
        p.minOtsuStdDev = 2.0

    return p


class _ArucoCompatDetector:
    """Wrapper that uses cv2.aruco APIs and returns {id: 4x2 corners}."""
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


def _conf_from_visibility(found_corners: int, total: int = 4) -> float:
    """0.0 if <3 corners; otherwise linear with count."""
    if found_corners < 3:
        return 0.0
    return min(1.0, found_corners / float(total))


class BoardCalibrator:
    """Finds board corner markers, estimates homography, and scores calibration quality."""
    def __init__(self, cam: Optional[CameraModel] = None):
        self.det = _ArucoCompatDetector()
        self.cam = cam or CameraModel()
        self.board_pose: Optional[BoardPose] = None
        self.pose_fresh: bool = False
        self.calibration_locked: bool = False

    def _center_from_quad(self, tl, tr, br, bl) -> Tuple[float, float]:
        c = 0.25 * (tl + tr + br + bl)
        return float(c[0]), float(c[1])

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
        elif have >= {"TR", "BR", "BL"} and "TL" not in have:
            out["TL"] = out["BL"] + (out["TR"] - out["BR"])
        return out

    @staticmethod
    def _reproj_error(H: np.ndarray, img_pts: np.ndarray, dst_pts: np.ndarray) -> float:
        proj = cv2.perspectiveTransform(img_pts[None].astype(np.float32), H)[0]
        return float(np.mean(np.linalg.norm(proj - dst_pts.astype(np.float32), axis=1)))

    def calibrate(self, frame_bgr, force_recalibrate: bool = False) -> Optional[BoardPose]:
        """Compute and cache image→board homography; confidence = visibility only."""
        self.pose_fresh = False

        if self.calibration_locked and not force_recalibrate:
            if self.board_pose is not None and self.board_pose.confidence >= 0.5:
                return self.board_pose
            if not hasattr(self, '_quality_degrade_log') or time.time() - self._quality_degrade_log > 5.0:
                print(f"Calibration quality degraded (conf={self.board_pose.confidence:.2f}) - attempting recalibration")
                self._quality_degrade_log = time.time()
                self.calibration_locked = False

        frame_bgr = self._undistort(frame_bgr)
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        det_out = self.det.detect(gray)
        if isinstance(det_out, tuple) and len(det_out) >= 3:
            id_map = det_out[2]
        else:
            id_map = det_out
        if id_map is None:
            id_map = {}
        cs = self._collect_corners(id_map)
        if len(cs) < 3:
            if not hasattr(self, '_last_reject_log') or time.time() - self._last_reject_log > 2.0:
                print(f"Calibration: {len(cs)}/4 corners (need ≥3)")
                self._last_reject_log = time.time()
            return self.board_pose

        if len(cs) == 3:
            cs = self._complete_quad(cs)
            if not hasattr(self, '_last_completion_log') or time.time() - self._last_completion_log > 3.0:
                print(f"Calibration: Completed quad from 3 corners")
                self._last_completion_log = time.time()

        if not all(k in cs for k in ("TL", "TR", "BR", "BL")):
            return self.board_pose

        tl, tr, br, bl = cs["TL"], cs["TR"], cs["BR"], cs["BL"]
        img_pts = np.array([tl, tr, br, bl], dtype=np.float32)

        board_rect = np.array(
            [
                [0.0, KNOWN_BOARD_HEIGHT_MM],
                [KNOWN_BOARD_WIDTH_MM, KNOWN_BOARD_HEIGHT_MM],
                [KNOWN_BOARD_WIDTH_MM, 0.0],
                [0.0, 0.0],
            ],
            dtype=np.float32,
        )

        H = cv2.getPerspectiveTransform(img_pts.astype(np.float32), board_rect.astype(np.float32))
        if H is None:
            return self.board_pose
        if abs(H[2, 2]) > 1e-12:
            H = H / H[2, 2]

        reproj = self._reproj_error(H, img_pts, board_rect)
        found = sum(1 for k in ("TL", "TR", "BR", "BL") if k in cs)
        conf_raw = _conf_from_visibility(found)
        prev = self.board_pose

        width_px = float(np.linalg.norm(tr - tl))
        height_px = float(np.linalg.norm(bl - tl))

        cond_H = np.linalg.cond(H)

        if cond_H > _COND_HARD_LIMIT:
            if not hasattr(self, '_last_cond_reject') or time.time() - self._last_cond_reject > 3.0:
                print(f"Calibration REJECTED: condition number {cond_H:.1f} exceeds limit {_COND_HARD_LIMIT:.0f}")
                print("  -> Camera angle too extreme or markers nearly collinear")
                print("  -> Adjust camera to view board more directly")
                self._last_cond_reject = time.time()
            return self.board_pose

        if cond_H > _COND_WARN_LIMIT:
            if not hasattr(self, '_last_cond_warn') or time.time() - self._last_cond_warn > 5.0:
                print(f"Calibration WARNING: condition number {cond_H:.1f} is high (>{_COND_WARN_LIMIT:.0f})")
                print("  -> Consider adjusting camera angle for better stability")
                self._last_cond_warn = time.time()

        accept = not (reproj > _REPROJ_GOOD_PX and cond_H > _COND_HARD_LIMIT)
        rejection_reason = None

        if prev is not None and prev.board_size_px is not None:
            W0, H0 = prev.board_size_px
            if W0 > 0 and H0 > 0:
                w_jump = abs(width_px - W0) / max(W0, 1e-6)
                h_jump = abs(height_px - H0) / max(H0, 1e-6)
                if (w_jump > _MAX_SIZE_JUMP_FRAC) or (h_jump > _MAX_SIZE_JUMP_FRAC):
                    accept = False
                    rejection_reason = f"size_jump w={w_jump:.2%} h={h_jump:.2%}"
            c_new = np.array(self._center_from_quad(tl, tr, br, bl))

            try:
                Hinv = np.linalg.inv(prev.H_img2board)
                board_center = np.array(
                    [[0.5 * KNOWN_BOARD_WIDTH_MM, 0.5 * KNOWN_BOARD_HEIGHT_MM]],
                    dtype=np.float32,
                )[None]
                img_center_prev = cv2.perspectiveTransform(board_center, Hinv)[0][0]
                center_jump = float(np.linalg.norm(c_new - img_center_prev))
                if center_jump > _MAX_CENTER_JUMP_PX:
                    accept = False
                    rejection_reason = f"center_jump {center_jump:.1f}px > {_MAX_CENTER_JUMP_PX}px"
            except Exception:
                pass
        elif prev is None:
            accept = True

        conf_smooth = conf_raw
        if not accept:
            if not hasattr(self, '_last_reject_reason_log') or time.time() - self._last_reject_reason_log > 3.0:
                if rejection_reason:
                    print(f"Calibration rejected: {rejection_reason}")
                self._last_reject_reason_log = time.time()
            return prev

        if conf_smooth < 0.75:
            accept = False

        if not accept:
            if prev is not None:
                prev.confidence = float(conf_smooth)
            return prev

        if prev is None or not hasattr(self, '_calibrated_once'):
            print(f"✓ Calibration LOCKED: {len(cs)}/4 corners, reproj={reproj:.2f}px, cond={cond_H:.1f}")
            print(f"  Homography maps to nominal board: {KNOWN_BOARD_WIDTH_MM}mm × {KNOWN_BOARD_HEIGHT_MM}mm")
            self._calibrated_once = True
            self.calibration_locked = True

        self.board_pose = BoardPose(
            H_img2board=H,
            board_size_px=(int(round(width_px)), int(round(height_px))),
            mm_per_px=None,
            reproj_err_px=float(reproj),
            confidence=float(conf_smooth),
            corners_board_px={
                "TL": (0.0, KNOWN_BOARD_HEIGHT_MM),
                "TR": (KNOWN_BOARD_WIDTH_MM, KNOWN_BOARD_HEIGHT_MM),
                "BR": (KNOWN_BOARD_WIDTH_MM, 0.0),
                "BL": (0.0, 0.0),
            },
        )
        self.pose_fresh = True
        return self.board_pose


class BotTracker:
    """Detects robot markers and outputs center/heading in the board coordinate frame."""
    def __init__(self, cal: BoardCalibrator):
        self.det = _ArucoCompatDetector()
        self.cal = cal
        self._last_valid_pose: Optional[Tuple[BotPose, float]] = None
        self._pose_timeout_s = 5.0

        self._heading_hist: List[Tuple[float, float]] = []
        self._heading_hist_max = _HEADING_HISTORY_SIZE

    @staticmethod
    def _center(c4):
        return c4.mean(axis=0)

    @staticmethod
    def _warp(H, pts_xy):
        return cv2.perspectiveTransform(pts_xy[None, :, :].astype(np.float32), H)[0]

    def _add_heading_sample(self, v_board: np.ndarray) -> None:
        vx, vy = float(v_board[0]), float(v_board[1])
        norm = math.hypot(vx, vy)
        if norm < 1e-6:
            return
        ux, uy = vx / norm, vy / norm
        self._heading_hist.append((ux, uy))
        if len(self._heading_hist) > self._heading_hist_max:
            self._heading_hist.pop(0)

    def _robust_heading(self) -> Optional[float]:
        if not self._heading_hist:
            return None

        u = np.asarray(self._heading_hist, dtype=np.float32)
        N = u.shape[0]
        if N < _HEADING_RANSAC_MIN_SAMPLES:
            ux, uy = u[-1]
            return float(math.atan2(uy, ux))

        cos_thresh = math.cos(math.radians(_HEADING_RANSAC_MAX_DEG))

        best_inliers_mask = None
        best_count = 0

        for i in range(N):
            ui = u[i]
            dots = u @ ui
            inliers_mask = dots >= cos_thresh
            count = int(inliers_mask.sum())
            if count > best_count:
                best_count = count
                best_inliers_mask = inliers_mask

        if best_inliers_mask is None or best_count <= max(2, N // 3):
            ux, uy = u[-1]
            return float(math.atan2(uy, ux))

        inliers = u[best_inliers_mask]
        avg = inliers.mean(axis=0)
        norm = float(np.linalg.norm(avg))
        if norm < 1e-6:
            ux, uy = u[-1]
            return float(math.atan2(uy, ux))

        ux, uy = avg[0] / norm, avg[1] / norm
        return float(math.atan2(uy, ux))

    def track(self, frame_bgr) -> Optional[BotPose]:
        if self.cal.board_pose is None:
            return None

        frame_bgr = self.cal._undistort(frame_bgr)
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)

        det_out = self.det.detect(gray)
        if isinstance(det_out, tuple) and len(det_out) >= 3:
            id_map = det_out[2]
        else:
            id_map = det_out

        if id_map is None:
            id_map = {}

        has_marker_0 = BOT_MARKERS[0] in id_map
        has_marker_1 = BOT_MARKERS[1] in id_map

        if not has_marker_0 and not has_marker_1:
            if self._last_valid_pose is not None:
                cached_pose, cached_time = self._last_valid_pose
                age = time.time() - cached_time
                if age < self._pose_timeout_s:
                    if not hasattr(self, '_last_cache_use_log') or time.time() - self._last_cache_use_log > 1.0:
                        print(f"BOT_TRACK: Using cached pose (age={age:.1f}s) - markers temporarily lost")
                        self._last_cache_use_log = time.time()
                    return cached_pose
            return None

        if not (has_marker_0 and has_marker_1):
            missing = BOT_MARKERS[1] if has_marker_0 else BOT_MARKERS[0]
            detected = BOT_MARKERS[0] if has_marker_0 else BOT_MARKERS[1]
            if not hasattr(self, '_last_single_marker_warn') or time.time() - self._last_single_marker_warn > 3.0:
                print(f"BOT_TRACK WARNING: Only detected marker {detected}, missing {missing} - using cached pose")
                self._last_single_marker_warn = time.time()

            if self._last_valid_pose is not None:
                cached_pose, cached_time = self._last_valid_pose
                age = time.time() - cached_time
                if age < self._pose_timeout_s:
                    return cached_pose

            return None

        H = self.cal.board_pose.H_img2board
        c0 = self._center(id_map[BOT_MARKERS[0]])
        c1 = self._center(id_map[BOT_MARKERS[1]])
        pair_board = self._warp(H, np.vstack([c0, c1]))

        v = pair_board[1] - pair_board[0]
        center = 0.5 * (pair_board[0] + pair_board[1])
        baseline = float(np.linalg.norm(v))

        self._add_heading_sample(v)
        heading_robust = self._robust_heading()
        if heading_robust is None:
            heading = float(np.arctan2(v[1], v[0]))
        else:
            heading = heading_robust

        baseline_ratio = baseline / BOT_BASELINE_MM
        conf = float(np.clip(baseline_ratio, 0.8, 1.0))

        pose = BotPose(
            center_board_mm=(float(center[0]), float(center[1])),
            heading_rad=heading,
            baseline_mm=baseline,
            confidence=conf,
        )

        self._last_valid_pose = (pose, time.time())
        return pose


class PathCorrectionEngine:
    """Centralized CV-based path correction system."""
    def __init__(self, cvp: 'CVPipeline'):
        self.cvp = cvp
        self.last_correction_time = 0.0
        self.correction_count = 0
        self.last_pose: Optional[Tuple[Tuple[float, float], float]] = None
        self.velocity_estimate = (0.0, 0.0)
        self.min_correction_interval = 1.0 / CV_CORRECTION_RATE_HZ

    def reset(self):
        self.last_correction_time = 0.0
        self.correction_count = 0
        self.last_pose = None
        self.velocity_estimate = (0.0, 0.0)

    def compute_correction(
        self,
        waypoints: List[object],
        current_index: int,
        next_is_g1: bool = True,
    ) -> Optional[Tuple[int, int]]:
        if not next_is_g1:
            return None

        if current_index >= len(waypoints):
            return None

        now = time.time()
        if now - self.last_correction_time < self.min_correction_interval:
            return None

        pose = self.cvp.get_pose_mm()
        if not pose:
            return None

        (cx, cy), heading, confidence = pose

        if confidence < 0.6:
            return None

        def _wp_xy(wp: object) -> Tuple[float, float]:
            if hasattr(wp, "x_mm") and hasattr(wp, "y_mm"):
                return float(getattr(wp, "x_mm")), float(getattr(wp, "y_mm"))
            if isinstance(wp, (tuple, list)) and len(wp) >= 2:
                return float(wp[0]), float(wp[1])
            raise ValueError("Waypoint must have x_mm/y_mm or be (x, y)")

        target_wp = waypoints[current_index]
        tx, ty = _wp_xy(target_wp)

        ex = tx - cx
        ey = ty - cy
        error_dist = math.hypot(ex, ey)

        if error_dist < CV_MIN_ERROR_MM:
            return None

        lookahead_target_x = tx
        lookahead_target_y = ty
        if CV_LOOKAHEAD_COUNT > 0 and error_dist < 15.0:
            w_sum = 1.0
            for i in range(1, min(CV_LOOKAHEAD_COUNT + 1, len(waypoints) - current_index)):
                nx, ny = _wp_xy(waypoints[current_index + i])
                w = 1.0 / (i + 1)
                lookahead_target_x += nx * w
                lookahead_target_y += ny * w
                w_sum += w
            lookahead_target_x /= w_sum
            lookahead_target_y /= w_sum

        ex = lookahead_target_x - cx
        ey = lookahead_target_y - cy
        error_dist = math.hypot(ex, ey)

        path_tangent = None
        if current_index + 1 < len(waypoints):
            nx, ny = _wp_xy(waypoints[current_index + 1])
            vx = nx - tx
            vy = ny - ty
            vlen = math.hypot(vx, vy)
            if vlen > 1e-2:
                path_tangent = (vx / vlen, vy / vlen)

        if path_tangent is not None:
            txu, tyu = path_tangent
            along = ex * txu + ey * tyu
            cross_x = ex - along * txu
            cross_y = ey - along * tyu
            weighted_ex = cross_x * CV_CROSS_TRACK_GAIN + along * txu
            weighted_ey = cross_y * CV_CROSS_TRACK_GAIN + along * tyu
        else:
            weighted_ex = ex
            weighted_ey = ey

        if CV_ADAPTIVE_STEP:
            base = min(12.0, error_dist)
            scale = min(1.0, error_dist / 15.0)
            scale = max(0.4, scale)
        else:
            base = min(12.0, error_dist)
            scale = 1.0

        conf_factor = max(0.4, min(1.0, confidence))

        step_norm = math.hypot(weighted_ex, weighted_ey)
        if step_norm < 1e-3:
            return None

        step_magnitude = base * scale * conf_factor
        dx_board = weighted_ex / step_norm * step_magnitude
        dy_board = weighted_ey / step_norm * step_magnitude

        bounds = self.cvp.get_board_bounds_mm()
        if bounds:
            min_x, min_y, max_x, max_y = bounds
            nx = max(min_x, min(cx + dx_board, max_x))
            ny = max(min_y, min(cy + dy_board, max_y))
            dx_board = nx - cx
            dy_board = ny - cy

        cos_h = math.cos(heading)
        sin_h = math.sin(heading)
        fwd = dx_board * cos_h + dy_board * sin_h
        left = -dx_board * sin_h + dy_board * cos_h

        if abs(fwd) < 0.5 and abs(left) < 0.5:
            return None

        sx = int(round(fwd))
        sy = int(round(left))
        if sx == 0 and sy == 0:
            return None

        self.last_correction_time = now
        self.correction_count += 1

        if self.correction_count % 10 == 1 or error_dist > 5.0:
            print(
                f"CV_CORRECTION[{self.correction_count}]: "
                f"wp={current_index}/{len(waypoints)} "
                f"err={error_dist:.1f}mm "
                f"pos=({cx:.1f},{cy:.1f}) → target=({lookahead_target_x:.1f},{lookahead_target_y:.1f}) "
                f"robot=({sx},{sy})mm heading={math.degrees(heading):.1f}° conf={confidence:.2f}"
            )

        return sx, sy


class CVPipeline:
    """Top-level CV pipeline."""
    def __init__(self, cam: Optional[CameraModel] = None):
        self.cal = BoardCalibrator(cam)
        self.trk = BotTracker(self.cal)
        self._bot: Optional[BotPose] = None
        self._pose_smooth: Optional[BotPose] = None
        self._path_deviation_history: List[float] = []
        self.correction = PathCorrectionEngine(self)

    @staticmethod
    def _ema(old, new, a):
        if old is None:
            return new
        return a * old + (1.0 - a) * new

    def _ema_angle(self, prev: float, new: float, a: float) -> float:
        d = math.atan2(math.sin(new - prev), math.cos(new - prev))
        h = prev + (1.0 - a) * d
        return math.atan2(math.sin(h), math.cos(h))

    def calibrate_board(self, frame_bgr) -> bool:
        return self.cal.calibrate(frame_bgr) is not None

    def update_bot(self, frame_bgr) -> bool:
        bot = self.trk.track(frame_bgr)
        if bot is None:
            return False
        self._bot = bot

        if self._pose_smooth is None:
            self._pose_smooth = bot
        else:
            cx = self._ema(self._pose_smooth.center_board_mm[0], bot.center_board_mm[0], _POSE_ALPHA)
            cy = self._ema(self._pose_smooth.center_board_mm[1], bot.center_board_mm[1], _POSE_ALPHA)
            hd = self._ema_angle(self._pose_smooth.heading_rad, bot.heading_rad, _POSE_ALPHA)

            self._pose_smooth = BotPose(
                center_board_mm=(cx, cy),
                heading_rad=hd,
                baseline_mm=bot.baseline_mm,
                confidence=max(self._pose_smooth.confidence, bot.confidence),
            )
        return True

    def process_frame(self, frame_bgr) -> tuple[bool, float]:
        t0 = time.monotonic()
        st = self.get_state()
        needs_cal = not st["calibrated"]
        if needs_cal or (st["board_confidence"] is not None and st["board_confidence"] < _CONF_MIN_BOARD):
            self.calibrate_board(frame_bgr)

        _ = self.update_bot(frame_bgr)
        st = self.get_state()

        good_cal = (
            st["calibrated"]
            and st["board_reproj_err_px"] is not None
            and st["board_reproj_err_px"] <= _MAX_REPROJ_ERR_PX
            and (st["board_confidence"] is None or st["board_confidence"] >= _CONF_MIN_BOARD)
        )

        good_bot = st["bot_pose"] is not None and st["bot_pose"]["confidence"] >= _CONF_MIN_BOT
        valid = bool(good_cal and good_bot)

        return valid, time.monotonic() - t0

    def get_state(self):
        bp = self.cal.board_pose
        bot = self._pose_smooth or self._bot
        return {
            "calibrated": bp is not None,
            "board_size_px": None if bp is None else bp.board_size_px,
            "board_reproj_err_px": None if bp is None else bp.reproj_err_px,
            "board_confidence": None if bp is None else bp.confidence,
            "board_pose_fresh": getattr(self.cal, "pose_fresh", False),
            "bot_pose": None if bot is None else {
                "center_board_mm": bot.center_board_mm,
                "heading_rad": bot.heading_rad,
                "baseline_mm": bot.baseline_mm,
                "confidence": bot.confidence,
            },
        }

    def _baseline_scale(self) -> Optional[float]:
        """Kept for compatibility, no longer used for board scaling."""
        bot = self._pose_smooth or self._bot
        if bot is None:
            return None
        if bot.baseline_mm <= 1e-6:
            return None
        return BOT_BASELINE_MM / bot.baseline_mm

    def board_size_mm(self) -> Optional[Tuple[float, float]]:
        """
        Board physical size in mm.

        We now treat the homography frame (KNOWN_BOARD_WIDTH_MM × KNOWN_BOARD_HEIGHT_MM)
        as the actual physical board dimensions.
        """
        if self.cal.board_pose is None:
            return None
        return (KNOWN_BOARD_WIDTH_MM, KNOWN_BOARD_HEIGHT_MM)

    def get_board_bounds_mm(self) -> Optional[Tuple[float, float, float, float]]:
        """Get SAFE board bounds in mm with margins."""
        bs = self.board_size_mm()
        if not bs:
            return None

        Wmm, Hmm = bs
        margin = max(BOARD_MARGIN_MM, BOARD_MARGIN_FRAC * min(Wmm, Hmm))
        print(f"Board bounds with margin {margin:.1f}mm: W={Wmm:.1f}mm, H={Hmm:.1f}mm")
        safe_min_x = margin
        safe_min_y = margin
        safe_max_x = Wmm - margin
        safe_max_y = Hmm - margin
        return (safe_min_x, safe_min_y, safe_max_x, safe_max_y)

    def fit_path_to_board(self, wps, margin_frac: float = 0.05):
        sz = self.board_size_mm()
        if not sz:
            return []

        if self.cal.board_pose is None or self.cal.board_pose.confidence < 0.75:
            print(
                "PATH FITTING REJECTED: Board not sufficiently calibrated "
                f"(confidence={getattr(self.cal.board_pose, 'confidence', 0.0):.2f}, need ≥0.75)"
            )
            return []

        return _fit_to_board(wps, sz, margin_frac)

    def corrected_delta_for_bt(self, desired_delta_board_mm, rotate_into_bot_frame=False):
        _warn_deprecated("CVPipeline.corrected_delta_for_bt")
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

    def get_pose_mm(self, use_raw=False):
        """
        Smoothed robot pose in physical mm and heading/confidence, or None if not ready.

        Since the board homography is defined directly in mm, we now return
        center_board_mm as the physical pose without extra scaling.
        """
        if self._bot is None:
            return None

        bot = self._bot if use_raw else self._pose_smooth
        if bot is None:
            return None

        cx, cy = bot.center_board_mm
        return (cx, cy), float(bot.heading_rad), float(bot.confidence)

    def get_path_correction(
        self,
        target_board_mm: Tuple[float, float],
        max_correction_mm: float = 5.0,
        min_correction_mm: float = 1.0,
        apply_bounds: bool = True,
    ) -> Optional[Tuple[float, float]]:
        _warn_deprecated("CVPipeline.get_path_correction")
        pose = self.get_pose_mm()
        if not pose:
            return None

        (current_x, current_y), heading, conf = pose

        target_x, target_y = target_board_mm
        error_x = target_x - current_x
        error_y = target_y - current_y
        error_mag = math.sqrt(error_x**2 + error_y**2)

        self._path_deviation_history.append(error_mag)
        if len(self._path_deviation_history) > 100:
            self._path_deviation_history.pop(0)

        if error_mag < min_correction_mm:
            return (0.0, 0.0)

        if error_mag > max_correction_mm:
            scale = max_correction_mm / error_mag
            error_x *= scale
            error_y *= scale

        if apply_bounds:
            bounds = self.get_board_bounds_mm()
            if bounds:
                min_x, min_y, max_x, max_y = bounds
                width_mm = max_x - min_x
                height_mm = max_y - min_y
                margin = max(BOARD_MARGIN_MM, BOARD_MARGIN_FRAC * min(width_mm, height_mm))

                next_x = current_x + error_x
                next_y = current_y + error_y
                next_x = max(min_x + margin, min(next_x, max_x - margin))
                next_y = max(min_y + margin, min(next_y, max_y - margin))
                error_x = next_x - current_x
                error_y = next_y - current_y

        gain = 0.3 * (conf ** 1.5)

        return (error_x * gain, error_y * gain)

    def get_path_stats(self) -> Dict:
        if not self._path_deviation_history:
            return {}

        arr = np.asarray(self._path_deviation_history, dtype=float)
        rms = float(np.sqrt(np.mean(arr**2))) if arr.size else 0.0
        return {
            "mean_deviation_mm": float(np.mean(arr)) if arr.size else 0.0,
            "max_deviation_mm": float(np.max(arr)) if arr.size else 0.0,
            "std_deviation_mm": float(np.std(arr)) if arr.size else 0.0,
            "rms_deviation_mm": rms,
        }

    def build_firmware_gcode(self, gcode_text: str, canvas_size: Tuple[float, float] = (575.0, 730.0)) -> str:
        """
        Convert pathfinding G-code (canvas units) into firmware-ready, relative G-code.
        """
        bs = self.board_size_mm()
        if not bs:
            raise ValueError("Board not calibrated or no scale available")

        bounds = self.get_board_bounds_mm()
        if bounds is None:
            Wmm, Hmm = bs
            margin = max(BOARD_MARGIN_MM, BOARD_MARGIN_FRAC * min(Wmm, Hmm))
            min_x, min_y = margin, margin
            max_x, max_y = Wmm - margin, Hmm - margin
            print("BOUNDS: Using computed board size (bounds unavailable)")
        else:
            min_x, min_y, max_x, max_y = bounds
            print(f"BOUNDS: Using scaled marker-based bounds ({min_x:.1f},{min_y:.1f}) to ({max_x:.1f},{max_y:.1f})")

        scaled = scale_gcode_to_board(gcode_text, canvas_size, bs)
        print("DEBUG build_firmware_gcode: After scaling, first 5 lines:")
        for i, line in enumerate(scaled.split('\n')[:5]):
            print(f"  {i}: {line}")

        normalized = convert_pathfinding_gcode(scaled)

        lines = normalized.split('\n')
        bounded_lines = []
        pos_x, pos_y = 0.0, 0.0

        for line in lines:
            stripped = line.strip()
            if not stripped or stripped.startswith(';') or stripped.startswith('M'):
                bounded_lines.append(line)
                continue

            if stripped.startswith('G1') or stripped.startswith('G0'):
                parts = stripped.split()
                dx = dy = 0
                for p in parts[1:]:
                    if p.startswith('X'):
                        try:
                            dx = int(p[1:])
                        except Exception:
                            pass
                    elif p.startswith('Y'):
                        try:
                            dy = int(p[1:])
                        except Exception:
                            pass

                next_x = pos_x + dx
                next_y = pos_y + dy

                clamped_x = max(min_x, min(next_x, max_x))
                clamped_y = max(min_y, min(next_y, max_y))

                actual_dx = int(round(clamped_x - pos_x))
                actual_dy = int(round(clamped_y - pos_y))

                if actual_dx != dx or actual_dy != dy:
                    print(
                        f"BOUNDS: Clamped move from ({dx},{dy}) to ({actual_dx},{actual_dy}) "
                        f"at pos ({pos_x:.1f},{pos_y:.1f})"
                    )

                pos_x = clamped_x
                pos_y = clamped_y

                if actual_dx == 0 and actual_dy == 0:
                    continue

                cmd_parts = [parts[0]]
                if actual_dx != 0:
                    cmd_parts.append(f"X{actual_dx}")
                if actual_dy != 0:
                    cmd_parts.append(f"Y{actual_dy}")
                bounded_lines.append(' '.join(cmd_parts))
            else:
                bounded_lines.append(line)

        bounded = '\n'.join(bounded_lines)

        print("DEBUG build_firmware_gcode: After bounds enforcement, first 5 lines:")
        for i, line in enumerate(bounded.split('\n')[:5]):
            print(f"  {i}: {line}")

        return bounded
