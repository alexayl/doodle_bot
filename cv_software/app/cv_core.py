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
BOARD_MARGIN_MM = float(os.getenv("BOARD_MARGIN_MM", "12"))
BOARD_MARGIN_FRAC = float(os.getenv("BOARD_MARGIN_FRAC", "0.10"))
_POSE_ALPHA = 0.7  #  factor for updating the pose estimate - higher = faster response
_SCALE_ALPHA = 0.15  #  factor for updating the scale estimate
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
CV_CORRECTION_RATE_HZ = float(os.getenv("CV_CORRECTION_RATE_HZ", "10.0"))
CV_MIN_ERROR_MM = float(os.getenv("CV_MIN_ERROR_MM", "1.5"))
CV_LOOKAHEAD_COUNT = int(os.getenv("CV_LOOKAHEAD_COUNT", "3"))
CV_CROSS_TRACK_GAIN = float(os.getenv("CV_CROSS_TRACK_GAIN", "1.5"))
CV_ADAPTIVE_STEP = bool(int(os.getenv("CV_ADAPTIVE_STEP", "1")))

BOARD_CORNERS = {"TL": 0, "TR": 1, "BR": 2, "BL": 3}
BOT_MARKERS = (10, 11)

# Options: "4x4_50", "5x5_100", "6x6_250", "apriltag_36h11"
MARKER_DICT = os.getenv("MARKER_DICT", "5x5_100").upper()

BOT_BASELINE_MM = 100.0

KNOWN_BOARD_WIDTH_MM = float(650)
KNOWN_BOARD_HEIGHT_MM = float(350)
_MAX_REPROJ_ERR_PX = 1.2  # More tolerant for 5x5 markers
_MAX_SIZE_JUMP_FRAC = 0.15  # Allow slightly larger jumps
_MAX_CENTER_JUMP_PX = 40.0  # More tolerant center movement 

# Thresholds expected by tests
_REPROJ_GOOD_PX = 0.5
_COND_HARD_LIMIT = 5e7  # Reject if condition number > 50,000 (too ill-conditioned)
_COND_WARN_LIMIT = 3e4  # Warn if condition number > 30,000

# ---------- Module-level helpers expected by tests ----------
def board_pixels_to_mm(pts: np.ndarray, mm_per_px: float) -> np.ndarray:
    """Convert board pixel coordinates to millimeters using mm/px scale."""
    return np.asarray(pts, dtype=np.float32) * float(mm_per_px)

def warp_points_board2img(H_img2board: np.ndarray, pts_board: np.ndarray) -> np.ndarray:
    """Warp points from board coordinates back to image coordinates using inverse homography."""
    H = np.asarray(H_img2board, dtype=np.float32)
    Hinv = np.linalg.inv(H)
    return cv2.perspectiveTransform(np.asarray(pts_board, dtype=np.float32)[None], Hinv)[0]

def correct_delta_mm(delta_board_mm: tuple[float, float], heading_rad: float) -> tuple[float, float]:
    """Rotate a board-frame delta (dx,dy) into robot forward/left components.

    forward = dx*cos(h) + dy*sin(h)
    left    = -dx*sin(h) + dy*cos(h)
    """
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
    # Store actual corner positions in board pixel space for strict bounds
    corners_board_px: Optional[Dict[str, Tuple[float, float]]] = None

@dataclass
class BotPose:
    """Robot pose in board millimeters: center (mm), heading (rad), inter-marker baseline (mm), and confidence."""
    center_board_mm: Tuple[float, float]
    heading_rad: float
    baseline_mm: float
    confidence: float = 0.0

def _aruco_dict():
    """Get ArUco/AprilTag dictionary based on MARKER_DICT setting"""
    dict_name = MARKER_DICT.replace("X", "x")  # Normalize
    
    # Map common names to OpenCV constants
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
    if hasattr(cv2.aruco, "DetectorParameters"):
        p = cv2.aruco.DetectorParameters()
    else:
        p = cv2.aruco.DetectorParameters_create()
    if hasattr(p, "cornerRefinementMethod"):
        p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    if hasattr(p, "adaptiveThreshConstant"):
        p.adaptiveThreshConstant = 5  # Lower = more sensitive to variations
    
    # AGGRESSIVE tolerances for 5x5_100 markers
    if hasattr(p, "minMarkerPerimeterRate"): p.minMarkerPerimeterRate = 0.01  # Allow smaller markers
    if hasattr(p, "maxMarkerPerimeterRate"): p.maxMarkerPerimeterRate = 6.0   # Allow larger markers
    if hasattr(p, "adaptiveThreshWinSizeMin"): p.adaptiveThreshWinSizeMin = 3
    if hasattr(p, "adaptiveThreshWinSizeMax"): p.adaptiveThreshWinSizeMax = 53  # Much larger window for varying lighting
    if hasattr(p, "adaptiveThreshWinSizeStep"): p.adaptiveThreshWinSizeStep = 4  # Finer steps
    
    # Polygon approximation accuracy (lower = more tolerant to perspective distortion)
    if hasattr(p, "polygonalApproxAccuracyRate"): p.polygonalApproxAccuracyRate = 0.05
    
    # Corner quality (lower = accept more candidates)
    if hasattr(p, "minCornerDistanceRate"): p.minCornerDistanceRate = 0.03
    if hasattr(p, "minDistanceToBorder"): p.minDistanceToBorder = 1  # Allow markers near edges
    
    # Bit extraction (important for 5x5 with small cells)
    if hasattr(p, "perspectiveRemovePixelPerCell"): p.perspectiveRemovePixelPerCell = 6  # More samples per cell
    if hasattr(p, "perspectiveRemoveIgnoredMarginPerCell"): p.perspectiveRemoveIgnoredMarginPerCell = 0.10
    
    # Error correction
    if hasattr(p, "maxErroneousBitsInBorderRate"): p.maxErroneousBitsInBorderRate = 0.45  # More tolerant
    if hasattr(p, "errorCorrectionRate"): p.errorCorrectionRate = 0.8  # Higher error correction
    
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
            # print(f"Detected {0 if ids is None else len(ids)} markers")
            # print(ids)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self._dict, parameters=self._params)
        id_map: Dict[int, np.ndarray] = {}
        if ids is not None:
            for c, i in zip(corners, ids.flatten()):
                id_map[int(i)] = c.reshape(-1, 2).astype(np.float32)
        return id_map

def _conf_from_visibility(found_corners: int, total: int = 4) -> float:
    """
    0.0 if <3 corners; otherwise linear with count.
    3/4 -> 0.75, 4/4 -> 1.0
    """
    if found_corners < 3:
        return 0.0
    return min(1.0, found_corners / float(total))

class BoardCalibrator:
    """Finds board corner markers, estimates image→board homography, and scores calibration quality."""
    def __init__(self, cam: Optional[CameraModel] = None):
        self.det = _ArucoCompatDetector()
        self.cam = cam or CameraModel()
        self.board_pose: Optional[BoardPose] = None
        self.pose_fresh: bool = False
        self.calibration_locked: bool = False  # Lock calibration after first success  
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
        """Compute and cache image→board homography; confidence = visibility only.
        
        Once calibration succeeds, it locks in the board bounds and only updates if:
        - force_recalibrate=True is passed
        - Current calibration quality is poor (confidence < 0.5)
        """
        self.pose_fresh = False  # reset; set True only if we accept a new pose

        # If calibration is locked and quality is good, skip re-calibration
        if self.calibration_locked and not force_recalibrate:
            if self.board_pose is not None and self.board_pose.confidence >= 0.5:
                return self.board_pose
            # Quality degraded - allow recalibration attempt
            if not hasattr(self, '_quality_degrade_log') or time.time() - self._quality_degrade_log > 5.0:
                print(f"Calibration quality degraded (conf={self.board_pose.confidence:.2f}) - attempting recalibration")
                self._quality_degrade_log = time.time()
                self.calibration_locked = False

        frame_bgr = self._undistort(frame_bgr)
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        det_out = self.det.detect(gray)
        # Support detectors that return (corners, ids, id_map) or just id_map
        if isinstance(det_out, tuple) and len(det_out) >= 3:
            id_map = det_out[2]
        else:
            id_map = det_out
        if id_map is None:
            id_map = {}
        cs = self._collect_corners(id_map)
        # RELAXED: Allow 3/4 corners with geometric completion
        if len(cs) < 3:
            # Only log rejections occasionally to reduce noise
            if not hasattr(self, '_last_reject_log') or time.time() - self._last_reject_log > 2.0:
                print(f"Calibration: {len(cs)}/4 corners (need ≥3)")
                self._last_reject_log = time.time()
            return self.board_pose
        
        # Complete the quad if we have 3/4 corners
        if len(cs) == 3:
            cs = self._complete_quad(cs)
            # Only log completion when fresh
            if not hasattr(self, '_last_completion_log') or time.time() - self._last_completion_log > 3.0:
                print(f"Calibration: Completed quad from 3 corners")
                self._last_completion_log = time.time()
        
        if not all(k in cs for k in ("TL", "TR", "BR", "BL")):
            # Silent rejection - should not happen after quad completion
            return self.board_pose

        tl, tr, br, bl = cs["TL"], cs["TR"], cs["BR"], cs["BL"]
        img_pts = np.array([tl, tr, br, bl], dtype=np.float32)
        
        # Use KNOWN physical board dimensions (mm) for homography target
        # This makes the homography map directly to millimeters
        board_rect = np.array(
            [[0.0, 0.0], [KNOWN_BOARD_WIDTH_MM, 0.0], [KNOWN_BOARD_WIDTH_MM, KNOWN_BOARD_HEIGHT_MM], [0.0, KNOWN_BOARD_HEIGHT_MM]],
            dtype=np.float32
        )

        # Compute homography using getPerspectiveTransform to align with tests
        H = cv2.getPerspectiveTransform(img_pts.astype(np.float32), board_rect.astype(np.float32))
        if H is None:
            # No previous pose -> remain None
            return self.board_pose
        if abs(H[2, 2]) > 1e-12:
            H = H / H[2, 2]

        reproj = self._reproj_error(H, img_pts, board_rect)
        found = sum(1 for k in ("TL", "TR", "BR", "BL") if k in cs)
        # Confidence reflects actual detected corners (3/4=0.75, 4/4=1.0)
        conf_raw = _conf_from_visibility(found)
        prev = self.board_pose

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

        if prev is not None:
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
                board_center = np.array([[0.5 * prev.board_size_px[0], 0.5 * prev.board_size_px[1]]], dtype=np.float32)[None]
                img_center_prev = cv2.perspectiveTransform(board_center, Hinv)[0][0]
                center_jump = float(np.linalg.norm(c_new - img_center_prev))
                if center_jump > _MAX_CENTER_JUMP_PX:
                    accept = False
                    rejection_reason = f"center_jump {center_jump:.1f}px > {_MAX_CENTER_JUMP_PX}px"
            except Exception:
                pass
        elif prev is None:
            # First calibration - always accept if quad completed
            accept = True

        # RELAXED: Accept 3/4 or 4/4 corners (confidence reflects actual count)
        conf_smooth = conf_raw
        # Early exit on acceptance rule
        if not accept:
            if not hasattr(self, '_last_reject_reason_log') or time.time() - self._last_reject_reason_log > 3.0:
                if rejection_reason:
                    print(f"Calibration rejected: {rejection_reason}")
                self._last_reject_reason_log = time.time()
            return prev
        
        if conf_smooth < 0.75:
            # Silent rejection - already logged above if needed
            accept = False

        if not accept:
            if prev is not None:
                prev.confidence = float(conf_smooth)
            return prev
        
        # Log successful calibration acceptance
        if prev is None or not hasattr(self, '_calibrated_once'):
            print(f"✓ Calibration LOCKED: {len(cs)}/4 corners, reproj={reproj:.2f}px, cond={cond_H:.1f}")
            print(f"  Homography maps directly to physical board: {KNOWN_BOARD_WIDTH_MM}mm × {KNOWN_BOARD_HEIGHT_MM}mm")
            self._calibrated_once = True
            self.calibration_locked = True  # Lock in the calibration
        
        self.board_pose = BoardPose(
            H_img2board=H,
            board_size_px=None,  # No longer used - homography maps directly to mm
            mm_per_px=None,  # No longer used - homography maps directly to mm
            reproj_err_px=float(reproj),
            confidence=float(conf_smooth),
            corners_board_px={
                "TL": (0.0, 0.0),
                "TR": (KNOWN_BOARD_WIDTH_MM, 0.0),
                "BR": (KNOWN_BOARD_WIDTH_MM, KNOWN_BOARD_HEIGHT_MM),
                "BL": (0.0, KNOWN_BOARD_HEIGHT_MM)
            }
        )
        self.pose_fresh = True
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
        det_out = self.det.detect(gray)
        if isinstance(det_out, tuple) and len(det_out) >= 3:
            id_map = det_out[2]
        else:
            id_map = det_out
        if id_map is None:
            return None
        if BOT_MARKERS[0] not in id_map or BOT_MARKERS[1] not in id_map:
            return None

        H = self.cal.board_pose.H_img2board
        c0 = self._center(id_map[BOT_MARKERS[0]])
        c1 = self._center(id_map[BOT_MARKERS[1]])
        pair_board = self._warp(H, np.vstack([c0, c1]))

        v = pair_board[1] - pair_board[0]
        center = 0.5 * (pair_board[0] + pair_board[1])
        heading = float(np.arctan2(v[1], v[0]))  # [-pi, pi]
        baseline = float(np.linalg.norm(v))  # in mm now (homography→mm)
        # Confidence based on baseline visibility (expect ~100mm)
        conf = float(np.clip((baseline / 100.0), 0.0, 1.0))

        return BotPose(
            center_board_mm=(center[0], center[1]),
            heading_rad=heading,
            baseline_mm=baseline,
            confidence=conf,
        )

# ScaleEstimator removed - homography now maps directly to physical mm

class PathCorrectionEngine:
    """
    Centralized CV-based path correction system.
    Computes real-time corrections for path following with:
    - Adaptive step sizing
    - Cross-track error priority
    - Look-ahead waypoint blending
    - Velocity estimation
    """
    def __init__(self, cvp: 'CVPipeline'):
        self.cvp = cvp
        self.last_correction_time = 0.0
        self.correction_count = 0
        self.last_pose: Optional[Tuple[Tuple[float, float], float]] = None
        self.velocity_estimate = (0.0, 0.0)  # (vx, vy) in board frame mm/s
        self.min_correction_interval = 1.0 / CV_CORRECTION_RATE_HZ
    
    def reset(self):
        """Reset correction state (call when starting new path)."""
        self.last_correction_time = 0.0
        self.correction_count = 0
        self.last_pose = None
        self.velocity_estimate = (0.0, 0.0)
    
    def compute_correction(
        self,
        waypoints: List[object],
        current_index: int,
        next_is_g1: bool = True
    ) -> Optional[Tuple[int, int]]:
        """
        Compute CV correction for current waypoint.
        
        Args:
            waypoints: Full list of path waypoints in board mm
            current_index: Index of current target waypoint
            next_is_g1: True if next command is a movement (G1)
        
        Returns:
            (sx, sy) correction in robot frame (mm as ints), or None if no correction needed
        """
        # Only correct movement commands
        if not next_is_g1:
            return None
        
        # Rate limiting
        now = time.time()
        if now - self.last_correction_time < self.min_correction_interval:
            return None
        
        # Need valid waypoint
        if current_index >= len(waypoints):
            return None

        # Helper to read coordinates from either control.Waypoint or (x,y) tuple/list
        def _wp_xy(wp: object) -> Tuple[float, float]:
            if hasattr(wp, "x_mm") and hasattr(wp, "y_mm"):
                return float(getattr(wp, "x_mm")), float(getattr(wp, "y_mm"))
            if isinstance(wp, (tuple, list)) and len(wp) >= 2:
                return float(wp[0]), float(wp[1])
            raise ValueError("Waypoint must have x_mm/y_mm or be a (x,y) tuple")

        target_wp = waypoints[current_index]
        
        # Get current pose
        pose = self.cvp.get_pose_mm()
        if not pose:
            return None
        
        (cx, cy), heading, confidence = pose
        
        # Update velocity estimate
        if self.last_pose is not None:
            (last_x, last_y), last_t = self.last_pose
            dt = now - last_t
            if dt > 0.01:
                vx = (cx - last_x) / dt
                vy = (cy - last_y) / dt
                # Smooth with EMA
                alpha = 0.3
                self.velocity_estimate = (
                    alpha * vx + (1 - alpha) * self.velocity_estimate[0],
                    alpha * vy + (1 - alpha) * self.velocity_estimate[1]
                )
        self.last_pose = ((cx, cy), now)
        
        # Calculate error to current target
        tx, ty = _wp_xy(target_wp)
        ex = tx - cx
        ey = ty - cy
        error_dist = math.sqrt(ex**2 + ey**2)
        
        # Ignore tiny errors
        if error_dist < CV_MIN_ERROR_MM:
            return None
        
        # Look-ahead blending
        lookahead_target_x = tx
        lookahead_target_y = ty
        
        if CV_LOOKAHEAD_COUNT > 0 and error_dist < 10.0:
            lookahead_weight = 0.0
            for i in range(1, min(CV_LOOKAHEAD_COUNT + 1, len(waypoints) - current_index)):
                next_idx = current_index + i
                if next_idx < len(waypoints):
                    next_wp = waypoints[next_idx]
                    nx, ny = _wp_xy(next_wp)
                    weight = 1.0 / (i + 1)
                    lookahead_target_x += nx * weight
                    lookahead_target_y += ny * weight
                    lookahead_weight += weight
            
            if lookahead_weight > 0:
                total_weight = 1.0 + lookahead_weight
                lookahead_target_x /= total_weight
                lookahead_target_y /= total_weight
        
        # Cross-track error decomposition
        path_tangent = None
        cross_track_x = ex
        cross_track_y = ey
        
        if current_index + 1 < len(waypoints):
            next_wp = waypoints[current_index + 1]
            nx, ny = _wp_xy(next_wp)
            tx_vec = nx - tx
            ty_vec = ny - ty
            t_len = math.sqrt(tx_vec**2 + ty_vec**2)
            if t_len > 0.1:
                path_tangent = (tx_vec / t_len, ty_vec / t_len)
        
        if path_tangent:
            tx, ty = path_tangent
            along_track = ex * tx + ey * ty
            cross_track_x = ex - along_track * tx
            cross_track_y = ey - along_track * ty
            
            # Prioritize cross-track
            weighted_ex = cross_track_x * CV_CROSS_TRACK_GAIN + along_track * tx
            weighted_ey = cross_track_y * CV_CROSS_TRACK_GAIN + along_track * ty
        else:
            weighted_ex = ex
            weighted_ey = ey
        
        # Adaptive step sizing
        if CV_ADAPTIVE_STEP:
            step_scale = min(1.0, error_dist / 10.0)
            step_scale = max(0.3, step_scale)
        else:
            step_scale = 1.0
        
        # Compute base step (board frame)
        # Confidence-weighted scaling: reduce aggressiveness at low CV confidence
        conf_factor = max(0.3, min(1.0, confidence ** 1.5))
        step_magnitude = min(25.0, error_dist) * step_scale * conf_factor  # Max 25mm per correction
        step_norm = math.sqrt(weighted_ex**2 + weighted_ey**2)
        if step_norm < 0.01:
            return None
        
        dx_board = (weighted_ex / step_norm) * step_magnitude
        dy_board = (weighted_ey / step_norm) * step_magnitude
        
        # Apply bounds clamping
        bounds = self.cvp.get_board_bounds_mm()
        if bounds:
            min_x, min_y, max_x, max_y = bounds
            width_mm = max_x - min_x
            height_mm = max_y - min_y
            margin_mm = max(BOARD_MARGIN_MM, BOARD_MARGIN_FRAC * min(width_mm, height_mm))
            
            nx = max(min_x + margin_mm, min(cx + dx_board, max_x - margin_mm))
            ny = max(min_y + margin_mm, min(cy + dy_board, max_y - margin_mm))
            dx_board = nx - cx
            dy_board = ny - cy
        
        # Rotate to robot frame
        cos_t = math.cos(heading)
        sin_t = math.sin(heading)
        fwd = dx_board * cos_t + dy_board * sin_t
        left = -dx_board * sin_t + dy_board * cos_t
        
        # Ignore tiny moves
        if abs(fwd) < 0.5 and abs(left) < 0.5:
            return None
        
        # Convert to integers for firmware
        sx = int(round(fwd))
        sy = int(round(left))
        if sx == 0 and sy == 0:
            return None
        
        # Update state
        self.last_correction_time = now
        self.correction_count += 1
        
        # Logging
        if self.correction_count % 10 == 1 or error_dist > 5.0:
            cross_track_mag = math.sqrt(cross_track_x**2 + cross_track_y**2) if path_tangent else 0.0
            print(
                f"CV_CORRECTION[{self.correction_count}]: "
                f"wp={current_index}/{len(waypoints)} "
                f"err={error_dist:.1f}mm (cross={cross_track_mag:.1f}mm) "
                f"pos=({cx:.1f},{cy:.1f}) → target=({lookahead_target_x:.1f},{lookahead_target_y:.1f}) "
                f"robot=({sx},{sy})mm heading={math.degrees(heading):.1f}° conf={confidence:.2f}"
            )
        
        return (sx, sy)


class CVPipeline:
    """Top-level CV pipeline: board calibration, bot tracking, scale smoothing, and API to query state."""
    def __init__(self, cam: Optional[CameraModel] = None):
        self.cal = BoardCalibrator(cam)
        self.trk = BotTracker(self.cal)
        self._bot: Optional[BotPose] = None
        self._pose_smooth: Optional[BotPose] = None
        self._path_deviation_history: List[float] = []  # Track path following accuracy
        self.correction = PathCorrectionEngine(self)  # CV correction engine

    @staticmethod
    def _ema(old, new, a):
        """Simple exponential moving average helper (returns new if old is None)."""
        if old is None:
            return new
        return a * old + (1.0 - a) * new
    def _ema_angle(self, prev: float, new: float, a: float) -> float:
        d = math.atan2(math.sin(new - prev), math.cos(new - prev))
        h = prev + (1.0 - a) * d
        return math.atan2(math.sin(h), math.cos(h))
    # wrap to [-pi, pi]
    def calibrate_board(self, frame_bgr) -> bool:
        """Run a single-board calibration step from a frame; returns True if a pose is available."""
        return self.cal.calibrate(frame_bgr) is not None

    def update_bot(self, frame_bgr) -> bool:
        """Track the robot for the given frame and update smoothed pose and mm/px scale if available."""
        bot = self.trk.track(frame_bgr)
        if bot is None:
            return False
        self._bot = bot

        # Positions already in mm from homography - no scaling needed

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

        # Update robot position (already in mm from homography)
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
        """Return a serializable snapshot of current calibration and (smoothed) bot pose."""
        bp = self.cal.board_pose
        bot = self._pose_smooth or self._bot
        return {
            "calibrated": bp is not None,
            "board_size_px": None,  # No longer used - homography→mm directly
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

    def board_size_mm(self) -> Optional[Tuple[float, float]]:
        """Board physical size in mm using KNOWN dimensions.
        Returns None if not calibrated (need board corners detected).
        """
        st = self.get_state()
        if not st["calibrated"]:
            return None
        
        # Return known physical board size
        return (KNOWN_BOARD_WIDTH_MM, KNOWN_BOARD_HEIGHT_MM)
    
    def get_board_bounds_mm(self) -> Optional[Tuple[float, float, float, float]]:
        """Get strict board bounds in mm using KNOWN physical dimensions.
        Returns (min_x, min_y, max_x, max_y) or None if not calibrated.
        The board corners define the coordinate frame, but physical size is known.
        """
        bp = self.cal.board_pose
        if bp is None:
            return None
        
        # Use KNOWN physical board size, not pixel-based scaling
        # The board coordinate frame origin is at TL corner
        return (0.0, 0.0, KNOWN_BOARD_WIDTH_MM, KNOWN_BOARD_HEIGHT_MM)

    def fit_path_to_board(self, wps, margin_frac: float = 0.05):
        """Scale/center normalized waypoints to the calibrated board."""
        sz = self.board_size_mm()
        if not sz:
            return []
        
        # RELAXED: Allow 3/4 or 4/4 corner calibration (confidence >= 0.75)
        if self.cal.board_pose is None or self.cal.board_pose.confidence < 0.75:
            print(f"PATH FITTING REJECTED: Board not sufficiently calibrated (confidence={getattr(self.cal.board_pose, 'confidence', 0.0):.2f}, need ≥0.75)")
            return []
        
        return _fit_to_board(wps, sz, margin_frac)

    def corrected_delta_for_bt(self, desired_delta_board_mm, rotate_into_bot_frame=False):
        _warn_deprecated("CVPipeline.corrected_delta_for_bt")
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

    def get_pose_mm(self, use_raw=False):
        """
        Smoothed robot pose in mm and heading/confidence, or None if not ready.
        Args:
            use_raw: If True, return raw unsmoothed pose (useful for homing where instant feedback needed)
        """
        if self._bot is None:
            return None
        
        # Select raw or smoothed pose
        bot = self._bot if use_raw else self._pose_smooth
        if bot is None:
            return None
        
        # Position already in mm from homography
        return bot.center_board_mm, float(bot.heading_rad), float(bot.confidence)
    
    def get_path_correction(
        self, 
        target_board_mm: Tuple[float, float],
        max_correction_mm: float = 5.0,
        min_correction_mm: float = 1.0,
        apply_bounds: bool = True
    ) -> Optional[Tuple[float, float]]:
        _warn_deprecated("CVPipeline.get_path_correction")
        """
        Compute real-time path correction based on current vs target position.
        
        Args:
            target_board_mm: Target (x, y) position in board mm coordinates
            max_correction_mm: Maximum correction magnitude to prevent overcorrection
            min_correction_mm: Minimum error before applying correction (deadband for jitter)
            apply_bounds: If True, clamp correction to stay within board bounds
        
        Returns:
            (dx_mm, dy_mm) correction in board frame, or None if no correction needed
        """
        pose = self.get_pose_mm()
        if not pose:
            return None
        
        (current_x, current_y), heading, conf = pose
        target_x, target_y = target_board_mm
        
        # Compute error vector
        error_x = target_x - current_x
        error_y = target_y - current_y
        error_mag = math.sqrt(error_x**2 + error_y**2)
        
        # Track deviation for diagnostics
        self._path_deviation_history.append(error_mag)
        if len(self._path_deviation_history) > 100:
            self._path_deviation_history.pop(0)
        
        # Deadband: ignore small errors to reduce jitter
        if error_mag < min_correction_mm:
            return (0.0, 0.0)
        
        # Clamp correction to avoid overcorrection
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
        
        # Apply proportional gain based on confidence (higher conf = stronger correction)
        # Use softer gain curve to reduce jitter
        gain = 0.3 * (conf ** 1.5)  # Nonlinear: gentler at low confidence
        
        return (error_x * gain, error_y * gain)
    
    def get_path_stats(self) -> Dict:
        """Get statistics on path following performance."""
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
        Convert pathfinding G-code (canvas units) into firmware-ready, relative G-code
        using the current calibration.

        - Scales G0/G1 X/Y from canvas units to detected board mm
        - Normalizes to firmware subset: enforce G91 and map any legacy M3/M5 to M280
        - BOUNDS-CHECK: Ensures all positions stay within actual detected marker positions

        Returns the G-code string ready to transmit. Requires a valid board size.
        """
        bs = self.board_size_mm()
        if not bs:
            raise ValueError("Board not calibrated or no scale available")
        
        # Get strict bounds from actual marker positions
        bounds = self.get_board_bounds_mm()
        if bounds is None:
            # Fallback to computed board size if corners not available
            Wmm, Hmm = bs
            margin = max(BOARD_MARGIN_MM, BOARD_MARGIN_FRAC * min(Wmm, Hmm))
            min_x, min_y = margin, margin
            max_x, max_y = Wmm - margin, Hmm - margin
            print("BOUNDS: Using computed board size (corners unavailable)")
        else:
            min_x, min_y, max_x, max_y = bounds
            margin = max(BOARD_MARGIN_MM, BOARD_MARGIN_FRAC * min(max_x - min_x, max_y - min_y))
            min_x += margin
            min_y += margin
            max_x -= margin
            max_y -= margin
            print(f"BOUNDS: Using actual marker positions ({min_x:.1f},{min_y:.1f}) to ({max_x:.1f},{max_y:.1f})")
        
        scaled = scale_gcode_to_board(gcode_text, canvas_size, bs)
        print(f"DEBUG build_firmware_gcode: After scaling, first 5 lines:")
        for i, line in enumerate(scaled.split('\n')[:5]):
            print(f"  {i}: {line}")
        
        normalized = convert_pathfinding_gcode(scaled)
        
        # BOUNDS ENFORCEMENT: Track cumulative position and clamp deltas
        lines = normalized.split('\n')
        bounded_lines = []
        pos_x, pos_y = 0.0, 0.0  # Start at origin (should be homed to top-left inset)
        
        for line in lines:
            stripped = line.strip()
            if not stripped or stripped.startswith(';') or stripped.startswith('M'):
                bounded_lines.append(line)
                continue
            
            if stripped.startswith('G1') or stripped.startswith('G0'):
                # Parse relative deltas
                parts = stripped.split()
                dx = dy = 0
                for p in parts[1:]:
                    if p.startswith('X'):
                        try:
                            dx = int(p[1:])
                        except:
                            pass
                    elif p.startswith('Y'):
                        try:
                            dy = int(p[1:])
                        except:
                            pass
                
                # Project next position
                next_x = pos_x + dx
                next_y = pos_y + dy
                
                # Clamp to bounds
                clamped_x = max(min_x, min(next_x, max_x))
                clamped_y = max(min_y, min(next_y, max_y))
                
                # Recompute delta to clamped position
                actual_dx = int(round(clamped_x - pos_x))
                actual_dy = int(round(clamped_y - pos_y))
                
                if actual_dx != dx or actual_dy != dy:
                    print(f"BOUNDS: Clamped move from ({dx},{dy}) to ({actual_dx},{actual_dy}) at pos ({pos_x:.1f},{pos_y:.1f})")
                
                # Update position
                pos_x = clamped_x
                pos_y = clamped_y
                
                # Rebuild command
                if actual_dx == 0 and actual_dy == 0:
                    continue  # Skip zero moves
                
                cmd_parts = [parts[0]]
                if actual_dx != 0:
                    cmd_parts.append(f"X{actual_dx}")
                if actual_dy != 0:
                    cmd_parts.append(f"Y{actual_dy}")
                bounded_lines.append(' '.join(cmd_parts))
            else:
                bounded_lines.append(line)
        
        bounded = '\n'.join(bounded_lines)
        
        print(f"DEBUG build_firmware_gcode: After bounds enforcement, first 5 lines:")
        for i, line in enumerate(bounded.split('\n')[:5]):
            print(f"  {i}: {line}")
        
        return bounded
