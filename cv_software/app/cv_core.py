from __future__ import annotations
from dataclasses import dataclass
from typing import Optional, Tuple, Dict, List, Callable
import numpy as np
import cv2
import math
import time
from app.utils import board_to_robot
import os
from app.path_parse import (
    load_gcode_file,
    scale_gcode_to_board,
)
from app.control import Waypoint
BOARD_MARGIN_MM = float(os.getenv("BOARD_MARGIN_MM", "20"))
BOARD_MARGIN_FRAC = float(os.getenv("BOARD_MARGIN_FRAC", "0.05"))
_POSE_ALPHA = 0.7
_CONF_MIN_BOARD = 0.60
_CONF_MIN_BOT = 0.30


CV_CORRECTION_RATE_HZ = float(os.getenv("CV_CORRECTION_RATE_HZ", "7.0"))
CV_MIN_ERROR_MM = float(os.getenv("CV_MIN_ERROR_MM", "2.5"))
CV_LOOKAHEAD_COUNT = int(os.getenv("CV_LOOKAHEAD_COUNT", "2"))
CV_CROSS_TRACK_GAIN = float(os.getenv("CV_CROSS_TRACK_GAIN", "2.0"))
CV_ADAPTIVE_STEP = bool(int(os.getenv("CV_ADAPTIVE_STEP", "1")))
CV_MAX_CORRECTION_MM = float(os.getenv("CV_MAX_CORRECTION_MM", "8.0"))
CV_MAX_POSE_AGE_S = float(os.getenv("CV_MAX_POSE_AGE_S", "3.0"))
CV_DYNAMIC_GAIN_SCALE = float(os.getenv("CV_DYNAMIC_GAIN_SCALE", "1.0"))
CV_WAYPOINT_HYSTERESIS = int(os.getenv("CV_WAYPOINT_HYSTERESIS", "3"))
CV_ARRIVAL_MM = float(os.getenv("CV_ARRIVAL_MM", "15.0"))

BOARD_CORNERS = {"TL": 0, "TR": 1, "BR": 2, "BL": 3}
BOT_MARKERS = (10, 11)

MARKER_DICT = os.getenv("MARKER_DICT", "4x4_100").upper()

BOT_BASELINE_MM = 100.0

KNOWN_BOARD_WIDTH_MM = float(800)
KNOWN_BOARD_HEIGHT_MM = float(400)

_COND_HARD_LIMIT = 5e7
_COND_WARN_LIMIT = 3e4

_HEADING_MAX_DEG = float(os.getenv("HEADING_MAX_DEG", "10.0"))
_HEADING_MIN_SAMPLES = int(os.getenv("HEADING_MIN_SAMPLES", "5"))
_HEADING_HISTORY_SIZE = int(os.getenv("HEADING_HISTORY_SIZE", "30"))



@dataclass
class CameraModel:
    K: Optional[np.ndarray] = None
    dist: Optional[np.ndarray] = None


@dataclass
class BoardPose:
    H_img2board: np.ndarray
    board_size_px: Tuple[int, int]
    confidence: float = 0.0
    corners_board_px: Optional[Dict[str, Tuple[float, float]]] = None


@dataclass
class BotPose:
    center_board_mm: Tuple[float, float]
    heading_rad: float
    baseline_mm: float
    confidence: float = 0.0


def _aruco_dict():
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
    """Configure ArUco detector parameters for FAST detection with good accuracy.
    
    Optimized for speed: reduced window sizes, no subpixel corner refinement, fewer iterations.
    Still maintains good detection for markers in normal viewing angles.
    """
    if hasattr(cv2.aruco, "DetectorParameters"):
        p = cv2.aruco.DetectorParameters()
    else:
        p = cv2.aruco.DetectorParameters_create()

    # Fast corner refinement (no subpixel refinement - HUGE speedup)
    if hasattr(p, "cornerRefinementMethod"):
        p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE  # FASTEST - skip refinement entirely
    if hasattr(p, "cornerRefinementMaxIterations"):
        p.cornerRefinementMaxIterations = 10  # Faster: was 50
    if hasattr(p, "cornerRefinementMinAccuracy"):
        p.cornerRefinementMinAccuracy = 0.01

    # STRICTER perimeter rates to reduce false positives
    # Spurious markers detected in logs (1,2,3,37,38,80,99) are likely artifacts, not real markers
    if hasattr(p, "minMarkerPerimeterRate"):
        p.minMarkerPerimeterRate = 0.02  # STRICTER: was 0.01 (require at least 2% of image perimeter)
    if hasattr(p, "maxMarkerPerimeterRate"):
        p.maxMarkerPerimeterRate = 3.0  # STRICTER: was 4.0 (was too lenient, allowing huge false detections)

    # CRITICAL: Fewer adaptive threshold window sizes = HUGE speedup
    # Was: 3-63 by 4 steps = 16 iterations per frame
    # Now: 5-21 by 8 steps = 3 iterations per frame (5x faster!)
    if hasattr(p, "adaptiveThreshWinSizeMin"):
        p.adaptiveThreshWinSizeMin = 5  # Faster: was 3
    if hasattr(p, "adaptiveThreshWinSizeMax"):
        p.adaptiveThreshWinSizeMax = 21  # Faster: was 63
    if hasattr(p, "adaptiveThreshWinSizeStep"):
        p.adaptiveThreshWinSizeStep = 8  # CRITICAL: was 4, now 8 (only 3 window sizes instead of 16)
    if hasattr(p, "adaptiveThreshConstant"):
        p.adaptiveThreshConstant = 7

    # STRICTER polygonal approximation to reject poor corner shapes
    if hasattr(p, "polygonalApproxAccuracyRate"):
        p.polygonalApproxAccuracyRate = 0.04  # STRICTER: was 0.05 (require better polygon fit)

    # STRICTER corner distance to reject markers with corners too close together
    if hasattr(p, "minCornerDistanceRate"):
        p.minCornerDistanceRate = 0.02  # STRICTER: was 0.01 (require corners farther apart)
    
    # STRICTER border error tolerance - reject markers with bit errors in border
    if hasattr(p, "maxErroneousBitsInBorderRate"):
        p.maxErroneousBitsInBorderRate = 0.5  # STRICTER: was 0.6 (require cleaner border)

    # STRICTER Otsu threshold to reduce low-contrast false positives
    if hasattr(p, "minOtsuStdDev"):
        p.minOtsuStdDev = 3.0  # STRICTER: was 2.0 (require more contrast)
    
    # Standard perspective removal
    if hasattr(p, "perspectiveRemovePixelPerCell"):
        p.perspectiveRemovePixelPerCell = 4
    if hasattr(p, "perspectiveRemoveIgnoredMarginPerCell"):
        p.perspectiveRemoveIgnoredMarginPerCell = 0.13
    
    # STRICTER error correction
    if hasattr(p, "errorCorrectionRate"):
        p.errorCorrectionRate = 0.5  # STRICTER: was 0.6 (reject more errors)
    
    # Disable aruco3 for speed
    if hasattr(p, "useAruco3Detection"):
        p.useAruco3Detection = False
    
    # STRICTER marker distance - require markers to be well-separated
    if hasattr(p, "minMarkerDistanceRate"):
        p.minMarkerDistanceRate = 0.02  # STRICTER: was 0.01

    return p


class _ArucoCompatDetector:
    def __init__(self):
        self._dict = _aruco_dict()
        self._params = _aruco_params()
        self._new = hasattr(cv2.aruco, "ArucoDetector")
        self._det = cv2.aruco.ArucoDetector(self._dict, self._params) if self._new else None

    def detect(self, gray):
        """
        Detect ArUco markers in grayscale image - OPTIMIZED FOR SPEED.
        Returns dict mapping marker ID to corner coordinates.
        Works for markers at any orientation (normal, rotated, upside-down).
        The ArUco library handles rotation internally, so markers are detected
        regardless of their orientation in the image.
        """
        # Fast path: downscale large images for 2x speedup (ArUco is scale-invariant)
        h, w = gray.shape[:2]
        if h > 1080 or w > 1440:  # Only downscale if image is large
            scale_factor = 0.5
            gray_scaled = cv2.resize(gray, None, fx=scale_factor, fy=scale_factor, interpolation=cv2.INTER_LINEAR)
        else:
            gray_scaled = gray
            scale_factor = 1.0
        
        # Skip CLAHE (very slow), use fast histogram equalization instead
        # Histogram equalization is ~10x faster and works well for markers
        try:
            enhanced = cv2.equalizeHist(gray_scaled)
        except Exception:
            enhanced = gray_scaled
        
        # Try detection with enhanced image
        if self._new:
            corners, ids, rejected = self._det.detectMarkers(enhanced)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(enhanced, self._dict, parameters=self._params)
        
        # Scale corners back up if we downscaled
        if scale_factor != 1.0 and corners is not None:
            corners = [c / scale_factor for c in corners]
        
        id_map: Dict[int, np.ndarray] = {}
        if ids is not None:
            for c, i in zip(corners, ids.flatten()):
                # Store corners - ArUco always returns 4 corners in consistent order
                # regardless of marker orientation (the library handles rotation internally)
                # Corners are in order: top-left, top-right, bottom-right, bottom-left
                id_map[int(i)] = c.reshape(-1, 2).astype(np.float32)
        
        return id_map


def _conf_from_visibility(found_corners: int, total: int = 4) -> float:
    if found_corners < 3:
        return 0.0
    return min(1.0, found_corners / float(total))


class BoardCalibrator:
    def __init__(self, cam: Optional[CameraModel] = None):
        self.det = _ArucoCompatDetector()
        self.cam = cam or CameraModel()
        self.board_pose: Optional[BoardPose] = None
        self.pose_fresh: bool = False
        self.calibration_locked: bool = False
        
        # Temporal filtering for homography stability
        self._homography_history: List[Tuple[np.ndarray, float, float]] = []  # (H, confidence, timestamp)
        self._homography_history_max = 5
        self._homography_alpha = 0.3  # EMA weight for temporal smoothing
        
        # Outlier rejection thresholds
        self._max_corner_jump_px = 50.0  # Max pixel movement between frames
        self._min_quad_area_ratio = 0.3  # Min area ratio vs expected
        self._max_quad_area_ratio = 3.0  # Max area ratio vs expected

    def _center_from_quad(self, tl, tr, br, bl) -> Tuple[float, float]:
        c = 0.25 * (tl + tr + br + bl)
        return float(c[0]), float(c[1])

    def _undistort(self, frame_bgr):
        # Skip undistortion if no camera calibration available (optimization #5)
        if self.cam.K is None or self.cam.dist is None:
            return frame_bgr
        # Skip if distortion coefficients are negligible
        if self.cam.dist is not None and np.allclose(self.cam.dist, 0.0, atol=1e-6):
            return frame_bgr
        h, w = frame_bgr.shape[:2]
        K_new, _ = cv2.getOptimalNewCameraMatrix(self.cam.K, self.cam.dist, (w, h), 1.0, (w, h))
        return cv2.undistort(frame_bgr, self.cam.K, self.cam.dist, None, K_new)

    def _collect_corners(self, id_map: Dict[int, np.ndarray]) -> Dict[str, np.ndarray]:
        """Collect corners with centroid computation for stability."""
        have: Dict[str, np.ndarray] = {}
        for name, mid in BOARD_CORNERS.items():
            if mid in id_map:
                # Use centroid of marker corners for more stable position
                corners = id_map[mid]
                if len(corners) >= 4:
                    # Compute centroid (mean of all 4 corners)
                    centroid = corners.mean(axis=0)
                    have[name] = centroid
                else:
                    have[name] = corners.mean(axis=0)
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
    
    def _validate_quad_geometry(self, cs: Dict[str, np.ndarray]) -> bool:
        """
        Validate that the four corners form a reasonable rectangle.
        Checks area, aspect ratio, and that corners are in expected order.
        """
        if not all(k in cs for k in ("TL", "TR", "BR", "BL")):
            return False
        
        tl = cs["TL"]
        tr = cs["TR"]
        br = cs["BR"]
        bl = cs["BL"]
        
        # Compute side lengths
        top_len = float(np.linalg.norm(tr - tl))
        right_len = float(np.linalg.norm(br - tr))
        bottom_len = float(np.linalg.norm(bl - br))
        left_len = float(np.linalg.norm(tl - bl))
        
        # Check for degenerate cases
        if top_len < 10.0 or right_len < 10.0 or bottom_len < 10.0 or left_len < 10.0:
            return False
        
        # Compute area using cross product
        v1 = tr - tl
        v2 = bl - tl
        area = abs(v1[0] * v2[1] - v1[1] * v2[0])
        
        # Expected area (rough estimate based on known board size)
        # This is approximate - actual area depends on camera distance/angle
        expected_area = top_len * right_len  # Rough estimate
        
        if expected_area < 1e-6:
            return False
        
        area_ratio = area / expected_area
        
        # Check area ratio is reasonable
        if area_ratio < self._min_quad_area_ratio or area_ratio > self._max_quad_area_ratio:
            return False
        
        # Check aspect ratio is reasonable (board is roughly 2:1)
        aspect_ratio = max(top_len, bottom_len) / max(left_len, right_len)
        if aspect_ratio < 0.3 or aspect_ratio > 3.0:
            return False
        
        return True
    
    def _check_corner_consistency(self, cs: Dict[str, np.ndarray]) -> bool:
        """
        Check if detected corners are consistent with previous calibration.
        Rejects large jumps that likely indicate detection errors.
        """
        if self.board_pose is None:
            return True  # No previous pose to compare against
        
        if not all(k in cs for k in ("TL", "TR", "BR", "BL")):
            return True  # Can't validate incomplete quad
        
        try:
            H_prev = self.board_pose.H_img2board
            Hinv = np.linalg.inv(H_prev)
            
            # Transform current corners to board space
            for name, corner_img in cs.items():
                corner_pt = np.array([[corner_img[0], corner_img[1]]], dtype=np.float32)
                corner_board = cv2.perspectiveTransform(corner_pt[None, :, :], Hinv)[0][0]
                
                # Expected board position for this corner
                if name == "TL":
                    expected = np.array([0.0, KNOWN_BOARD_HEIGHT_MM])
                elif name == "TR":
                    expected = np.array([KNOWN_BOARD_WIDTH_MM, KNOWN_BOARD_HEIGHT_MM])
                elif name == "BR":
                    expected = np.array([KNOWN_BOARD_WIDTH_MM, 0.0])
                else:  # BL
                    expected = np.array([0.0, 0.0])
                
                # Check deviation (should be small if calibration is stable)
                deviation = np.linalg.norm(corner_board - expected)
                if deviation > 50.0:  # Allow 50mm deviation
                    return False
        except Exception:
            # If transformation fails, allow the detection
            pass
        
        return True
    
    def _temporal_filter_homography(self, H_new: np.ndarray, conf: float) -> np.ndarray:
        """
        Apply minimal temporal filtering to homography for SPEED.
        Skip history tracking when calibration is locked (fast path).
        """
        # If calibration is locked and confident, skip filtering entirely (fastest path)
        if self.calibration_locked and self.board_pose is not None and self.board_pose.confidence > 0.7:
            return H_new  # Return immediately without any history tracking
        
        now = time.time()
        
        # Add to small history (only during unstable calibration)
        self._homography_history.append((H_new.copy(), conf, now))
        
        # Keep only last 2 (was 5)
        if len(self._homography_history) > 2:
            self._homography_history = self._homography_history[-2:]
        
        # Only use EMA if confidence is low
        if self.board_pose is not None and conf < 0.7 and len(self._homography_history) > 1:
            H_prev = self.board_pose.H_img2board
            alpha = 0.5  # Faster convergence than 0.3
            H_filtered = alpha * H_prev + (1.0 - alpha) * H_new
            if abs(H_filtered[2, 2]) > 1e-12:
                H_filtered = H_filtered / H_filtered[2, 2]
            return H_filtered
        
        return H_new



    def calibrate(self, frame_bgr, force_recalibrate: bool = True) -> Optional[BoardPose]:
        self.pose_fresh = False
        
        # Fast path: if calibration is locked and recent, return immediately
        if self.calibration_locked and self.board_pose is not None and self.board_pose.confidence > 0.6:
            return self.board_pose

        # Skip undistortion during startup (calibration not locked yet) - save processing time
        if self.calibration_locked:
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
        found = len(cs)
        conf_raw = _conf_from_visibility(found)
        prev = self.board_pose

        if found < 3:
            if not hasattr(self, '_last_reject_log') or time.time() - self._last_reject_log > 2.0:
                print(f"Calibration: {found}/4 corners (need ≥3)")
                self._last_reject_log = time.time()
            return prev

        if found == 3:
            cs = self._complete_quad(cs)
            if not hasattr(self, '_last_completion_log') or time.time() - self._last_completion_log > 3.0:
                print("Calibration: Completed quad from 3 corners")
                self._last_completion_log = time.time()

        if not all(k in cs for k in ("TL", "TR", "BR", "BL")):
            return prev

        # Validate quad geometry
        if not self._validate_quad_geometry(cs):
            if not hasattr(self, '_last_geom_reject_log') or time.time() - self._last_geom_reject_log > 3.0:
                print("[BOARD_CAL] Rejected: Invalid quad geometry (area/aspect ratio)")
                self._last_geom_reject_log = time.time()
            return prev
        
        # Check corner consistency with previous calibration
        if not self._check_corner_consistency(cs):
            if not hasattr(self, '_last_consistency_reject_log') or time.time() - self._last_consistency_reject_log > 3.0:
                print("[BOARD_CAL] Rejected: Corner positions inconsistent with previous calibration")
                self._last_consistency_reject_log = time.time()
            return prev

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

        # Adaptive RANSAC threshold based on marker confidence and corner count
        # Higher confidence → tighter threshold for better precision
        # Lower confidence → looser threshold to accept more uncertain matches
        base_threshold = 2.0
        if found == 4:  # All corners detected
            ransac_thresh = base_threshold * (1.0 - 0.3 * conf_raw)  # 1.4-2.0px
        else:  # 3 corners (one completed)
            ransac_thresh = base_threshold * 1.5  # More lenient: 3.0px
        
        # Compute homography with adaptive RANSAC
        H, mask = cv2.findHomography(img_pts, board_rect, cv2.RANSAC, ransac_thresh, maxIters=2000)
        if H is None:
            if not hasattr(self, '_last_h_fail_log') or time.time() - self._last_h_fail_log > 3.0:
                print(f"[BOARD_CAL] findHomography failed (corners={found}, thresh={ransac_thresh:.2f}px)")
                self._last_h_fail_log = time.time()
            return prev
        
        # Normalize homography
        if abs(H[2, 2]) > 1e-12:
            H = H / H[2, 2]
        
        # Check condition number before accepting
        cond_H = np.linalg.cond(H)
        if cond_H > _COND_HARD_LIMIT:
            if not hasattr(self, '_last_cond_reject_log') or time.time() - self._last_cond_reject_log > 3.0:
                print(f"[BOARD_CAL] Rejected: Ill-conditioned homography (cond={cond_H:.1e} > {_COND_HARD_LIMIT:.1e})")
                self._last_cond_reject_log = time.time()
            return prev
        
        # Apply temporal filtering for stability
        H = self._temporal_filter_homography(H, conf_raw)

        width_px = float(np.linalg.norm(tr - tl))
        height_px = float(np.linalg.norm(bl - tl))
        cond_H = np.linalg.cond(H)
        
        # Compute reprojection error for quality assessment
        reproj_pts = cv2.perspectiveTransform(board_rect.reshape(-1, 1, 2), np.linalg.inv(H)).reshape(-1, 2)
        reproj_error = np.mean(np.linalg.norm(img_pts - reproj_pts, axis=1))
        
        # Reject if reprojection error is too high (indicates poor fit)
        if reproj_error > 10.0:  # 10px threshold
            if not hasattr(self, '_last_reproj_reject_log') or time.time() - self._last_reproj_reject_log > 3.0:
                print(f"[BOARD_CAL] Rejected: High reprojection error {reproj_error:.2f}px (threshold 10.0px)")
                self._last_reproj_reject_log = time.time()
            return prev

        if conf_raw < _CONF_MIN_BOARD:
            if not hasattr(self, "_low_conf_log") or time.time() - self._low_conf_log > 2.0:
                print(
                    f"[BOARD_CAL] Low confidence: found {found}/4 corners, "
                    f"conf_raw={conf_raw:.2f}, cond={cond_H:.1e}"
                )
                print("[BOARD_CAL] Marking calibration as unstable until markers recover.")
                self._low_conf_log = time.time()

        if cond_H > _COND_WARN_LIMIT:
            if not hasattr(self, "_cond_log") or time.time() - self._cond_log > 3.0:
                # print(
                #     f"[BOARD_CAL] Homography condition number high: {cond_H:.1e}, "
                #     f"warn_limit={_COND_WARN_LIMIT}, hard_limit={_COND_HARD_LIMIT}"
                # )
                self._cond_log = time.time()

            accept = not (cond_H > _COND_HARD_LIMIT)
            rejection_reason = None

            c_new = np.array(self._center_from_quad(tl, tr, br, bl))

            try:
                Hinv = np.linalg.inv(prev.H_img2board)
                board_center = np.array(
                    [[0.5 * KNOWN_BOARD_WIDTH_MM, 0.5 * KNOWN_BOARD_HEIGHT_MM]],
                    dtype=np.float32,
                )[None]
                img_center_prev = cv2.perspectiveTransform(board_center, Hinv)[0][0]
                center_jump_mm = float(np.linalg.norm(c_new - img_center_prev))
                if center_jump_mm > 100.0:
                    accept = False
                    rejection_reason = f"center_jump {center_jump_mm:.1f}mm > 100mm"
            except Exception:
                pass
        elif prev is None:
            accept = True

        if not accept:
            if not hasattr(self, '_last_reject_reason_log') or time.time() - self._last_reject_reason_log > 3.0:
                # if rejection_reason:
                #     print(f"Calibration rejected: {rejection_reason}")
                self._last_reject_reason_log = time.time()
            return prev

        if prev is not None:
            conf_smooth = max(float(conf_raw), float(prev.confidence))
        else:
            conf_smooth = float(conf_raw)

        if prev is None or not hasattr(self, '_calibrated_once'):
            print(f"[CALIBRATION] Homography updated and LOCKED. Board mapped to {KNOWN_BOARD_WIDTH_MM}mm × {KNOWN_BOARD_HEIGHT_MM}mm")
            print("[CALIBRATION] Localization restored after instability.")
            self._calibrated_once = True
            self.calibration_locked = True

        self.board_pose = BoardPose(
            H_img2board=H,
            board_size_px=(int(round(width_px)), int(round(height_px))),
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
    def __init__(self, cal: BoardCalibrator):
        self.det = _ArucoCompatDetector()
        self.cal = cal
        self._last_valid_pose: Optional[Tuple[BotPose, float]] = None
        self._pose_timeout_s = 10.0  # Increased from 5.0 to handle intermittent marker loss (up to 10s of cached pose usage)

        self._heading_hist: List[Tuple[float, float]] = []
        self._heading_hist_max = _HEADING_HISTORY_SIZE
        
        # Pose validation and filtering
        self._position_history: List[Tuple[float, float, float]] = []  # (x, y, timestamp)
        self._position_history_max = 10
        self._max_position_jump_mm = 200.0  # Max position change between frames (increased for fast movement)
        self._min_baseline_mm = 85.0  # Min baseline length (accounts for extreme camera angles)
        self._max_baseline_mm = 260.0  # Max baseline length (accounts for perspective angle)
        
        # Adaptive baseline tracking
        self._baseline_history: List[float] = []
        self._baseline_history_max = 20
        self._adaptive_baseline_enabled = True

    @staticmethod
    def _center(c4):
        """Compute marker center from 4 corners. Works regardless of corner order/orientation."""
        # ArUco markers always have 4 corners in a specific order, but we just need the center
        # This works for any orientation (normal, rotated, upside-down)
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
        if N < _HEADING_MIN_SAMPLES:
            ux, uy = u[-1]
            return float(math.atan2(uy, ux))

        cos_thresh = math.cos(math.radians(_HEADING_MAX_DEG))

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
    
    def _filter_bot_markers(self, id_map: Dict[int, np.ndarray]) -> Dict[int, np.ndarray]:
        """
        Post-process detected markers: reject ALL spurious detections, keep ONLY bot markers 10 & 11.
        
        Key insight: The logs show spurious low-ID detections (0,1,2,3,etc) that are almost certainly
        false positives from the ArUco library. DO NOT try to "fix" marker 10 detection by claiming
        spurious markers - this causes massive baseline errors (300-400mm instead of ~100mm).
        
        Strategy: Simply filter to markers 10 & 11 ONLY. No spatial anchor search. If marker 10 is
        missing, return the single marker 11 and let cached pose handle it. The stricter ArUco
        parameters should improve marker 10 detection over time.
        """
        filtered = {}
        
        # Accept ONLY the real bot markers (10 and 11)
        # Reject everything else - they are false positives
        for marker_id in BOT_MARKERS:
            if marker_id in id_map:
                filtered[marker_id] = id_map[marker_id]
        
        return filtered

    def _update_adaptive_baseline(self, baseline: float) -> None:
        """Update adaptive baseline range based on observed values."""
        if not self._adaptive_baseline_enabled:
            return
        
        self._baseline_history.append(baseline)
        if len(self._baseline_history) > self._baseline_history_max:
            self._baseline_history.pop(0)
        
        if len(self._baseline_history) >= 5:
            # Calculate mean and std of recent baselines
            mean_baseline = sum(self._baseline_history) / len(self._baseline_history)
            std_baseline = math.sqrt(sum((b - mean_baseline) ** 2 for b in self._baseline_history) / len(self._baseline_history))
            
            # Use mean ± 2.5*std for more lenient range, but ensure minimum range
            # Add padding to avoid edge cases (e.g., 127.1mm rejected when min is 127.2mm)
            padding = max(10.0, std_baseline * 0.5)  # At least 10mm padding, or 0.5*std
            adaptive_min = max(70.0, mean_baseline - 2.5 * std_baseline - padding)
            adaptive_max = min(300.0, mean_baseline + 2.5 * std_baseline + padding)
            
            # Ensure minimum range of at least 100mm to handle camera angle variation
            if adaptive_max - adaptive_min < 100.0:
                center = (adaptive_min + adaptive_max) / 2.0
                adaptive_min = max(70.0, center - 50.0)
                adaptive_max = min(300.0, center + 50.0)
            
            # Only update if the adaptive range is reasonable
            if adaptive_max - adaptive_min >= 50.0:
                self._min_baseline_mm = adaptive_min
                self._max_baseline_mm = adaptive_max
    
    def _validate_pose(self, center: np.ndarray, baseline: float) -> bool:
        """
        Validate that the detected pose is reasonable.
        Checks baseline length and position jumps.
        Uses adaptive baseline range if enabled.
        """
        # Update adaptive baseline range
        self._update_adaptive_baseline(baseline)
        
        # Check baseline length (use adaptive range if available)
        min_baseline = self._min_baseline_mm
        max_baseline = self._max_baseline_mm
        
        if baseline < min_baseline or baseline > max_baseline:
            if not hasattr(self, '_last_pose_reject_log') or time.time() - self._last_pose_reject_log > 2.0:
                print(f"[BOT_TRACK] Rejected pose: baseline={baseline:.1f}mm (range: {min_baseline:.1f}-{max_baseline:.1f}mm), center=({center[0]:.1f},{center[1]:.1f})")
                self._last_pose_reject_log = time.time()
            return False
        
        # Check position jump (adaptive based on time since last pose)
        if self._last_valid_pose is not None:
            last_pose, last_time = self._last_valid_pose
            last_center = np.array(last_pose.center_board_mm)
            jump = float(np.linalg.norm(center - last_center))
            time_delta = time.time() - last_time
            
            # Adaptive jump threshold: allow larger jumps if more time has passed
            # Base threshold: 200mm, but scale up to 400mm if >1 second has passed
            adaptive_jump_threshold = self._max_position_jump_mm
            if time_delta > 1.0:
                adaptive_jump_threshold = min(400.0, self._max_position_jump_mm * (1.0 + time_delta * 0.2))
            
            if jump > adaptive_jump_threshold:
                if not hasattr(self, '_last_jump_reject_log') or time.time() - self._last_jump_reject_log > 2.0:
                    print(f"[BOT_TRACK] Rejected pose: position jump={jump:.1f}mm (threshold: {adaptive_jump_threshold:.1f}mm, dt={time_delta:.2f}s)")
                    self._last_jump_reject_log = time.time()
                return False
        
        return True

    def track(self, frame_bgr) -> Optional[BotPose]:
        """
        Track bot position using ArUco markers.
        Works for markers at any camera angle or orientation (including upside-down).
        """
        if self.cal.board_pose is None:
            return None

        # Undistort image first (removes camera distortion)
        frame_bgr = self.cal._undistort(frame_bgr)
        
        # Convert to grayscale for marker detection
        # The detect() method will apply additional enhancement (CLAHE) for better angle tolerance
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)

        # Detect markers (handles any orientation automatically)
        det_out = self.det.detect(gray)
        if isinstance(det_out, tuple) and len(det_out) >= 3:
            id_map = det_out[2]
        else:
            id_map = det_out

        if id_map is None:
            id_map = {}
        
        # Filter out spurious detections, keep only bot markers
        # This prevents false baseline calculations from spurious detections
        id_map = self._filter_bot_markers(id_map)

        has_marker_0 = BOT_MARKERS[0] in id_map
        has_marker_1 = BOT_MARKERS[1] in id_map
        # Marker visibility diagnostics: list all detected IDs this frame
        try:
            if not hasattr(self, '_last_vis_log') or time.time() - self._last_vis_log > 0.75:
                ids_list = sorted(list(id_map.keys()))
                print(f"BOT_TRACK markers={ids_list} has_markers: {has_marker_0} {has_marker_1}")
                self._last_vis_log = time.time()
        except Exception:
            pass

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
        
        # Get marker corners - ArUco library handles orientation internally
        # Corners are always returned in consistent marker-local order regardless of image orientation
        marker0_corners = id_map[BOT_MARKERS[0]]
        marker1_corners = id_map[BOT_MARKERS[1]]
        
        # Compute centers (works for any orientation - center is invariant to rotation)
        # The center calculation is robust because it's just the mean of 4 points
        c0 = self._center(marker0_corners)
        c1 = self._center(marker1_corners)
        
        # Quick sanity check: markers should be reasonably close in image space
        # If they're very far apart in image, likely a detection error
        img_dist = float(np.linalg.norm(c0 - c1))
        if img_dist < 10.0 or img_dist > 2000.0:  # Unreasonably close or far in pixels
            if not hasattr(self, '_last_img_dist_warn') or time.time() - self._last_img_dist_warn > 3.0:
                print(f"[BOT_TRACK] WARNING: Suspicious marker separation in image: {img_dist:.1f}px (likely detection error)")
                self._last_img_dist_warn = time.time()
            # Return cached pose if available
            if self._last_valid_pose is not None:
                cached_pose, cached_time = self._last_valid_pose
                age = time.time() - cached_time
                if age < self._pose_timeout_s:
                    return cached_pose
            return None
        
        # Transform to board coordinates
        pair_board = self._warp(H, np.vstack([c0, c1]))

        # Compute baseline vector (orientation-independent)
        # The vector from marker 0 to marker 1 defines the robot's forward direction
        v = pair_board[1] - pair_board[0]
        center = 0.5 * (pair_board[0] + pair_board[1])
        baseline = float(np.linalg.norm(v))
        
        # The baseline should be positive regardless of which marker is "first"
        # If baseline is negative (markers swapped), we can handle it, but the norm is always positive

        # Pre-validate baseline before full pose validation (catch obvious errors early)
        if baseline < 30.0 or baseline > 300.0:  # Hard limits - anything outside is definitely wrong
            if not hasattr(self, '_last_baseline_hard_reject') or time.time() - self._last_baseline_hard_reject > 2.0:
                print(f"[BOT_TRACK] ERROR: Invalid baseline {baseline:.1f}mm (hard limits: 30-300mm) - likely marker misdetection")
                self._last_baseline_hard_reject = time.time()
            # Return cached pose if available
            if self._last_valid_pose is not None:
                cached_pose, cached_time = self._last_valid_pose
                age = time.time() - cached_time
                if age < self._pose_timeout_s:
                    return cached_pose
            return None

        # Validate pose before using it
        if not self._validate_pose(center, baseline):
            # Return cached pose if available
            if self._last_valid_pose is not None:
                cached_pose, cached_time = self._last_valid_pose
                age = time.time() - cached_time
                if age < self._pose_timeout_s:
                    return cached_pose
            return None

        # Normalize heading vector for consistent direction (works for any orientation)
        # This ensures heading is computed correctly regardless of marker orientation
        v_normalized = v / baseline if baseline > 1e-6 else v
        
        self._add_heading_sample(v_normalized)
        heading_robust = self._robust_heading()
        if heading_robust is None:
            # Compute heading from normalized vector (works for any orientation)
            heading = float(np.arctan2(v[1], v[0]))
        else:
            heading = heading_robust
        
        # Heading is now in [-pi, pi] range, which works for any orientation
        # The heading represents the direction from marker 0 to marker 1 in board coordinates

        baseline_ratio = baseline / BOT_BASELINE_MM
        conf = float(np.clip(baseline_ratio, 0.8, 1.0))
        
        # Update position history for velocity estimation
        now = time.time()
        self._position_history.append((float(center[0]), float(center[1]), now))
        if len(self._position_history) > self._position_history_max:
            self._position_history.pop(0)

        pose = BotPose(
            center_board_mm=(float(center[0]), float(center[1])),
            heading_rad=heading,
            baseline_mm=baseline,
            confidence=conf,
        )
        # print(f"BOT_TRACK: Valid pose: center=({center[0]:.1f},{center[1]:.1f})mm heading={math.degrees(heading):.1f}° baseline={baseline:.1f}mm conf={conf:.2f}")
        self._last_valid_pose = (pose, time.time())
        return pose

    def pose_age_s(self) -> Optional[float]:
        """Return seconds since the last verified bot pose, or None if never."""
        if self._last_valid_pose is None:
            return None
        _, ts = self._last_valid_pose
        return max(0.0, time.time() - ts)


class PathCorrectionEngine:
    """
    Computes bounded board-frame correction deltas based on cross-track error
    relative to the current segment of the path. The server converts these
    deltas into safe robot-frame steps via _map_board_to_robot_steps().
    
    Improved algorithm features:
    - Cross-track error computation (perpendicular distance to path segment)
    - Lookahead target selection for smoother corrections
    - Heading-aware corrections that account for robot orientation
    - Adaptive gains based on error magnitude and path curvature
    - Path curvature estimation for better corner handling
    """

    def __init__(self, cvp: "CVPipeline"):
        self.cvp = cvp
        self.last_correction_time = 0.0
        self.correction_count = 0
        self.min_correction_interval = 1.0 / CV_CORRECTION_RATE_HZ
        self.cross_track_gain = CV_CROSS_TRACK_GAIN
        self.min_error_mm = CV_MIN_ERROR_MM
        self.lookahead = max(1, CV_LOOKAHEAD_COUNT)
        self.adaptive_step = CV_ADAPTIVE_STEP
        self.max_correction_mm = CV_MAX_CORRECTION_MM
        self.max_pose_age_s = CV_MAX_POSE_AGE_S
        self._last_stale_log = 0.0

        # Deviation history for RMS stats (filled externally)
        self.dev_history: list[float] = []
        
        # Adaptive correction parameters
        self.lookahead_distance_mm = float(os.getenv("CV_LOOKAHEAD_DISTANCE_MM", "40.0"))
        self.base_gain = float(os.getenv("CV_BASE_GAIN", "0.4"))
        self.max_gain = float(os.getenv("CV_MAX_GAIN", "0.7"))
        self.curvature_gain_scale = float(os.getenv("CV_CURVATURE_GAIN_SCALE", "1.2"))

    def reset(self) -> None:
        self.last_correction_time = 0.0
        self.correction_count = 0
        self.dev_history.clear()

    def _pose(self) -> Optional[tuple[float, float, float, float]]:
        pose = self.cvp.get_pose_mm()
        if not pose:
            return None
        (cx, cy), heading, conf = pose
        return float(cx), float(cy), float(heading), float(conf)

    @staticmethod
    def _wp_xy(wp: object) -> Tuple[float, float]:
        if hasattr(wp, "x_mm") and hasattr(wp, "y_mm"):
            return float(getattr(wp, "x_mm")), float(getattr(wp, "y_mm"))
        if isinstance(wp, (tuple, list)) and len(wp) >= 2:
            return float(wp[0]), float(wp[1])
        raise ValueError("Waypoint must have x_mm/y_mm or be (x, y)")
    
    def _compute_cross_track_error(
        self, 
        x: float, 
        y: float, 
        wp1: Tuple[float, float], 
        wp2: Tuple[float, float]
    ) -> Tuple[float, float, float]:
        """
        Compute cross-track error (perpendicular distance to path segment).
        Returns: (signed_cross_track_error, along_track_distance, segment_length)
        """
        x1, y1 = wp1
        x2, y2 = wp2
        
        # Segment vector
        seg_dx = x2 - x1
        seg_dy = y2 - y1
        seg_len = math.hypot(seg_dx, seg_dy)
        
        if seg_len < 1e-6:
            # Degenerate segment - use point distance
            dist = math.hypot(x - x1, y - y1)
            return dist, 0.0, 0.0
        
        # Normalized segment direction
        seg_ux = seg_dx / seg_len
        seg_uy = seg_dy / seg_len
        
        # Normal vector (perpendicular, pointing left of path direction)
        norm_x = -seg_uy
        norm_y = seg_ux
        
        # Vector from wp1 to robot
        rx = x - x1
        ry = y - y1
        
        # Project onto segment (along-track distance)
        along = rx * seg_ux + ry * seg_uy
        
        # Cross-track error (signed, positive = left of path)
        cross = rx * norm_x + ry * norm_y
        
        return cross, along, seg_len
    
    def _get_lookahead_target(
        self, 
        waypoints: list, 
        current_index: int, 
        robot_x: float, 
        robot_y: float
    ) -> Optional[Tuple[float, float, int]]:
        """
        Find a target point ahead on the path at approximately lookahead_distance_mm.
        Returns: (target_x, target_y, segment_index) or None
        """
        if current_index >= len(waypoints):
            return None
        
        lookahead_dist = self.lookahead_distance_mm
        accumulated_dist = 0.0
        
        # Start from current waypoint
        for i in range(current_index, len(waypoints) - 1):
            wp1 = self._wp_xy(waypoints[i])
            wp2 = self._wp_xy(waypoints[i + 1])
            
            seg_len = math.hypot(wp2[0] - wp1[0], wp2[1] - wp1[1])
            
            if accumulated_dist + seg_len >= lookahead_dist:
                # Interpolate along this segment
                remaining = lookahead_dist - accumulated_dist
                if seg_len > 1e-6:
                    t = remaining / seg_len
                    target_x = wp1[0] + t * (wp2[0] - wp1[0])
                    target_y = wp1[1] + t * (wp2[1] - wp1[1])
                    return (target_x, target_y, i)
                else:
                    return (wp2[0], wp2[1], i)
            
            accumulated_dist += seg_len
        
        # If path is shorter than lookahead, use last waypoint
        if waypoints:
            last_wp = self._wp_xy(waypoints[-1])
            return (last_wp[0], last_wp[1], len(waypoints) - 1)
        
        return None
    
    def _estimate_path_curvature(
        self, 
        waypoints: list, 
        segment_idx: int
    ) -> float:
        """
        Estimate path curvature at the given segment.
        Returns curvature (1/radius in mm^-1), higher = sharper turn.
        """
        if segment_idx < 0 or segment_idx >= len(waypoints) - 1:
            return 0.0
        
        # Need at least 3 points to estimate curvature
        if segment_idx == 0 and len(waypoints) < 3:
            return 0.0
        
        # Get three consecutive waypoints
        if segment_idx > 0:
            wp0 = self._wp_xy(waypoints[segment_idx - 1])
        else:
            wp0 = self._wp_xy(waypoints[0])
        
        wp1 = self._wp_xy(waypoints[segment_idx])
        wp2 = self._wp_xy(waypoints[min(segment_idx + 1, len(waypoints) - 1)])
        
        # Vectors
        v1x = wp1[0] - wp0[0]
        v1y = wp1[1] - wp0[1]
        v2x = wp2[0] - wp1[0]
        v2y = wp2[1] - wp1[1]
        
        len1 = math.hypot(v1x, v1y)
        len2 = math.hypot(v2x, v2y)
        
        if len1 < 1e-6 or len2 < 1e-6:
            return 0.0
        
        # Normalize
        v1x /= len1
        v1y /= len1
        v2x /= len2
        v2y /= len2
        
        # Angle between vectors (dot product)
        dot = v1x * v2x + v1y * v2y
        dot = max(-1.0, min(1.0, dot))  # Clamp for acos
        angle = math.acos(dot)
        
        # Curvature approximation: angle / average segment length
        avg_len = 0.5 * (len1 + len2)
        if avg_len < 1e-6:
            return 0.0
        
        # Curvature in mm^-1 (higher for sharper turns)
        curvature = angle / avg_len
        
        return curvature
    
    def compute_correction(
        self,
        waypoints: list,
        current_index: int,
        next_is_g1: bool = True,
    ) -> Optional[Tuple[float, float]]:
        """
        Improved correction algorithm using cross-track error, lookahead, and heading.
        
        Key improvement: Allows corrections with moderately stale poses (up to 3x max age)
        if the robot is significantly off the path (error > 50mm). This handles cases where
        marker 10 is intermittently lost but robot still needs course corrections.
        """

        if not next_is_g1:
            return None

        if current_index >= len(waypoints):
            return None

        pose = self._pose()
        if pose is None:
            return None
        cx, cy, heading, conf = pose
        pose_age = self.cvp.pose_age_s()
        
        # Check for stale pose and decide if we can still correct
        if pose_age is not None and pose_age > self.max_pose_age_s:
            # Pose is stale - but check if error is large enough to justify correction anyway
            try:
                tx, ty = self._wp_xy(waypoints[current_index])
                error_mag = math.hypot(tx - cx, ty - cy)
                
                # Strategy:
                # - If error is huge (>100mm), allow correction even with very stale pose (scale heavily)
                # - If error is large (>50mm) and pose is < 3x stale, allow it (scale moderately)
                # - Otherwise, skip correction (pose too stale and error small)
                
                too_old = pose_age > (self.max_pose_age_s * 3.0)  # Way too stale
                moderately_old = pose_age > (self.max_pose_age_s * 1.5)  # Pretty stale
                
                if error_mag > 100.0 and pose_age <= (self.max_pose_age_s * 5.0):
                    # Huge error even with very stale pose - we NEED to correct
                    pass  # Allow correction, will scale aggressively
                elif error_mag > 50.0 and not too_old:
                    # Large error, moderately stale pose - allow correction with scaling
                    pass  # Allow correction
                else:
                    # Either error is small or pose is way too stale
                    now = time.time()
                    if now - self._last_stale_log > 1.5:
                        reason = "error small" if error_mag <= 50.0 else "pose way too stale"
                        print(
                            f"[CORRECTION] skipping: pose stale ({pose_age:.2f}s > {self.max_pose_age_s:.2f}s, error={error_mag:.1f}mm, reason={reason})"
                        )
                        self._last_stale_log = now
                    return None
            except Exception as e:
                # If we can't check error, be conservative
                now = time.time()
                if now - self._last_stale_log > 1.5:
                    print(
                        f"[CORRECTION] skipping: pose stale ({pose_age:.2f}s > {self.max_pose_age_s:.2f}s, check_error_failed: {e})"
                    )
                    self._last_stale_log = now
                return None

        # Get lookahead target for smoother corrections
        lookahead_target = self._get_lookahead_target(waypoints, current_index, cx, cy)
        if lookahead_target is None:
            return None
        
        target_x, target_y, segment_idx = lookahead_target
        
        # Get current segment endpoints
        if segment_idx >= len(waypoints) - 1:
            # At last waypoint, use simple distance
            error_x = target_x - cx
            error_y = target_y - cy
            error_mag = math.hypot(error_x, error_y)
            
            if error_mag < self.min_error_mm:
                return None
            
            # Simple proportional correction
            gain = self.base_gain
            
            # Scale down gain if pose is stale
            pose_age = self.cvp.pose_age_s()
            if pose_age is not None and pose_age > self.max_pose_age_s:
                stale_factor = max(0.3, 1.0 - (pose_age - self.max_pose_age_s) / (self.max_pose_age_s * 2.0))
                gain *= stale_factor
            
            dx_board = error_x * gain
            dy_board = error_y * gain
        else:
            # Compute cross-track error
            wp1 = self._wp_xy(waypoints[segment_idx])
            wp2 = self._wp_xy(waypoints[segment_idx + 1])
            
            cross_error, along_dist, seg_len = self._compute_cross_track_error(cx, cy, wp1, wp2)
            
            # Estimate path curvature
            curvature = self._estimate_path_curvature(waypoints, segment_idx)
            
            # Adaptive gain based on error magnitude and curvature
            error_mag = abs(cross_error)
            if error_mag < self.min_error_mm:
                return None
            
            # Base gain increases with error (up to max_gain)
            normalized_error = min(error_mag / 20.0, 1.0)  # Normalize to 0-1 for 20mm max
            gain = self.base_gain + (self.max_gain - self.base_gain) * normalized_error
            
            # Scale down gain if pose is stale (but still allow correction)
            pose_age = self.cvp.pose_age_s()
            if pose_age is not None and pose_age > self.max_pose_age_s:
                # Scale down more aggressively for stale poses
                # At max_pose_age_s: scale = 1.0 (no reduction)
                # At 3x max_pose_age_s: scale = 0.3 (70% reduction)
                # At 5x max_pose_age_s: scale = 0.1 (90% reduction)
                age_over_threshold = pose_age - self.max_pose_age_s
                stale_factor = max(0.1, 1.0 / (1.0 + age_over_threshold / self.max_pose_age_s))
                gain *= stale_factor
                if pose_age > self.max_pose_age_s * 2.0:
                    print(f"[CORRECTION] Very stale pose ({pose_age:.1f}s), scaling gain by {stale_factor:.2f}")
            
            # Increase gain for sharp turns
            if curvature > 0.01:  # Significant curvature
                gain *= self.curvature_gain_scale
            
            # Compute correction direction (perpendicular to path, toward path)
            seg_dx = wp2[0] - wp1[0]
            seg_dy = wp2[1] - wp1[1]
            seg_len_check = math.hypot(seg_dx, seg_dy)
            
            if seg_len_check > 1e-6:
                seg_ux = seg_dx / seg_len_check
                seg_uy = seg_dy / seg_len_check
                # Normal vector (perpendicular, pointing left)
                norm_x = -seg_uy
                norm_y = seg_ux
                
                # Correction is opposite of cross-track error
                corr_mag = cross_error * gain
                dx_board = -norm_x * corr_mag
                dy_board = -norm_y * corr_mag
                
                # Also add small along-track component to help reach lookahead target
                along_gain = 0.15  # Smaller gain for along-track
                along_error = target_x - cx
                along_error_y = target_y - cy
                along_dist_to_target = math.hypot(along_error, along_error_y)
                
                if along_dist_to_target > 1e-6:
                    # Project onto segment direction
                    along_component = (along_error * seg_ux + along_error_y * seg_uy) * along_gain
                    dx_board += seg_ux * along_component
                    dy_board += seg_uy * along_component
            else:
                # Degenerate segment - fall back to direct correction
                error_x = target_x - cx
                error_y = target_y - cy
                error_mag = math.hypot(error_x, error_y)
                if error_mag < self.min_error_mm:
                    return None
                dx_board = error_x * gain
                dy_board = error_y * gain

        # Clamp correction magnitude
        corr_mag = math.hypot(dx_board, dy_board)
        if corr_mag > self.max_correction_mm:
            scale = self.max_correction_mm / corr_mag
            dx_board *= scale
            dy_board *= scale

        # Ensure correction doesn't take us outside bounds
        bounds = self.cvp.get_board_bounds_mm()
        if bounds:
            min_x, min_y, max_x, max_y = bounds
            final_x = cx + dx_board
            final_y = cy + dy_board
            
            # Clamp final position to bounds and recompute delta
            final_x = max(min_x, min(final_x, max_x))
            final_y = max(min_y, min(final_y, max_y))
            dx_board = final_x - cx
            dy_board = final_y - cy

        if abs(dx_board) < 1e-3 and abs(dy_board) < 1e-3:
            return None

        # Compute final error magnitude for logging
        final_error_mag = math.hypot(target_x - cx, target_y - cy)
        final_gain = corr_mag / final_error_mag if final_error_mag > 1e-6 else 0.0
        print(f"[CORRECTION] error={final_error_mag:.1f}mm gain={final_gain:.2f} → correction=({dx_board:.1f},{dy_board:.1f})mm")
        return dx_board, dy_board


    def log_deviation(self, d_mm: float) -> None:
        self.dev_history.append(float(d_mm))
        if len(self.dev_history) > 2000:
            self.dev_history.pop(0)

    def rms_deviation(self) -> float:
        if not self.dev_history:
            return 0.0
        s = sum(d * d for d in self.dev_history)
        return math.sqrt(s / len(self.dev_history))



class CVPipeline:
    def __init__(self, cam: Optional[CameraModel] = None):
        self.cal = BoardCalibrator(cam)
        self.trk = BotTracker(self.cal)
        self._bot: Optional[BotPose] = None
        self._pose_smooth: Optional[BotPose] = None
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

        prev_state = self.get_state()
        prev_calibrated = prev_state["calibrated"]

        self.cal.calibrate(frame_bgr)

        st = self.get_state()
        calibrated = st["calibrated"]
        conf = st["board_confidence"]

        if not calibrated:
            if not hasattr(self, "_no_cal_log") or time.time() - self._no_cal_log > 3.0:
                print("[CALIBRATION] No valid board calibration. Waiting for stable corner markers.")
                self._no_cal_log = time.time()
        elif conf is not None and conf < _CONF_MIN_BOARD:
            if not hasattr(self, "_low_conf_frame_log") or time.time() - self._low_conf_frame_log > 3.0:
                print(f"[CALIBRATION] Runtime board confidence {conf:.2f} < {_CONF_MIN_BOARD:.2f}")
                print("[CALIBRATION] Treating homography as unstable and continuously re-estimating from markers.")
                self._low_conf_frame_log = time.time()

        bot_ok = self.update_bot(frame_bgr)

        st = self.get_state()

        good_cal = (
            st["calibrated"]
            # and st["board_reproj_err_px"] is not None
            # and st["board_reproj_err_px"] <= _MAX_REPROJ_ERR_PX
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
        bot = self._pose_smooth or self._bot
        if bot is None:
            return None
        if bot.baseline_mm <= 1e-6:
            return None
        return BOT_BASELINE_MM / bot.baseline_mm

    def board_size_mm(self) -> Optional[Tuple[float, float]]:
        if self.cal.board_pose is None:
            return None
        return (KNOWN_BOARD_WIDTH_MM, KNOWN_BOARD_HEIGHT_MM)

    def get_board_bounds_mm(self) -> Optional[Tuple[float, float, float, float]]:
        sz = self.board_size_mm()
        if not sz:
            return None

        Wmm, Hmm = sz
        margin = max(BOARD_MARGIN_MM, BOARD_MARGIN_FRAC * min(Wmm, Hmm))
        safe_min_x = margin
        safe_min_y = margin
        safe_max_x = Wmm - margin
        safe_max_y = Hmm - margin
        return (safe_min_x, safe_min_y, safe_max_x, safe_max_y)

    def _safe_board_rect(self, margin_frac: float = 0.10) -> Optional[tuple[float, float, float, float]]:
        bounds = self.get_board_bounds_mm()
        if not bounds:
            return None
        min_x, min_y, max_x, max_y = bounds
        safe_w = max_x - min_x
        safe_h = max_y - min_y
        if safe_w <= 0 or safe_h <= 0:
            return None

        pad_x = margin_frac * 0.5 * safe_w
        pad_y = margin_frac * 0.5 * safe_h
        min_x2 = min_x + pad_x
        max_x2 = max_x - pad_x
        min_y2 = min_y + pad_y
        max_y2 = max_y - pad_y

        span_x = max_x2 - min_x2
        span_y = max_y2 - min_y2
        if span_x <= 0 or span_y <= 0:
            return None
        return (min_x2, min_y2, span_x, span_y)

    def fit_path_to_board(self, wps: list[Waypoint], margin_frac: float = 0.10) -> list[Waypoint]:
        """
        Take normalized waypoints (0..1 in both axes) and map them into the
        already-safe board bounds returned by get_board_bounds_mm().
        margin_frac optionally shrinks inside those safe bounds.
        All waypoints are validated and clamped to ensure they stay within bounds.
        """
        if not wps:
            return []

        rect = self._safe_board_rect(margin_frac)
        if rect is None:
            print("PATH FITTING REJECTED: No usable board bounds available.")
            return []

        min_x2, min_y2, span_x, span_y = rect
        
        # Get absolute bounds for validation
        bounds = self.get_board_bounds_mm()
        if not bounds:
            print("PATH FITTING REJECTED: No board bounds for validation.")
            return []
        min_x_abs, min_y_abs, max_x_abs, max_y_abs = bounds

        out: list[Waypoint] = []
        clamped_count = 0
        for i, p in enumerate(wps):
            # p.x_mm and p.y_mm are actually normalized [0,1] here
            nx = float(p.x_mm)
            ny = float(p.y_mm)

            x_mm = min_x2 + nx * span_x
            y_mm = min_y2 + ny * span_y
            
            # Validate and clamp to absolute bounds
            x_clamped = max(min_x_abs, min(x_mm, max_x_abs))
            y_clamped = max(min_y_abs, min(y_mm, max_y_abs))
            
            if abs(x_mm - x_clamped) > 0.1 or abs(y_mm - y_clamped) > 0.1:
                clamped_count += 1
                if clamped_count <= 3:  # Log first few to avoid spam
                    print(f"[FIT_PATH] Waypoint {i} clamped from ({x_mm:.1f},{y_mm:.1f}) to ({x_clamped:.1f},{y_clamped:.1f})")

            out.append(Waypoint(x_mm=x_clamped, y_mm=y_clamped))
        
        if clamped_count > 0:
            print(f"[FIT_PATH] Total {clamped_count}/{len(wps)} waypoints clamped to bounds")

        return out

    def map_normalized_point_to_board(
        self,
        nx: float,
        ny: float,
        margin_frac: float = 0.10,
    ) -> Optional[tuple[float, float]]:
        rect = self._safe_board_rect(margin_frac)
        if rect is None:
            return None
        min_x2, min_y2, span_x, span_y = rect
        nx = max(0.0, min(1.0, float(nx)))
        ny = max(0.0, min(1.0, float(ny)))
        x_mm = min_x2 + nx * span_x
        y_mm = min_y2 + ny * span_y
        return (x_mm, y_mm)



    def get_pose_mm(self, use_raw=False):
        if self._bot is None:
            return None

        bot = self._bot if use_raw else self._pose_smooth
        if bot is None:
            return None

        cx, cy = bot.center_board_mm
        return (cx, cy), float(bot.heading_rad), float(bot.confidence)

    def pose_age_s(self) -> Optional[float]:
        """Expose tracker pose age so correction logic can detect stale data."""
        if not hasattr(self, "trk") or self.trk is None:
            return None
        return self.trk.pose_age_s()
    def get_path_stats(self) -> Dict:
        if not self.correction.dev_history:
            return {}
        arr = np.asarray(self.correction.dev_history, dtype=float)
        return {
            "mean_deviation_mm": float(arr.mean()),
            "max_deviation_mm": float(arr.max()),
            "std_deviation_mm": float(arr.std()),
            "rms_deviation_mm": float(np.sqrt(np.mean(arr**2))),
        }


    def build_firmware_gcode(
        self,
        gcode_text: str,
        start_pos: Optional[Tuple[float, float]] = None,
        fitted_wps: Optional[List[Waypoint]] = None,
        norm_origin: Optional[Tuple[float, float]] = None,
        margin_frac: float = 0.10,
    ) -> str:
        bounds = self.get_board_bounds_mm()
        if bounds is None:
            raise ValueError("Board not calibrated – bounds unavailable")

        min_x, min_y, max_x, max_y = bounds

        def _planar_anchor() -> Tuple[float, float]:
            anchor = None
            # Prioritize actual start_pos (bot position from markers)
            if anchor is None and start_pos is not None:
                anchor = start_pos
                print(f"[GCODE] Using bot marker position as anchor: {anchor}")
            if anchor is None and norm_origin is not None:
                anchor = self.map_normalized_point_to_board(norm_origin[0], norm_origin[1], margin_frac=margin_frac)
                print(f"[GCODE] Using normalized origin {norm_origin} → board pos {anchor}")
            if anchor is None and fitted_wps:
                first = fitted_wps[0]
                anchor = (first.x_mm, first.y_mm)
            if anchor is None:
                anchor = (min_x, min_y)
            ax, ay = anchor
            ax = min(max(ax, min_x), max_x)
            ay = min(max(ay, min_y), max_y)
            return ax, ay

        if fitted_wps:
            anchor = _planar_anchor()
            anchor = (start_pos[0], start_pos[1]) if start_pos is not None else anchor
            path_points: List[Tuple[float, float]] = [anchor] + [
                (float(wp.x_mm), float(wp.y_mm)) for wp in fitted_wps
            ]
            move_idx = 0
            pos_x, pos_y = anchor
            out_lines: List[str] = []

            for raw in gcode_text.splitlines():
                line = raw.strip()
                if not line or line.startswith(";"):
                    out_lines.append(raw)
                    continue

                upper = line.upper()
                if not upper.startswith(("G0", "G1")):
                    out_lines.append(raw)
                    continue

                if move_idx >= len(fitted_wps):
                    continue

                next_pt = path_points[move_idx + 1]
                dx = next_pt[0] - pos_x
                dy = next_pt[1] - pos_y
                pos_x, pos_y = next_pt
                move_idx += 1

                dx_i = int(round(dx))
                dy_i = int(round(dy))
                if dx_i == 0 and dy_i == 0:
                    continue

                cmd = line.split()[0]
                parts = [cmd]
                if dx_i != 0:
                    parts.append(f"X{dx_i}")
                if dy_i != 0:
                    parts.append(f"Y{dy_i}")
                out_lines.append(" ".join(parts))

            return "\n".join(out_lines)

        # Legacy clamping fallback when path metadata unavailable

        out_lines = []
        if start_pos is not None:
            start_x, start_y = start_pos
            if not (min_x <= start_x <= max_x and min_y <= start_y <= max_y):
                raise ValueError(
                    f"Start pos ({start_x:.1f},{start_y:.1f}) outside board bounds "
                    f"({min_x:.1f}-{max_x:.1f},{min_y:.1f}-{max_y:.1f})"
                )
            pos_x = start_x
            pos_y = start_y
        else:
            pos_x = 0.0
            pos_y = 0.0

        for raw in gcode_text.splitlines():
            line = raw.strip()

            if not line or line.startswith(';') or line.startswith('M'):
                out_lines.append(raw)
                continue

            if not (line.startswith('G0') or line.startswith('G1')):
                out_lines.append(raw)
                continue

            parts = line.split()
            dx = dy = 0.0

            for p in parts[1:]:
                if p.startswith('X'):
                    dx = float(p[1:])
                elif p.startswith('Y'):
                    dy = float(p[1:])

            next_x = pos_x + dx
            next_y = pos_y + dy

            clamped_x = min(max(next_x, min_x), max_x)
            clamped_y = min(max(next_y, min_y), max_y)

            if (next_x != clamped_x) or (next_y != clamped_y):
                print(
                    f"[GCODE] Clamping move from ({next_x:.1f},{next_y:.1f}) "
                    f"to ({clamped_x:.1f},{clamped_y:.1f}) "
                    f"safe=({min_x:.1f}-{max_x:.1f},{min_y:.1f}-{max_y:.1f})"
                )

            actual_dx = clamped_x - pos_x
            actual_dy = clamped_y - pos_y

            if abs(actual_dx) < 1e-3 and abs(actual_dy) < 1e-3:
                continue

            pos_x = clamped_x
            pos_y = clamped_y

            cmd = [parts[0]]
            if abs(actual_dx) >= 1e-3:
                cmd.append(f"X{actual_dx:.0f}")
            if abs(actual_dy) >= 1e-3:
                cmd.append(f"Y{actual_dy:.0f}")

            out_lines.append(" ".join(cmd))

        return "\n".join(out_lines)
