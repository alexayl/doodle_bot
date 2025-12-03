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
BOARD_MARGIN_MM = float(os.getenv("BOARD_MARGIN_MM", "67"))
BOARD_MARGIN_FRAC = float(os.getenv("BOARD_MARGIN_FRAC", "0.20"))
_POSE_ALPHA = 0.7
_CONF_MIN_BOARD = 0.60
_CONF_MIN_BOT = 0.30


CV_CORRECTION_RATE_HZ = float(os.getenv("CV_CORRECTION_RATE_HZ", "7.0"))
CV_MIN_ERROR_MM = float(os.getenv("CV_MIN_ERROR_MM", "2.5"))
CV_LOOKAHEAD_COUNT = int(os.getenv("CV_LOOKAHEAD_COUNT", "2"))
CV_CROSS_TRACK_GAIN = float(os.getenv("CV_CROSS_TRACK_GAIN", "2.0"))
CV_ADAPTIVE_STEP = bool(int(os.getenv("CV_ADAPTIVE_STEP", "1")))
CV_MAX_CORRECTION_MM = float(os.getenv("CV_MAX_CORRECTION_MM", "8.0"))
CV_MAX_POSE_AGE_S = float(os.getenv("CV_MAX_POSE_AGE_S", "1.0"))

BOARD_CORNERS = {"TL": 0, "TR": 1, "BR": 2, "BL": 3}
BOT_MARKERS = (10, 11)

MARKER_DICT = os.getenv("MARKER_DICT", "4x4_100").upper()

BOT_BASELINE_MM = 100.0

KNOWN_BOARD_WIDTH_MM = float(800)
KNOWN_BOARD_HEIGHT_MM = float(400)

_MAX_REPROJ_ERR_PX = 1.2
_MAX_SIZE_JUMP_FRAC = 0.15
_MAX_CENTER_JUMP_PX = 40.0

_REPROJ_GOOD_PX = 0.5
_COND_HARD_LIMIT = 5e7
_COND_WARN_LIMIT = 3e4

_HEADING_MAX_DEG = float(os.getenv("HEADING_MAX_DEG", "10.0"))
_HEADING_MIN_SAMPLES = int(os.getenv("HEADING_MIN_SAMPLES", "5"))
_HEADING_HISTORY_SIZE = int(os.getenv("HEADING_HISTORY_SIZE", "30"))


def correct_delta_mm(delta_board_mm: tuple[float, float], heading_rad: float) -> tuple[float, float]:
    dx = float(delta_board_mm[0])
    dy = float(delta_board_mm[1])
    c = math.cos(float(heading_rad))
    s = math.sin(float(heading_rad))
    fwd = dx * c + dy * s
    left = -dx * s + dy * c
    return fwd, left


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

    def calibrate(self, frame_bgr, force_recalibrate: bool = True) -> Optional[BoardPose]:
        self.pose_fresh = False
        
        if self.calibration_locked and self.board_pose is not None and self.board_pose.confidence > 0.6:
            return self.board_pose

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

        H, mask = cv2.findHomography(img_pts, board_rect, cv2.RANSAC, 3.0)
        if H is None:
            return prev
        if abs(H[2, 2]) > 1e-12:
            H = H / H[2, 2]

        reproj = self._reproj_error(H, img_pts, board_rect)

        width_px = float(np.linalg.norm(tr - tl))
        height_px = float(np.linalg.norm(bl - tl))
        cond_H = np.linalg.cond(H)

        if conf_raw < _CONF_MIN_BOARD:
            if not hasattr(self, "_low_conf_log") or time.time() - self._low_conf_log > 2.0:
                print(
                    f"[BOARD_CAL] Low confidence: found {found}/4 corners, "
                    f"conf_raw={conf_raw:.2f}, reproj={reproj:.3f}px, cond={cond_H:.1e}"
                )
                print("[BOARD_CAL] Marking calibration as unstable until markers recover.")
                self._low_conf_log = time.time()

        if reproj > _REPROJ_GOOD_PX:
            if not hasattr(self, "_reproj_log") or time.time() - self._reproj_log > 2.5:
                # print(
                #     f"[BOARD_CAL] High reprojection error: {reproj:.3f}px "
                #     f"(threshold={_REPROJ_GOOD_PX})"
                # )
                self._reproj_log = time.time()

        if cond_H > _COND_WARN_LIMIT:
            if not hasattr(self, "_cond_log") or time.time() - self._cond_log > 3.0:
                # print(
                #     f"[BOARD_CAL] Homography condition number high: {cond_H:.1e}, "
                #     f"warn_limit={_COND_WARN_LIMIT}, hard_limit={_COND_HARD_LIMIT}"
                # )
                self._cond_log = time.time()

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

        # try:
        #     img_corner_log = (
        #         f"[BOARD_CAL] Image corners (px): "
        #         f"TL=({tl[0]:.1f},{tl[1]:.1f}), "
        #         f"TR=({tr[0]:.1f},{tr[1]:.1f}), "
        #         f"BR=({br[0]:.1f},{br[1]:.1f}), "
        #         f"BL=({bl[0]:.1f},{bl[1]:.1f})"
        #     )
        #     board_corner_log = (
        #         f"[BOARD_CAL] Board corners (mm): "
        #         f"TL=(0.0,{KNOWN_BOARD_HEIGHT_MM:.1f}), "
        #         f"TR=({KNOWN_BOARD_WIDTH_MM:.1f},{KNOWN_BOARD_HEIGHT_MM:.1f}), "
        #         f"BR=({KNOWN_BOARD_WIDTH_MM:.1f},0.0), "
        #         f"BL=(0.0,0.0)"
        #     )
        #     print(img_corner_log)
        #     print(board_corner_log)
        # except Exception:
        #     pass

        return self.board_pose



class BotTracker:
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
        # if has_marker_0 and has_marker_1:
        #     print(f"BOT_TRACK: BOT located at {pose}")
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
    def compute_correction(
        self,
        waypoints: list,
        current_index: int,
        next_is_g1: bool = True,
    ) -> Optional[Tuple[float, float]]:
        """Simple correction: nudge robot toward current target waypoint."""

        if not next_is_g1:
            return None

        if current_index >= len(waypoints):
            return None

        pose = self._pose()
        if pose is None:
            return None
        cx, cy, _, conf = pose
        pose_age = self.cvp.pose_age_s()
        if pose_age is not None and pose_age > self.max_pose_age_s:
            now = time.time()
            if now - self._last_stale_log > 1.5:
                print(
                    f"[CORRECTION] skipping: pose stale ({pose_age:.2f}s > {self.max_pose_age_s:.2f}s)"
                )
                self._last_stale_log = now
            return None

        # # low-confidence pose
        # if conf < 0.60:
        #     return None

        # Get current target waypoint
        try:
            tx, ty = self._wp_xy(waypoints[current_index])
        except Exception:
            return None

        # Compute error from current position to target
        error_x = tx - cx
        error_y = ty - cy
        error_mag = math.hypot(error_x, error_y)
        
        # If we're close enough, don't correct
        if error_mag < self.min_error_mm:
            return None

        # Apply proportional correction (gentle nudge)
        corr_gain = 0.3  # 30% of error per correction
        dx_board = error_x * corr_gain
        dy_board = error_y * corr_gain

        # Clamp to max correction
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

        print(f"[CORRECTION] error=({error_x:.1f},{error_y:.1f})mm mag={error_mag:.1f}mm → correction=({dx_board:.1f},{dy_board:.1f})mm")
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
        """
        if not wps:
            return []

        rect = self._safe_board_rect(margin_frac)
        if rect is None:
            print("PATH FITTING REJECTED: No usable board bounds available.")
            return []

        min_x2, min_y2, span_x, span_y = rect

        out: list[Waypoint] = []
        for p in wps:
            # p.x_mm and p.y_mm are actually normalized [0,1] here
            nx = float(p.x_mm)
            ny = float(p.y_mm)

            x_mm = min_x2 + nx * span_x
            y_mm = min_y2 + ny * span_y

            out.append(Waypoint(x_mm=x_mm, y_mm=y_mm))

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
