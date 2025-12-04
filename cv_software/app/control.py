from __future__ import annotations
from dataclasses import dataclass
from typing import Optional, List, Tuple, Dict
import math
import time
import os
from app.utils import board_to_robot


# ------------------------------------------------------------
# Data Models
# ------------------------------------------------------------

@dataclass(frozen=True)
class Waypoint:
    x_mm: float
    y_mm: float


@dataclass
class WaypointReached:
    waypoint_index: int
    target_x_mm: float
    target_y_mm: float
    actual_x_mm: float
    actual_y_mm: float
    confidence: float
    timestamp: float


@dataclass
class FollowerParams:
    pos_eps_mm: float = 2.0
    max_step_mm: float = 25.0
    max_step_forward_mm: float = 25.0
    max_step_left_mm: float = 12.0
    lookahead_distance_mm: float = 30.0  # Target point ahead on path
    max_position_jump_mm: float = 50.0   # Outlier rejection threshold
    min_segment_length_mm: float = 5.0   # Skip micro-segments


# ------------------------------------------------------------
# Path Follower
# ------------------------------------------------------------

class PathFollower:
    """
    Robot follows waypoints in board coordinates.
    Deviation = true lateral cross-track distance.
    """
    def __init__(self, params: Optional[FollowerParams] = None):
        self.p = params or FollowerParams()
        self._wps: List[Waypoint] = []
        self._i = 0
        self.active = False
        self.reached_history: List[WaypointReached] = []

        # EMA deviation smoothing
        self._smooth_deviation = 0.0
        self._deviation_alpha = float(os.getenv("DEVIATION_SMOOTHING", "0.3"))

        # Debounce for waypoint reached
        self._waypoint_stable_count = 0
        self._stable_threshold = int(os.getenv("WAYPOINT_DEBOUNCE_FRAMES", "3"))
        
        # Outlier rejection
        self._last_valid_pos: Optional[Tuple[float, float]] = None
        self._outlier_count = 0

    # ------------------------
    # Basic Controls
    # ------------------------

    def load_waypoints(self, wps: List[Waypoint], bounds: Optional[Tuple[float, float, float, float]] = None) -> None:
        self._wps = wps or []
        self._i = 0
        self.active = False
        self.reached_history = []
        self._smooth_deviation = 0.0
        self._waypoint_stable_count = 0
        self._last_valid_pos = None
        self._outlier_count = 0
        
        # Validate all waypoints are within bounds if provided
        if bounds and self._wps:
            min_x, min_y, max_x, max_y = bounds
            out_of_bounds = []
            for i, wp in enumerate(self._wps):
                if not (min_x <= wp.x_mm <= max_x and min_y <= wp.y_mm <= max_y):
                    out_of_bounds.append((i, wp.x_mm, wp.y_mm))
            
            if out_of_bounds:
                print(f"[FOLLOWER] WARNING: {len(out_of_bounds)}/{len(self._wps)} waypoints out of bounds:")
                for idx, x, y in out_of_bounds[:3]:  # Show first 3
                    print(f"  Waypoint {idx}: ({x:.1f},{y:.1f}) outside [{min_x:.1f}-{max_x:.1f}, {min_y:.1f}-{max_y:.1f}]")

    def start(self) -> None:
        self.active = bool(self._wps)
        self._i = 0
        self.reached_history = []
        self._smooth_deviation = 0.0
        self._waypoint_stable_count = 0
        self._last_valid_pos = None
        self._outlier_count = 0

    def stop(self) -> None:
        self.active = False

    def index(self) -> int:
        return self._i

    def total(self) -> int:
        return len(self._wps)

    def current_target(self) -> Optional[Waypoint]:
        if not self.active or self._i >= len(self._wps):
            return None
        return self._wps[self._i]

    # ------------------------
    # Path Geometry
    # ------------------------

    def _lateral_deviation(self, x: float, y: float, idx: int) -> float:
        """
        True signed cross-track deviation relative to segment idx→idx+1.
        Projects robot position onto segment and returns perpendicular distance.
        If past segment end, uses distance to next waypoint.
        """
        if idx >= len(self._wps) - 1:
            # Last waypoint → just distance
            wp = self._wps[idx]
            return math.hypot(x - wp.x_mm, y - wp.y_mm)

        wp1 = self._wps[idx]
        wp2 = self._wps[idx + 1]
        x1, y1 = wp1.x_mm, wp1.y_mm
        x2, y2 = wp2.x_mm, wp2.y_mm

        vx = x2 - x1
        vy = y2 - y1
        seg_len = math.hypot(vx, vy)
        
        # Skip micro-segments
        if seg_len < self.p.min_segment_length_mm:
            return math.hypot(x - x2, y - y2)

        # Normalized direction vector
        ux = vx / seg_len
        uy = vy / seg_len

        # Robot displacement from wp1
        rx = x - x1
        ry = y - y1

        # Project onto segment (along-track distance)
        along = rx * ux + ry * uy
        
        # Cross-track deviation (perpendicular distance)
        cross = abs(rx * uy - ry * ux)
        
        # If projection is past segment end, use distance to next waypoint
        if along > seg_len:
            return math.hypot(x - x2, y - y2)
        
        return cross

    # ------------------------
    # Lookahead Target
    # ------------------------
    
    def _get_lookahead_target(self, x: float, y: float) -> Optional[Waypoint]:
        """
        Find a target point on the path ahead of current position.
        Returns waypoint at approximately lookahead_distance_mm along path.
        """
        if not self.active or self._i >= len(self._wps):
            return None
        
        lookahead = self.p.lookahead_distance_mm
        accumulated_dist = 0.0
        
        # Start from current waypoint
        for j in range(self._i, len(self._wps)):
            wp = self._wps[j]
            dist_to_wp = math.hypot(wp.x_mm - x, wp.y_mm - y)
            
            if accumulated_dist + dist_to_wp >= lookahead:
                # Interpolate between current pos and this waypoint
                remaining = lookahead - accumulated_dist
                if dist_to_wp > 1e-6:
                    t = remaining / dist_to_wp
                    return Waypoint(
                        x_mm=x + t * (wp.x_mm - x),
                        y_mm=y + t * (wp.y_mm - y)
                    )
                return wp
            
            accumulated_dist += dist_to_wp
            if j + 1 < len(self._wps):
                next_wp = self._wps[j + 1]
                seg_len = math.hypot(next_wp.x_mm - wp.x_mm, next_wp.y_mm - wp.y_mm)
                accumulated_dist += seg_len
        
        # If path is shorter than lookahead, return last waypoint
        return self._wps[-1] if self._wps else None

    # ------------------------
    # Waypoint Advancement
    # ------------------------
    
    def _is_pose_valid(self, x: float, y: float) -> bool:
        """
        Check if pose is valid (not an outlier).
        Returns False if position jumped too far from last known good position.
        """
        if self._last_valid_pos is None:
            self._last_valid_pos = (x, y)
            return True
        
        lx, ly = self._last_valid_pos
        jump = math.hypot(x - lx, y - ly)
        
        if jump > self.p.max_position_jump_mm:
            self._outlier_count += 1
            if self._outlier_count >= 3:
                # Accept after 3 consistent outliers (robot may have actually moved)
                self._last_valid_pos = (x, y)
                self._outlier_count = 0
                return True
            return False
        
        # Valid pose - update tracking
        self._last_valid_pos = (x, y)
        self._outlier_count = 0
        return True

    def _advance_if_reached(self, x: float, y: float) -> None:
        """Advance index if robot is sufficiently close along segment direction."""
        if not self.active or self._i >= len(self._wps):
            return

        while self._i < len(self._wps):
            tgt = self._wps[self._i]
            dx = tgt.x_mm - x
            dy = tgt.y_mm - y
            dist = math.hypot(dx, dy)

            # Dynamic threshold based on segment length
            if self._i + 1 < len(self._wps):
                nxt = self._wps[self._i + 1]
                seg_len = math.hypot(nxt.x_mm - tgt.x_mm, nxt.y_mm - tgt.y_mm)
                # Larger segments allow earlier advancement
                dynamic_eps = min(self.p.pos_eps_mm * 2.0, seg_len * 0.2)
            else:
                dynamic_eps = self.p.pos_eps_mm

            # Directly on waypoint → reached
            if dist <= dynamic_eps:
                self._i += 1
                continue

            # Segment-based reach-ahead check with adaptive cross-track tolerance
            if self._i + 1 < len(self._wps):
                nxt = self._wps[self._i + 1]
                vx = nxt.x_mm - tgt.x_mm
                vy = nxt.y_mm - tgt.y_mm
                seg_len = math.hypot(vx, vy)

                if seg_len > self.p.min_segment_length_mm:
                    ux = vx / seg_len
                    uy = vy / seg_len
                    ex = x - tgt.x_mm
                    ey = y - tgt.y_mm
                    along = ex * ux + ey * uy
                    cross = abs(-ex * uy + ey * ux)
                    
                    # More lenient cross-track for longer segments
                    cross_threshold = min(seg_len * 0.15, 6.0 * self.p.pos_eps_mm)

                    if along > dynamic_eps and cross <= cross_threshold:
                        self._i += 1
                        continue

            break

    # ------------------------
    # Main Update
    # ------------------------

    def update_and_check_reached(
        self,
        pose: Tuple[Tuple[float, float], float, float],
    ) -> Tuple[bool, float]:
        """
        Update follower with pose.
        Returns: (reached_now, smoothed_lateral_deviation_mm)
        """
        if not self.active or not self._wps or self._i >= len(self._wps):
            return False, 0.0

        (x, y), heading, conf = pose
        
        # Outlier rejection
        if not self._is_pose_valid(x, y):
            # Skip this pose update but don't stop
            return False, self._smooth_deviation

        self._advance_if_reached(x, y)
        if self._i >= len(self._wps):
            self.stop()
            return True, 0.0

        raw_dev = self._lateral_deviation(x, y, self._i)

        a = self._deviation_alpha
        self._smooth_deviation = a * self._smooth_deviation + (1.0 - a) * raw_dev

        tgt = self._wps[self._i]
        dist_to_wp = math.hypot(tgt.x_mm - x, tgt.y_mm - y)
        
        # Dynamic arrival threshold
        if self._i + 1 < len(self._wps):
            nxt = self._wps[self._i + 1]
            seg_len = math.hypot(nxt.x_mm - tgt.x_mm, nxt.y_mm - tgt.y_mm)
            arrival_threshold = min(self.p.pos_eps_mm * 1.5, seg_len * 0.25)
        else:
            arrival_threshold = self.p.pos_eps_mm

        reached_now = False
        if dist_to_wp <= arrival_threshold:
            self._waypoint_stable_count += 1
            if self._waypoint_stable_count >= self._stable_threshold:
                self.reached_history.append(
                    WaypointReached(
                        waypoint_index=self._i,
                        target_x_mm=tgt.x_mm,
                        target_y_mm=tgt.y_mm,
                        actual_x_mm=x,
                        actual_y_mm=y,
                        confidence=conf,
                        timestamp=time.time(),
                    )
                )
                self._i += 1
                self._waypoint_stable_count = 0
                self._smooth_deviation = 0.0
                reached_now = True
        else:
            self._waypoint_stable_count = 0

        if self._i >= len(self._wps):
            self.stop()

        return reached_now, self._smooth_deviation

    # ------------------------
    # Step Generator
    # ------------------------

    @staticmethod
    def _to_robot_frame(dx_board: float, dy_board: float, heading_rad: float) -> Tuple[float, float]:
        return board_to_robot(dx_board, dy_board, heading_rad)

    def step(
        self,
        pose: Optional[Tuple[Tuple[float, float], float, float]],
        bounds: Optional[Tuple[float, float, float, float]] = None,
        margin: float = 0.0,
    ) -> Optional[Tuple[float, float]]:
        """
        Computes (forward_mm, left_mm) for firmware.
        Does not use lateral deviation.
        """
        if not self.active or not self._wps or pose is None:
            return None

        (x, y), heading, _ = pose

        self._advance_if_reached(x, y)
        if self._i >= len(self._wps):
            self.stop()
            return None

        tgt = self._wps[self._i]
        ex = tgt.x_mm - x
        ey = tgt.y_mm - y

        dist = math.hypot(ex, ey)
        if dist <= self.p.pos_eps_mm:
            self._advance_if_reached(x, y)
            return None

        # forward step toward target
        step_len = min(self.p.max_step_mm, dist)
        ux = ex / dist
        uy = ey / dist
        dx_board = ux * step_len
        dy_board = uy * step_len

        # board bounds
        if bounds is not None:
            min_x, min_y, max_x, max_y = bounds
            nx = max(min_x + margin, min(x + dx_board, max_x - margin))
            ny = max(min_y + margin, min(y + dy_board, max_y - margin))
            dx_board = nx - x
            dy_board = ny - y

        # convert board → robot frame
        fwd, left = self._to_robot_frame(dx_board, dy_board, heading)
        fwd = max(-self.p.max_step_forward_mm, min(self.p.max_step_forward_mm, fwd))
        left = max(-self.p.max_step_left_mm, min(self.p.max_step_left_mm, left))
        return (fwd, left)


# ------------------------------------------------------------
# Simple manager
# ------------------------------------------------------------

class PathRun:
    def __init__(self):
        self.wps: List[Waypoint] = []
        self.follower = PathFollower(FollowerParams())
        self.active = False
        self.path_name: Optional[str] = None

    def load(self, wps: List[Waypoint], path_name: Optional[str] = None, bounds: Optional[Tuple[float, float, float, float]] = None) -> None:
        self.wps = wps or []
        self.path_name = path_name
        self.follower.load_waypoints(self.wps, bounds=bounds)

    def start(self) -> None:
        if self.wps:
            self.active = True
            self.follower.start()

    def stop(self) -> None:
        self.active = False
        self.follower.stop()

    def status(self) -> Dict[str, int | bool]:
        return {
            "active": self.active,
            "idx": self.follower.index(),
            "total": self.follower.total(),
        }
