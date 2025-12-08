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
    max_position_jump_mm: float = 150.0  # Outlier rejection threshold (increased for initial alignment)
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
        
        # Error tracking
        self._consecutive_errors = 0
        self._max_consecutive_errors = 10

    # ------------------------
    # Basic Controls
    # ------------------------

    def _validate_waypoint(self, wp: Waypoint, index: int) -> bool:
        """Validate a single waypoint for NaN, infinite, and reasonable values."""
        import math
        x, y = wp.x_mm, wp.y_mm
        
        # Check for NaN or infinite
        if not (math.isfinite(x) and math.isfinite(y)):
            print(f"[FOLLOWER] ERROR: Waypoint {index} has invalid coordinates: ({x}, {y})")
            return False
        
        # Check for reasonable values (not too large/small)
        MAX_REASONABLE_MM = 10000.0  # 10 meters
        if abs(x) > MAX_REASONABLE_MM or abs(y) > MAX_REASONABLE_MM:
            print(f"[FOLLOWER] ERROR: Waypoint {index} has unreasonably large coordinates: ({x:.1f}, {y:.1f})")
            return False
        
        return True
    
    def _deduplicate_waypoints(self, wps: List[Waypoint], min_distance_mm: float = 0.5) -> List[Waypoint]:
        """Remove duplicate or very close waypoints."""
        if not wps:
            return []
        
        deduped = [wps[0]]  # Always keep first waypoint
        
        for wp in wps[1:]:
            last = deduped[-1]
            dist = math.hypot(wp.x_mm - last.x_mm, wp.y_mm - last.y_mm)
            if dist >= min_distance_mm:
                deduped.append(wp)
            # Otherwise skip this waypoint (too close to previous)
        
        if len(deduped) < len(wps):
            print(f"[FOLLOWER] Deduplicated waypoints: {len(wps)} → {len(deduped)} (removed {len(wps) - len(deduped)} duplicates)")
        
        return deduped
    
    def _simplify_path(self, wps: List[Waypoint], angle_threshold_deg: float = 5.0, min_segment_mm: float = 2.0) -> List[Waypoint]:
        """Simplify path by removing collinear waypoints while preserving path shape.
        
        Uses Ramer-Douglas-Peucker-like approach:
        - Remove waypoints that are nearly collinear with neighbors
        - Keep waypoints that represent significant direction changes
        - Always preserve first and last waypoints
        """
        if len(wps) <= 2:
            return wps
        
        simplified = [wps[0]]  # Always keep first
        
        for i in range(1, len(wps) - 1):
            prev = simplified[-1]
            curr = wps[i]
            next_wp = wps[i + 1]
            
            # Vector from prev to curr
            v1x = curr.x_mm - prev.x_mm
            v1y = curr.y_mm - prev.y_mm
            len1 = math.hypot(v1x, v1y)
            
            # Vector from curr to next
            v2x = next_wp.x_mm - curr.x_mm
            v2y = next_wp.y_mm - curr.y_mm
            len2 = math.hypot(v2x, v2y)
            
            # Skip if segments are too short
            if len1 < min_segment_mm or len2 < min_segment_mm:
                simplified.append(curr)
                continue
            
            # Compute angle between vectors
            # cos(theta) = (v1 · v2) / (|v1| * |v2|)
            dot = v1x * v2x + v1y * v2y
            cos_angle = dot / (len1 * len2)
            cos_angle = max(-1.0, min(1.0, cos_angle))  # Clamp for numerical stability
            angle_deg = math.degrees(math.acos(cos_angle))
            
            # Keep waypoint if angle is significant (direction change)
            if angle_deg >= angle_threshold_deg:
                simplified.append(curr)
            # Otherwise skip this waypoint (nearly collinear)
        
        simplified.append(wps[-1])  # Always keep last
        
        if len(simplified) < len(wps):
            print(f"[FOLLOWER] Simplified path: {len(wps)} → {len(simplified)} waypoints (removed {len(wps) - len(simplified)} collinear points)")
        
        return simplified

    def load_waypoints(self, wps: List[Waypoint], bounds: Optional[Tuple[float, float, float, float]] = None) -> None:
        """Load and validate waypoints."""
        if not wps:
            print("[FOLLOWER] WARNING: Empty waypoint list provided")
            self._wps = []
            self._i = 0
            self.active = False
            return
        
        # Validate all waypoints
        valid_wps = []
        invalid_count = 0
        for i, wp in enumerate(wps):
            if self._validate_waypoint(wp, i):
                valid_wps.append(wp)
            else:
                invalid_count += 1
        
        if invalid_count > 0:
            print(f"[FOLLOWER] WARNING: {invalid_count}/{len(wps)} waypoints were invalid and removed")
        
        if not valid_wps:
            print("[FOLLOWER] ERROR: No valid waypoints after validation")
            self._wps = []
            self._i = 0
            self.active = False
            return
        
        # Deduplicate waypoints
        valid_wps = self._deduplicate_waypoints(valid_wps)
        
        # Simplify path by removing collinear waypoints (improves efficiency)
        # More aggressive simplification for long paths
        if len(valid_wps) > 50:
            valid_wps = self._simplify_path(valid_wps, angle_threshold_deg=3.0, min_segment_mm=1.5)
        elif len(valid_wps) > 20:
            valid_wps = self._simplify_path(valid_wps, angle_threshold_deg=5.0, min_segment_mm=2.0)
        # Keep very short paths as-is for precision
        
        # Validate waypoints are within bounds if provided (skip first waypoint - it's the bot's actual position)
        if bounds and len(valid_wps) > 1:
            min_x, min_y, max_x, max_y = bounds
            out_of_bounds = []
            clamped_wps = [valid_wps[0]]  # Keep first waypoint as-is
            
            for i, wp in enumerate(valid_wps[1:], start=1):  # Skip waypoint 0
                if not (min_x <= wp.x_mm <= max_x and min_y <= wp.y_mm <= max_y):
                    out_of_bounds.append((i, wp.x_mm, wp.y_mm))
                    # Clamp to bounds instead of rejecting
                    clamped_x = max(min_x, min(wp.x_mm, max_x))
                    clamped_y = max(min_y, min(wp.y_mm, max_y))
                    clamped_wps.append(Waypoint(x_mm=clamped_x, y_mm=clamped_y))
                else:
                    clamped_wps.append(wp)
            
            if out_of_bounds:
                print(f"[FOLLOWER] WARNING: {len(out_of_bounds)}/{len(valid_wps)-1} waypoints out of bounds (clamped):")
                for idx, x, y in out_of_bounds[:3]:  # Show first 3
                    print(f"  Waypoint {idx}: ({x:.1f},{y:.1f}) clamped to bounds")
                valid_wps = clamped_wps
        
        self._wps = valid_wps
        self._i = 0
        self.active = False
        self.reached_history = []
        self._smooth_deviation = 0.0
        self._waypoint_stable_count = 0
        self._last_valid_pos = None
        self._outlier_count = 0
        self._consecutive_errors = 0
        
        print(f"[FOLLOWER] Loaded {len(self._wps)} valid waypoints")

    def start(self) -> None:
        self.active = bool(self._wps)
        self._i = 0
        self.reached_history = []
        self._smooth_deviation = 0.0
        self._waypoint_stable_count = 0
        self._last_valid_pos = None
        self._outlier_count = 0
        self._consecutive_errors = 0

    def stop(self) -> None:
        self.active = False

    def index(self) -> int:
        return self._i

    def total(self) -> int:
        return len(self._wps)

    def current_target(self) -> Optional[Waypoint]:
        """Get current target waypoint with bounds checking."""
        if not self.active or not self._wps:
            return None
        if self._i < 0 or self._i >= len(self._wps):
            # Index out of bounds - clamp it
            if self._i < 0:
                self._i = 0
            elif self._i >= len(self._wps):
                self._i = max(0, len(self._wps) - 1)
            print(f"[FOLLOWER] WARNING: Index {self._i} out of bounds, clamped to valid range")
        return self._wps[self._i]
    
    def get_waypoint(self, index: int) -> Optional[Waypoint]:
        """Safely get waypoint at index with bounds checking."""
        if not self._wps or index < 0 or index >= len(self._wps):
            return None
        return self._wps[index]
    
    def is_index_valid(self, index: int) -> bool:
        """Check if waypoint index is valid."""
        return 0 <= index < len(self._wps) if self._wps else False

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
        if not self.active or not self._wps:
            return
        
        # Ensure index is valid
        if self._i < 0:
            self._i = 0
        if self._i >= len(self._wps):
            return

        max_advances = len(self._wps)  # Prevent infinite loops
        advances = 0
        
        while self._i < len(self._wps) and advances < max_advances:
            advances += 1
            tgt = self._wps[self._i]
            dx = tgt.x_mm - x
            dy = tgt.y_mm - y
            dist = math.hypot(dx, dy)

            # Dynamic threshold based on segment length - more lenient
            if self._i + 1 < len(self._wps):
                nxt = self._wps[self._i + 1]
                seg_len = math.hypot(nxt.x_mm - tgt.x_mm, nxt.y_mm - tgt.y_mm)
                # Larger segments allow earlier advancement - increased to 30% of segment
                dynamic_eps = min(self.p.pos_eps_mm * 3.0, seg_len * 0.3, 50.0)  # Cap at 50mm
            else:
                dynamic_eps = self.p.pos_eps_mm * 3.0  # More lenient for last waypoint

            # Directly on waypoint → reached
            if dist <= dynamic_eps:
                print(f"[FOLLOWER] Reached waypoint {self._i} (dist={dist:.1f}mm <= {dynamic_eps:.1f}mm)")
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
                    
                    # More lenient cross-track for longer segments - increased to 25% of segment
                    cross_threshold = min(seg_len * 0.25, 8.0 * self.p.pos_eps_mm, 60.0)  # Cap at 60mm
                    
                    # Advance if we've passed the waypoint along the path (along > 0) and cross-track is reasonable
                    # OR if we're more than 50% along the segment
                    # DISABLE for first 10 waypoints to prevent startup skipping due to anchor mismatch
                    if self._i >= 10 and ((along > dynamic_eps and cross <= cross_threshold) or (along > seg_len * 0.5)):
                        print(f"[FOLLOWER] Advanced past waypoint {self._i} (along={along:.1f}mm/{seg_len:.1f}mm, cross={cross:.1f}mm)")
                        self._i += 1
                        continue
            else:
                # Last waypoint - be more lenient if we're close
                if dist <= self.p.pos_eps_mm * 5.0:  # 5x threshold for last waypoint
                    print(f"[FOLLOWER] Reached last waypoint (dist={dist:.1f}mm)")
                    self._i += 1
                    continue

            # Special case: if waypoint is very far away AND we've already made progress, check if we're closer to next waypoint
            # Don't skip early waypoints (first 5) - they represent the actual drawing path from the start position
            if dist > 150.0 and self._i >= 5 and len(self._wps) > self._i + 1:
                nxt = self._wps[self._i + 1]
                dist_to_next = math.hypot(nxt.x_mm - x, nxt.y_mm - y)
                if dist_to_next < dist * 0.7:  # At least 30% closer to next waypoint
                    print(f"[FOLLOWER] Skipping waypoint {self._i} (dist={dist:.1f}mm) - closer to waypoint {self._i+1} (dist={dist_to_next:.1f}mm)")
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
        if not self.active or not self._wps:
            return False, 0.0
        
        # Ensure index is valid
        if self._i < 0:
            self._i = 0
        if self._i >= len(self._wps):
            self.stop()
            return False, 0.0

        (x, y), heading, conf = pose
        
        # Validate pose coordinates
        import math
        if not (math.isfinite(x) and math.isfinite(y)):
            self._consecutive_errors += 1
            if self._consecutive_errors <= 3:  # Only log first few
                print(f"[FOLLOWER] WARNING: Invalid pose coordinates: ({x}, {y}), skipping update")
            if self._consecutive_errors >= self._max_consecutive_errors:
                print(f"[FOLLOWER] ERROR: Too many consecutive errors ({self._consecutive_errors}), stopping")
                self.stop()
            return False, self._smooth_deviation
        
        # Reset error counter on valid pose
        self._consecutive_errors = 0
        
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
        # More lenient arrival threshold - also check if we've progressed along the path
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
                print(f"[FOLLOWER] Confirmed waypoint {self._i} reached (dist={dist_to_wp:.1f}mm)")
                self._i += 1
                self._waypoint_stable_count = 0
                self._smooth_deviation = 0.0
                reached_now = True
        else:
            # Also check if we've progressed significantly along the path segment
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
                    # If we're more than 60% along the segment, advance
                    if along > seg_len * 0.6:
                        print(f"[FOLLOWER] Advanced waypoint {self._i} (progress={along:.1f}mm/{seg_len:.1f}mm = {along/seg_len*100:.0f}%)")
                        self._i += 1
                        self._waypoint_stable_count = 0
                        reached_now = True
                    else:
                        self._waypoint_stable_count = 0
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
        self._load_time: Optional[float] = None

    def load(self, wps: List[Waypoint], path_name: Optional[str] = None, bounds: Optional[Tuple[float, float, float, float]] = None) -> bool:
        """
        Load waypoints into the path follower.
        Returns True if successful, False otherwise.
        """
        import time
        try:
            if not wps:
                print("[PATH_RUN] ERROR: Cannot load empty waypoint list")
                self.wps = []
                self.follower.load_waypoints([], bounds=bounds)
                return False
            
            self.wps = wps
            self.path_name = path_name
            self.follower.load_waypoints(self.wps, bounds=bounds)
            self._load_time = time.time()
            
            # Verify load was successful
            if self.follower.total() == 0:
                print("[PATH_RUN] ERROR: Waypoint load failed - no valid waypoints")
                return False
            
            print(f"[PATH_RUN] Successfully loaded {self.follower.total()} waypoints" + 
                  (f" (path: {path_name})" if path_name else ""))
            return True
        except Exception as e:
            print(f"[PATH_RUN] ERROR: Failed to load waypoints: {e}")
            import traceback
            traceback.print_exc()
            self.wps = []
            self.follower.load_waypoints([], bounds=bounds)
            return False

    def start(self) -> bool:
        """Start path following. Returns True if successful."""
        if not self.wps or self.follower.total() == 0:
            print("[PATH_RUN] ERROR: Cannot start - no waypoints loaded")
            return False
        
        try:
            self.active = True
            self.follower.start()
            if not self.follower.active:
                print("[PATH_RUN] ERROR: Follower failed to start")
                self.active = False
                return False
            print(f"[PATH_RUN] Started following path with {self.follower.total()} waypoints")
            return True
        except Exception as e:
            print(f"[PATH_RUN] ERROR: Exception during start: {e}")
            self.active = False
            return False

    def stop(self) -> None:
        """Stop path following."""
        self.active = False
        self.follower.stop()

    def status(self) -> Dict[str, int | bool | str]:
        """Get current status of path run."""
        idx = self.follower.index()
        total = self.follower.total()
        return {
            "active": self.active,
            "idx": idx,
            "total": total,
            "path_name": self.path_name or "unknown",
            "progress_pct": (idx / total * 100.0) if total > 0 else 0.0,
            "has_waypoints": total > 0,
        }
    
    def is_ready(self) -> bool:
        """Check if path run is ready (has waypoints and is not active)."""
        return bool(self.wps and self.follower.total() > 0 and not self.active)
