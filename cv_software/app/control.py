from __future__ import annotations
from dataclasses import dataclass
from typing import Optional, List, Tuple, Dict
import math
import time
import os
from app.utils import board_to_robot

@dataclass(frozen=True)
class Waypoint:
    x_mm: float
    y_mm: float

@dataclass
class WaypointReached:
    """Data captured when robot reaches a waypoint (for learning)."""
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
    # legacy magnitude clamp
    max_step_mm: float = 25.0
    max_step_forward_mm: float = 25.0
    max_step_left_mm: float = 12.0   # give Y/left less if you want tighter control

class PathFollower:
    """
    Minimal path follower: computes a bounded step toward the next waypoint.
    Returns a (forward,left) delta in the ROBOT frame, leaving timing/motion to firmware.
    Also tracks waypoint arrivals for learning corrections.
    """
    def __init__(self, params: Optional[FollowerParams] = None):
        self.p = params or FollowerParams()
        self._wps: List[Waypoint] = []
        self._i = 0
        self.active = False
        self.reached_history: List[WaypointReached] = []  # Learning data
        
        # Smoothing/debouncing state (configurable via environment)
        self._smooth_deviation = 0.0
        self._deviation_alpha = float(os.getenv("DEVIATION_SMOOTHING", "0.3"))
        self._waypoint_stable_count = 0
        self._stable_threshold = int(os.getenv("WAYPOINT_DEBOUNCE_FRAMES", "3"))

    def load_waypoints(self, wps: List[Waypoint]) -> None:
        self._wps = wps or []
        self._i = 0
        self.reached_history = []  # Reset learning data

    def start(self) -> None:
        self.active = bool(self._wps)
        self._i = 0
        self.reached_history = []
        self._smooth_deviation = 0.0
        self._waypoint_stable_count = 0

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
    
    def update_and_check_reached(
        self, 
        pose: Tuple[Tuple[float, float], float, float]
    ) -> Tuple[bool, float]:
        """
        Update tracking with current pose. Returns (reached, smooth_deviation).
        Call this every frame with current robot pose.
        
        Returns:
            reached: True if waypoint was just reached (and advanced)
            smooth_deviation: Smoothed distance to target in mm
        """
        if not self.active or self._i >= len(self._wps):
            return False, 0.0
            
        (x, y), heading, conf = pose
        target_wp = self._wps[self._i]
        
        # Calculate distance
        dx = target_wp.x_mm - x
        dy = target_wp.y_mm - y
        raw_distance = math.sqrt(dx**2 + dy**2)
        
        # Smooth deviation
        self._smooth_deviation = (
            self._deviation_alpha * self._smooth_deviation + 
            (1.0 - self._deviation_alpha) * raw_distance
        )
        
        # Debounced waypoint detection
        if raw_distance <= self.p.pos_eps_mm:
            self._waypoint_stable_count += 1
            
            if self._waypoint_stable_count >= self._stable_threshold:
                # Record for learning before advancing
                self.reached_history.append(WaypointReached(
                    waypoint_index=self._i,
                    target_x_mm=target_wp.x_mm,
                    target_y_mm=target_wp.y_mm,
                    actual_x_mm=x,
                    actual_y_mm=y,
                    confidence=conf,
                    timestamp=time.time()
                ))
                
                # Advance waypoint
                old_idx = self._i
                self._advance_if_reached(pose)
                
                # Reset smoothing on waypoint change
                if self._i != old_idx:
                    self._waypoint_stable_count = 0
                    self._smooth_deviation = 0.0
                    return True, 0.0  # Waypoint reached!
        else:
            self._waypoint_stable_count = 0
            
        return False, self._smooth_deviation

    def _advance_if_reached(self, pose: Tuple[Tuple[float, float], float, float]) -> None:
        if not self.active or self._i >= len(self._wps):
            return
        (x, y), _, _ = pose
        while self._i < len(self._wps):
            tgt = self._wps[self._i]
            ex, ey = (tgt.x_mm - x), (tgt.y_mm - y)
            dist = math.hypot(ex, ey)

            if dist <= self.p.pos_eps_mm:
                self._i += 1
                continue
            # If there's a "next" waypoint, project onto the current segment
            if self._i + 1 < len(self._wps):
                nxt = self._wps[self._i + 1]
                vx, vy = (nxt.x_mm - tgt.x_mm), (nxt.y_mm - tgt.y_mm)
                seg_len = math.hypot(vx, vy) or 1.0
                ux, uy = (vx / seg_len), (vy / seg_len)
                along = ex * ux + ey * uy     # signed along-track error
                cross = abs(-ex * uy + ey * ux)  # cross-track magnitude
                if along <= self.p.pos_eps_mm and cross <= 5.0 * self.p.pos_eps_mm:
                    self._i += 1
                    continue
            break

    def current_error(self, pose: Tuple[Tuple[float, float], float, float]) -> Optional[Tuple[float, float, Waypoint]]:
        if not self.active or self._i >= len(self._wps):
            return None
        (x, y), _, _ = pose
        tgt = self._wps[self._i]
        return (tgt.x_mm - x, tgt.y_mm - y, tgt)

    @staticmethod
    def _to_robot_frame(dx_board: float, dy_board: float, heading_rad: float) -> Tuple[float, float]:
        return board_to_robot(dx_board, dy_board, heading_rad)

    def step(self, pose: Optional[Tuple[Tuple[float, float], float, float]], bounds: Optional[Tuple[float, float, float, float]] = None, margin: float = 0.0) -> Optional[Tuple[float, float]]:
        """
        pose: ((x_mm, y_mm), heading_rad, confidence)
        bounds: (min_x, min_y, max_x, max_y) in mm, optional
        margin: safety margin in mm to stay away from bounds edges
        returns: (forward_mm, left_mm) in ROBOT frame, or None when finished/not ready
        """
        if not self.active or not self._wps or pose is None:
            return None

        self._advance_if_reached(pose)
        if self._i >= len(self._wps):
            self.stop()
            return None

        err = self.current_error(pose)
        if err is None:
            return None
        ex, ey, _tgt = err

        dist = math.hypot(ex, ey)
        if dist <= self.p.pos_eps_mm:
            self._advance_if_reached(pose)
            if self._i >= len(self._wps):
                self.stop()
                return None
            return None

        step_len = min(self.p.max_step_mm, dist)
        ux, uy = ex / dist, ey / dist
        (x, y), heading, _ = pose

        dx_board = ux * step_len
        dy_board = uy * step_len

        if bounds is not None:
            min_x, min_y, max_x, max_y = bounds
            next_x = x + dx_board
            next_y = y + dy_board
            next_x = max(min_x + margin, min(next_x, max_x - margin))
            next_y = max(min_y + margin, min(next_y, max_y - margin))
            dx_board = next_x - x
            dy_board = next_y - y

        fwd, left = self._to_robot_frame(dx_board, dy_board, heading)

        fwd = max(-self.p.max_step_forward_mm, min(self.p.max_step_forward_mm, fwd))
        left = max(-self.p.max_step_left_mm,   min(self.p.max_step_left_mm,   left))

        return (fwd, left)



class PathRun:
    """Thin wrapper that owns a follower and active flag."""
    def __init__(self):
        self.wps: List[Waypoint] = []
        self.follower = PathFollower(FollowerParams())
        self.active = False
        self.path_name: Optional[str] = None  # For learning

    def load(self, wps: List[Waypoint], path_name: Optional[str] = None) -> None:
        self.wps = wps or []
        self.path_name = path_name
        self.follower.load_waypoints(self.wps)

    def start(self) -> None:
        if self.wps:
            self.active = True
            self.follower.start()

    def stop(self) -> None:
        self.active = False
        self.follower.stop()
        
    def save_learning_data(self) -> None:
        """Save waypoint arrival data for learning corrections."""
        if not self.path_name or not self.follower.reached_history:
            return
            
        from app.path_corrections import PathCorrectionLearner
        
        learner = PathCorrectionLearner()
        learner.current_path_name = self.path_name
        learner.recording = True
        
        for reached in self.follower.reached_history:
            learner.record_deviation(
                waypoint_index=reached.waypoint_index,
                target_mm=(reached.target_x_mm, reached.target_y_mm),
                actual_mm=(reached.actual_x_mm, reached.actual_y_mm),
                confidence=reached.confidence,
                timestamp=reached.timestamp
            )
        
        learner.stop_recording_and_save()

    def status(self) -> Dict[str, int | bool]:
        return {"active": self.active, "idx": self.follower.index(), "total": self.follower.total()}
