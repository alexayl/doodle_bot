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

    # ------------------------
    # Basic Controls
    # ------------------------

    def load_waypoints(self, wps: List[Waypoint]) -> None:
        self._wps = wps or []
        self._i = 0
        self.active = False
        self.reached_history = []
        self._smooth_deviation = 0.0
        self._waypoint_stable_count = 0

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

    # ------------------------
    # Path Geometry
    # ------------------------

    def _lateral_deviation(self, x: float, y: float, idx: int) -> float:
        """
        True signed cross-track deviation relative to segment idx→idx+1.
        Uses standard 2D cross product geometry.
        If last segment: return point distance.
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
        if seg_len < 1e-6:
            return math.hypot(x - x1, y - y1)

        # normalized direction
        nx = vx / seg_len
        ny = vy / seg_len

        # robot displacement from wp1
        rx = x - x1
        ry = y - y1

        # cross-track deviation (signed)
        cross = rx * ny - ry * nx
        return abs(cross)

    # ------------------------
    # Waypoint Advancement
    # ------------------------

    def _advance_if_reached(self, x: float, y: float) -> None:
        """Advance index if robot is sufficiently close along segment direction."""
        if not self.active or self._i >= len(self._wps):
            return

        while self._i < len(self._wps):
            tgt = self._wps[self._i]
            dx = tgt.x_mm - x
            dy = tgt.y_mm - y
            dist = math.hypot(dx, dy)

            # Too close → reached
            if dist <= self.p.pos_eps_mm:
                self._i += 1
                continue

            # Segment-based reach-ahead check
            if self._i + 1 < len(self._wps):
                nxt = self._wps[self._i + 1]
                vx = nxt.x_mm - tgt.x_mm
                vy = nxt.y_mm - tgt.y_mm
                seg_len = math.hypot(vx, vy)

                if seg_len > 1e-6:
                    ux = vx / seg_len
                    uy = vy / seg_len
                    ex = x - tgt.x_mm
                    ey = y - tgt.y_mm
                    along = ex * ux + ey * uy
                    cross = abs(-ex * uy + ey * ux)

                    if along > 0 and cross <= 4.0 * self.p.pos_eps_mm:
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

        print("USING LATERAL DEV")

        (x, y), heading, conf = pose

        # Advance through already-reached waypoints
        self._advance_if_reached(x, y)
        if self._i >= len(self._wps):
            self.stop()
            return True, 0.0

        # Compute lateral deviation
        raw_dev = self._lateral_deviation(x, y, self._i)

        # Smooth deviation
        a = self._deviation_alpha
        self._smooth_deviation = a * self._smooth_deviation + (1.0 - a) * raw_dev

        # Reached check
        tgt = self._wps[self._i]
        dist_to_wp = math.hypot(tgt.x_mm - x, tgt.y_mm - y)

        reached_now = False
        if dist_to_wp <= self.p.pos_eps_mm:
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

    def status(self) -> Dict[str, int | bool]:
        return {
            "active": self.active,
            "idx": self.follower.index(),
            "total": self.follower.total(),
        }
