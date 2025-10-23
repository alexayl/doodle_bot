from __future__ import annotations
from dataclasses import dataclass
from typing import Optional, List, Tuple, Dict
import math
from app.utils import board_to_robot

@dataclass(frozen=True)
class Waypoint:
    x_mm: float
    y_mm: float

@dataclass
class FollowerParams:
    pos_eps_mm: float = 2.0
    # legacy magnitude clamp
    max_step_mm: float = 25.0
    # NEW: anisotropic caps in ROBOT frame (forward/left)
    max_step_forward_mm: float = 25.0
    max_step_left_mm: float = 12.0   # give Y/left less if you want tighter control

class PathFollower:
    """
    Minimal path follower: computes a bounded step toward the next waypoint.
    Returns a (forward,left) delta in the ROBOT frame, leaving timing/motion to firmware.
    """
    def __init__(self, params: Optional[FollowerParams] = None):
        self.p = params or FollowerParams()
        self._wps: List[Waypoint] = []
        self._i = 0
        self.active = False

    def load_waypoints(self, wps: List[Waypoint]) -> None:
        self._wps = wps or []
        self._i = 0

    def start(self) -> None:
        self.active = bool(self._wps)
        self._i = 0

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

    def step(self, pose: Optional[Tuple[Tuple[float, float], float, float]]) -> Optional[Tuple[float, float]]:
        """
        pose: ((x_mm, y_mm), heading_rad, confidence)
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
        (_, heading, _) = pose

        # Rotate to robot frame, then apply anisotropic caps
        fwd, left = self._to_robot_frame(ux * step_len, uy * step_len, heading)

        # anisotropic clamp (robot frame)
        fwd = max(-self.p.max_step_forward_mm, min(self.p.max_step_forward_mm, fwd))
        left = max(-self.p.max_step_left_mm,   min(self.p.max_step_left_mm,   left))

        return (fwd, left)



class PathRun:
    """Thin wrapper that owns a follower and active flag."""
    def __init__(self):
        self.wps: List[Waypoint] = []
        self.follower = PathFollower(FollowerParams())
        self.active = False

    def load(self, wps: List[Waypoint]) -> None:
        self.wps = wps or []
        self.follower.load_waypoints(self.wps)

    def start(self) -> None:
        if self.wps:
            self.active = True
            self.follower.start()

    def stop(self) -> None:
        self.active = False
        self.follower.stop()

    def status(self) -> Dict[str, int | bool]:
        return {"active": self.active, "idx": self.follower.index(), "total": self.follower.total()}
