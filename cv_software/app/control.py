from __future__ import annotations
from dataclasses import dataclass
from typing import Optional, List, Tuple, Dict
import math

@dataclass(frozen=True)
class Waypoint:
    x_mm: float
    y_mm: float

@dataclass
class FollowerParams:
    pos_eps_mm: float = 2.0
    max_step_mm: float = 25.0
    tick_s: float = 0.08

class PathFollower:
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

    @staticmethod
    def _to_robot_frame(dx_board: float, dy_board: float, heading_rad: float) -> Tuple[float, float]:
        c = math.cos(-heading_rad)
        s = math.sin(-heading_rad)
        fwd = c * dx_board - s * dy_board
        lef = s * dx_board + c * dy_board
        return float(fwd), float(-lef)

    def step(self, pose: Optional[Tuple[Tuple[float, float], float, float]]) -> Optional[Tuple[float, float]]:
        if not self.active or not self._wps or pose is None:
            return None

        (x, y), heading, _ = pose

        while self._i < len(self._wps):
            tgt = self._wps[self._i]
            if math.hypot(tgt.x_mm - x, tgt.y_mm - y) <= self.p.pos_eps_mm:
                self._i += 1
            else:
                break

        if self._i >= len(self._wps):
            self.stop()
            return None

        tgt = self._wps[self._i]
        ex = tgt.x_mm - x
        ey = tgt.y_mm - y
        dist = math.hypot(ex, ey)
        if dist <= self.p.pos_eps_mm:
            self._i += 1
            return None

        step_len = min(self.p.max_step_mm, dist)
        ux = ex / dist
        uy = ey / dist
        return self._to_robot_frame(ux * step_len, uy * step_len, heading)

class PathRun:
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