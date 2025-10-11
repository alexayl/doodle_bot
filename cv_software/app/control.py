# app/control.py (pass-through, no closed-loop on the Pi)
from __future__ import annotations
from dataclasses import dataclass
from typing import Optional, List, Dict, Any, Union

@dataclass(frozen=True)
class MoveMM:
    dx_mm: float
    dy_mm: float
    speed_mm_s: Optional[float] = None

@dataclass(frozen=True)
class PenCmd:
    down: bool

Instruction = Union[MoveMM, PenCmd]

def parse_moves_payload(payload: Dict[str, Any]) -> List[Instruction]:
    """
    Accepts:
      { "frame":"board", "moves":[{"dx":..,"dy":..,"speed":..}, ...] }
      { "frame":"board", "waypoints":[[x,y], [x,y], ...] }  # absolute points (mm)
      [[dx,dy], ...] or [{"dx":..,"dy":..}, ...]
    Returns: list[MoveMM]
    """
    # list input variants
    if isinstance(payload, list):
        out: List[Instruction] = []
        for item in payload:
            if isinstance(item, dict):
                out.append(MoveMM(float(item["dx"]), float(item["dy"]),
                                  float(item["speed"]) if "speed" in item else None))
            else:
                dx, dy = item
                out.append(MoveMM(float(dx), float(dy)))
        return out

    frame = str(payload.get("frame", "board")).lower()
    if frame != "board":
        raise ValueError("Unsupported frame; expected 'board'")

    out: List[Instruction] = []
    if "moves" in payload:
        for m in payload["moves"]:
            out.append(MoveMM(
                dx_mm=float(m["dx"]),
                dy_mm=float(m["dy"]),
                speed_mm_s=float(m["speed"]) if "speed" in m else None,
            ))
        return out

    if "waypoints" in payload:
        wps: List[List[float]] = payload["waypoints"]
        # Convert absolute waypoints to relative legs for firmware
        for (x0, y0), (x1, y1) in zip(wps[:-1], wps[1:]):
            out.append(MoveMM(dx_mm=float(x1 - x0), dy_mm=float(y1 - y0)))
        return out

    raise ValueError("Payload must include 'moves' or 'waypoints'")