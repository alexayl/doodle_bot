from __future__ import annotations
from typing import List, Tuple
from app.control import Waypoint

def fit_path_to_board(wps: List[Waypoint], board_mm: Tuple[float, float], margin_frac: float = 0.10) -> List[Waypoint]:
    if not wps:
        return []
    Wmm, Hmm = board_mm
    xs = [w.x_mm for w in wps]
    ys = [w.y_mm for w in wps]
    minx = min(xs)
    maxx = max(xs)
    miny = min(ys)
    maxy = max(ys)
    w = (maxx - minx) or 1.0
    h = (maxy - miny) or 1.0
    s = min((1 - 2 * margin_frac) * Wmm / w, (1 - 2 * margin_frac) * Hmm / h)
    cx_src = 0.5 * (minx + maxx)
    cy_src = 0.5 * (miny + maxy)
    cx_dst = 0.5 * Wmm
    cy_dst = 0.5 * Hmm
    out = []
    for p in wps:
        x = (p.x_mm - cx_src) * s + cx_dst
        y = (p.y_mm - cy_src) * s + cy_dst
        out.append(Waypoint(x, y))
    return out