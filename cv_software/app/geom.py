from __future__ import annotations
from typing import List, Tuple
from app.control import Waypoint
def fit_path_to_board(wps: List[Waypoint], board_mm: Tuple[float, float], margin_frac: float = 0.10) -> List[Waypoint]:
    """Scale normalized [0,1] waypoints to board mm coordinates with margin."""
    if not wps:
        return []
    Wmm, Hmm = board_mm
    
    # Input waypoints are normalized [0,1]
    # Find bounding box in normalized space
    xs = [w.x_mm for w in wps]
    ys = [w.y_mm for w in wps]
    minx = min(xs)
    maxx = max(xs)
    miny = min(ys)
    maxy = max(ys)
    
    # Compute normalized dimensions
    w_norm = max(maxx - minx, 1e-6)
    h_norm = max(maxy - miny, 1e-6)
    
    # Scale to fit board with margin, preserving aspect ratio
    target_w = (1.0 - 2 * margin_frac) * Wmm
    target_h = (1.0 - 2 * margin_frac) * Hmm
    scale = min(target_w / w_norm, target_h / h_norm)
    
    # Center of normalized path
    cx_norm = 0.5 * (minx + maxx)
    cy_norm = 0.5 * (miny + maxy)
    
    # Center of board in mm
    cx_board = 0.5 * Wmm
    cy_board = 0.5 * Hmm
    
    out = []
    for p in wps:
        # Translate to origin, scale, then translate to board center
        x_mm = (p.x_mm - cx_norm) * scale + cx_board
        y_mm = (p.y_mm - cy_norm) * scale + cy_board
        
        # STRICT: Clamp all waypoints to board bounds (with margin)
        x_mm = max(margin_frac * Wmm, min(x_mm, (1.0 - margin_frac) * Wmm))
        y_mm = max(margin_frac * Hmm, min(y_mm, (1.0 - margin_frac) * Hmm))
        
        out.append(Waypoint(x_mm, y_mm))
    return out
