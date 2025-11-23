from __future__ import annotations
import os
import re
from typing import List, Tuple, Optional

__all__ = [
    "load_gcode_file",
    "scale_gcode_to_board",
]

_NUM_RE = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)"

def load_gcode_file(path: str) -> List[Tuple]:
    """
    Returns ops as:
      - ("move", dx, dy)       # incremental (G91) move in mm
      - ("pen", True|False)    # True = down (M3), False = up (M5)
    """
    text = _read_text(path)
    ops: List[Tuple] = []
    inc_mode = None  # None=auto-detect, True=G91, False=G90

    for raw in text.splitlines():
        line = raw.split(";")[0].strip()
        if not line:
            continue
        u = line.upper()

        # modes
        if u.startswith("G91"):
            inc_mode = True
            continue
        if u.startswith("G90"):
            inc_mode = False
            continue

        # Pen commands: M3/M5 (legacy) or M280 (servo)
        if re.match(r"^M0?3\b", u):
            ops.append(("pen", True))   # pen down
            continue
        if re.match(r"^M0?5\b", u):
            ops.append(("pen", False))  # pen up
            continue
        # M280 P0 S0 = down, M280 P0 S90 = up
        # if u.startswith("M280"):
        #     if "S0" in u or "S 0" in u:
        #         ops.append(("pen", True))
        #     elif "S90" in u or "S 90" in u:
        #         ops.append(("pen", False))
        #     continue

        if not u.startswith(("G0", "G1")):
            continue

        dx = _match_float(r"\bX(" + _NUM_RE + r")\b", u)
        dy = _match_float(r"\bY(" + _NUM_RE + r")\b", u)

        if dx is None or dy is None:
            tail = re.sub(r"^\s*G[01]\s*", "", line, flags=re.I)
            nums = re.findall(_NUM_RE, tail)
            if dx is None and len(nums) >= 1: dx = float(nums[0])
            if dy is None and len(nums) >= 2: dy = float(nums[1])

        dx = int(round(float(dx))) if dx is not None else 0
        dy = int(round(float(dy))) if dy is not None else 0

        # Auto-detect: if no G90/G91 specified, assume relative (pathfinding default)
        if inc_mode is None:
            inc_mode = True
            
        if inc_mode:
            if dx != 0 or dy != 0:
                ops.append(("move", dx, dy))

    return ops

def _read_text(path: str) -> str:
    with open(path, "r", encoding="utf-8") as f:
        return f.read()

def _match_float(pat: str, s: str) -> Optional[float]:
    m = re.search(pat, s, flags=re.I)
    if not m:
        return None
    try:
        return float(m.group(1))
    except ValueError:
        return None

def scale_gcode_to_board(
    gcode_text: str,
    source_size_mm: Tuple[float, float],
    target_size_mm: Tuple[float, float],
    preserve_aspect: bool = False,
) -> str:
    """
    Scale G-code coordinates from pathfinding canvas to actual detected board size.
    
    Args:
        gcode_text: G-code from pathfinding (in pathfinding canvas coordinates)
        source_size_mm: (width, height) of pathfinding canvas in mm
        target_size_mm: (width, height) of actual detected board in mm
    
    Returns:
        Scaled G-code with coordinates adjusted to target board size
        
    Example:
        # Pathfinding uses 575x730 canvas → 200x250 mm
        # Actual board detected as 200x250 mm
        source = (200.0, 250.0)  # pathfinding assumes this size
        target = cvp.board_size_mm()  # actual detected size (200, 250)
        scaled = scale_gcode_to_board(pathfinding_gcode, source, target)
    """
    if not gcode_text or source_size_mm == target_size_mm:
        return gcode_text
    
    scale_x = target_size_mm[0] / source_size_mm[0]
    scale_y = target_size_mm[1] / source_size_mm[1]
    if bool(preserve_aspect):
        s = min(scale_x, scale_y)
        scale_x = s
        scale_y = s
    
    lines = []
    for raw_line in gcode_text.splitlines():
        line = raw_line.split(";")[0].strip()
        if not line:
            continue
        
        line_upper = line.upper()
        
        # Scale G0/G1 commands only
        if line_upper.startswith(("G0 ", "G1 ", "G0\t", "G1\t", "G0", "G1")):
            m_cmd = re.match(r"^(G0|G1)\b", line, flags=re.IGNORECASE)
            cmd = m_cmd.group(1).upper() if m_cmd else line[:2].upper()

            x_match = re.search(r"\bX\s*(" + _NUM_RE + r")\b", line, flags=re.IGNORECASE)
            y_match = re.search(r"\bY\s*(" + _NUM_RE + r")\b", line, flags=re.IGNORECASE)

            if x_match or y_match:
                parts: List[str] = [cmd]
                if x_match:
                    x_val = int(round(float(x_match.group(1)) * scale_x))
                    parts.append(f"X{x_val}")
                if y_match:
                    y_val = int(round(float(y_match.group(1)) * scale_y))
                    parts.append(f"Y{y_val}")
                lines.append(" ".join(parts))
            else:
                lines.append(f"{cmd}")
        else:
            # Keep all other commands as-is (M280, G91, etc.)
            lines.append(line)
    
    return "\n".join(lines) + "\n" if lines else ""