from __future__ import annotations
import re
from typing import List, Tuple, Optional

__all__ = [
    "load_gcode_file",
    "convert_pathfinding_gcode",
    "scale_gcode_to_board",
    "segment_long_moves",
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
    inc_mode = True  # G91 on -> incremental; G90 -> absolute

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

        if re.match(r"^M0?3\b", u):
            ops.append(("pen", True))   # pen down
            continue
        if re.match(r"^M0?5\b", u):
            ops.append(("pen", False))  # pen up
            continue

        if not u.startswith(("G0", "G1")):
            continue

        dx = _match_float(r"\bX(" + _NUM_RE + r")\b", u)
        dy = _match_float(r"\bY(" + _NUM_RE + r")\b", u)

        if dx is None or dy is None:
            tail = re.sub(r"^\s*G[01]\s*", "", line, flags=re.I)
            nums = re.findall(_NUM_RE, tail)
            if dx is None and len(nums) >= 1: dx = float(nums[0])
            if dy is None and len(nums) >= 2: dy = float(nums[1])

        dx = float(dx) if dx is not None else 0.0
        dy = float(dy) if dy is not None else 0.0

        if inc_mode:
            if dx != 0.0 or dy != 0.0:
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


# Note: ops_to_gcode and moves_to_gcode were removed as the codebase now
# exclusively transmits full G-code over BLE (no legacy move streaming).


def convert_pathfinding_gcode(gcode_text: str) -> str:
    """
    Normalize pathfinding G-code to the firmware subset:
    - Start with G91 to ensure firmware is in relative mode FIRST
    - Drop any additional G90/G91 commands after the initial G91
    - Keep G0/G1 lines but only after G91 is encountered (skip absolute positioning moves)
    - Map M3→M280 P0 S0 (down), M5→M280 P0 S90 (up)
    - Pass through existing M280 lines
    - Strip comments/blank lines
    """
    out_lines: List[str] = []
    saw_g91 = False
    in_relative_mode = False  # Track whether we're in relative mode
    
    for raw in gcode_text.splitlines():
        line = raw.split(";")[0].strip()
        if not line:
            continue
        u = line.upper()

        # map legacy pen cmds (always keep these regardless of mode)
        if u.startswith("M3") or u == "M03":
            out_lines.append("M280 P0 S0")
            continue
        if u.startswith("M5") or u == "M05":
            out_lines.append("M280 P0 S90")
            continue

        # Track mode changes
        if u.startswith("G90"):
            in_relative_mode = False
            continue

        if u.startswith("G91"):
            in_relative_mode = True
            saw_g91 = True
            # Don't append here - we'll add it at the very beginning
            continue

        # pass pen servo through (always keep these)
        if u.startswith("M280"):
            out_lines.append(line)
            continue

        # ONLY pass motion commands if we're in relative mode (after G91)
        # Skip any absolute positioning commands that appear before G91
        if u.startswith(("G0", "G1")):
            if not in_relative_mode:
                continue  # Skip absolute positioning commands
                
            has_x = re.search(r"\bX\s*(" + _NUM_RE + r")\b", line, flags=re.IGNORECASE) is not None
            has_y = re.search(r"\bY\s*(" + _NUM_RE + r")\b", line, flags=re.IGNORECASE) is not None
            fixed = line
            if has_x and not has_y:
                fixed = f"{line} Y0"
            elif has_y and not has_x:
                fixed = f"{line} X0"
            out_lines.append(fixed)
            continue

        # ignore everything else
    
    # Prepend G91 at the very start before any other commands (including servo commands)
    if out_lines:
        out_lines.insert(0, "G91")
    
    return ("\n".join(out_lines) + "\n") if out_lines else ""


def scale_gcode_to_board(
    gcode_text: str,
    source_size_mm: Tuple[float, float],
    target_size_mm: Tuple[float, float],
    preserve_aspect: bool = True,  # NEW: preserve aspect ratio by default
) -> str:
    """
    Scale G-code coordinates from pathfinding canvas to actual detected board size.
    
    Args:
        gcode_text: G-code from pathfinding (in pathfinding canvas coordinates)
        source_size_mm: (width, height) of pathfinding canvas in mm
        target_size_mm: (width, height) of actual detected board in mm
        preserve_aspect: If True, use uniform scaling and center the path on the board
    
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
    
    if preserve_aspect:
        scale = min(scale_x, scale_y)  # Use smaller scale to fit within board
        scale_x = scale_y = scale        
        scaled_width = source_size_mm[0] * scale
        scaled_height = source_size_mm[1] * scale
        offset_x = (target_size_mm[0] - scaled_width) / 2.0
        offset_y = (target_size_mm[1] - scaled_height) / 2.0
    else:
        offset_x = offset_y = 0.0
    
    lines = []
    for raw_line in gcode_text.splitlines():
        line = raw_line.split(";")[0].strip()
        if not line:
            continue
        
        line_upper = line.upper()
        
        # Scale G0/G1 commands only
        if line_upper.startswith(("G0 ", "G1 ", "G0\t", "G1\t", "G0", "G1")):
            # Preserve the command (G0/G1) exactly as written at the start
            m_cmd = re.match(r"^(G0|G1)\b", line, flags=re.IGNORECASE)
            cmd = m_cmd.group(1).upper() if m_cmd else line[:2].upper()

            # Extract X and Y values using the robust numeric regex
            x_match = re.search(r"\bX\s*(" + _NUM_RE + r")\b", line, flags=re.IGNORECASE)
            y_match = re.search(r"\bY\s*(" + _NUM_RE + r")\b", line, flags=re.IGNORECASE)

            if x_match or y_match:
                parts: List[str] = [cmd]
                if x_match:
                    x_val = float(x_match.group(1)) * scale_x + offset_x
                    parts.append(f"X{x_val:.3f}")
                if y_match:
                    y_val = float(y_match.group(1)) * scale_y + offset_y
                    parts.append(f"Y{y_val:.3f}")
                lines.append(" ".join(parts))
            else:
                lines.append(f"{cmd}")
        else:
            # Keep all other commands as-is (M280, G91, etc.)
            lines.append(line)
    
    return "\n".join(lines) + "\n" if lines else ""


def segment_long_moves(gcode_text: str, max_segment_mm: float = 100.0, max_axis_segment_mm: Optional[float] = None) -> str:
    """
    Split long relative motion commands into smaller segments to satisfy firmware constraints.

    - Only processes G0/G1 lines (relative deltas assumed).
    - Ensures both X and Y tokens exist (adds 0 if missing).
    - Preserves the motion type (G0 or G1).
    """
    if not gcode_text:
        return gcode_text

    # Some firmware paths truncate to integers internally. Allow forcing integer mm output
    # as a mitigation via env var GCODE_INT_MM=1 (rounds X/Y to nearest int).
    import os as _os
    force_int_mm = (_os.getenv("GCODE_INT_MM", "0").strip() in {"1", "true", "True"})

    out: List[str] = []
    if max_axis_segment_mm is None:
        max_axis_segment_mm = max_segment_mm

    def _fmt_xy(cmd: str, x: float, y: float, fval: Optional[float] = None, include_f: bool = False) -> str:
        if force_int_mm:
            sx = int(round(x))
            sy = int(round(y))
            seg = f"{cmd} X{sx} Y{sy}"
        else:
            seg = f"{cmd} X{x:.3f} Y{y:.3f}"
        if include_f and fval is not None:
            # Preserve typical integer formatting for feed rates
            if abs(fval - int(fval)) < 1e-6:
                seg += f" F{int(fval)}"
            else:
                seg += f" F{fval:.0f}"
        return seg

    for raw in gcode_text.splitlines():
        line = raw.split(";")[0].strip()
        if not line:
            continue
        u = line.upper()
        if not u.startswith(("G0", "G1")):
            out.append(line)
            continue

        # Extract X and Y (default to 0 if missing)
        m_x = re.search(r"\bX\s*(" + _NUM_RE + r")\b", line, flags=re.IGNORECASE)
        m_y = re.search(r"\bY\s*(" + _NUM_RE + r")\b", line, flags=re.IGNORECASE)
        m_f = re.search(r"\bF\s*(" + _NUM_RE + r")\b", line, flags=re.IGNORECASE)
        dx = float(m_x.group(1)) if m_x else 0.0
        dy = float(m_y.group(1)) if m_y else 0.0
        fval = float(m_f.group(1)) if m_f else None
        cmd_match = re.match(r"^(G0|G1)\b", line, flags=re.IGNORECASE)
        cmd = cmd_match.group(1).upper() if cmd_match else "G1"

        mag = (dx * dx + dy * dy) ** 0.5
        ax = abs(dx)
        ay = abs(dy)
        if (mag <= max_segment_mm and ax <= max_axis_segment_mm and ay <= max_axis_segment_mm) or mag <= 0.0:
            # Re-emit the (possibly short) move, optionally coercing to integer mm
            # Preserve command and include feed if present
            include_f = fval is not None
            out.append(_fmt_xy(cmd, dx, dy, fval=fval, include_f=include_f))
            continue

        # Segment into n parts
        import math as _math
        n_mag = int(_math.ceil(mag / max_segment_mm)) if max_segment_mm > 0 else 1
        n_x = int(_math.ceil(ax / max_axis_segment_mm)) if max_axis_segment_mm and max_axis_segment_mm > 0 else 1
        n_y = int(_math.ceil(ay / max_axis_segment_mm)) if max_axis_segment_mm and max_axis_segment_mm > 0 else 1
        n = max(1, n_mag, n_x, n_y)
        sx = dx / n
        sy = dy / n
        for k in range(n):
            seg = _fmt_xy(cmd, sx, sy, fval=fval, include_f=(k == 0))
            out.append(seg)

    return ("\n".join(out) + "\n") if out else ""
