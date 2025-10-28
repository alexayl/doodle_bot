from __future__ import annotations
import re
from typing import List, Tuple, Optional

__all__ = [
    "load_gcode_file",
    "convert_pathfinding_gcode",
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
    inc_mode = False  # G91 on -> incremental; G90 -> absolute

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
    Convert pathfinding G-code format to firmware-compatible format.
    
    Normalize G-code produced by pathfinding into firmware-compatible commands.

    Accepted inputs (from path2gcode.py or legacy files):
    - path2gcode.py format:
        G90            ; absolute preamble (KEPT for initialization)
        M280 P0 S90    ; pen up (KEPT)
        G1 X.. Y..     ; absolute move to start (KEPT)
        G91            ; switch to relative (KEPT)
        G1 X.. Y..     ; relative stroke segments (KEPT)
        M280 P0 S0/90  ; pen down/up (KEPT)
    - Legacy format with M3/M5 will be converted to M280.
    
    Rules:
    - Keep G90, initial M280, and absolute positioning move to start.
    - Keep G91 and all subsequent relative moves.
    - Convert M3/M5 to M280 P0 S0/S90.
    - Remove comments.
    
    The firmware supports:
    - G90: Absolute positioning
    - G91: Relative positioning
    - G1 X<mm> Y<mm>: Linear movement
    - M280 P<servo> S<position>: Servo control
    
    Args:
        gcode_text: G-code string from pathfinding
    
    Returns:
        Firmware-compatible G-code string (with M280 commands)
    
    Example:
        with open("pathfinding/draw.gcode") as f:
            original = f.read()
        compatible = convert_pathfinding_gcode(original)
        bt.send_gcode(compatible, wait_for_ack=True)
    """
    out_lines: List[str] = []
    inc_mode = False  # Track if we're in relative mode (G91)
    
    for raw_line in gcode_text.splitlines():
        # Remove comments
        line = raw_line.split(";")[0].strip()
        
        # Skip empty lines
        if not line:
            continue
        
        line_upper = line.upper()
        
        # Map legacy M3/M5 to M280
        if line_upper.startswith("M3") or line_upper == "M03":
            out_lines.append("M280 P0 S0")
            continue
        if line_upper.startswith("M5") or line_upper == "M05":
            out_lines.append("M280 P0 S90")
            continue

        # Keep G90 (absolute positioning for initialization)
        if line_upper.startswith("G90"):
            out_lines.append("G90")
            inc_mode = False
            continue

        # Keep G91 (relative positioning)
        if line_upper.startswith("G91"):
            out_lines.append("G91")
            inc_mode = True
            continue

        # Pass M280 through unchanged
        if line_upper.startswith("M280"):
            out_lines.append(line)
            continue

        # Handle all G0/G1 moves (both absolute and relative)
        if line_upper.startswith(("G0 ", "G1 ", "G0\t", "G1\t", "G0", "G1")):
            out_lines.append(line)
            continue

        # Ignore everything else
        continue

    return "\n".join(out_lines) + "\n" if out_lines else ""


def scale_gcode_to_board(
    gcode_text: str,
    source_size_mm: Tuple[float, float],
    target_size_mm: Tuple[float, float],
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
                    x_val = float(x_match.group(1)) * scale_x
                    parts.append(f"X{x_val:.3f}")
                if y_match:
                    y_val = float(y_match.group(1)) * scale_y
                    parts.append(f"Y{y_val:.3f}")
                lines.append(" ".join(parts))
            else:
                lines.append(f"{cmd}")
        else:
            # Keep all other commands as-is (M280, G91, etc.)
            lines.append(line)
    
    return "\n".join(lines) + "\n" if lines else ""
