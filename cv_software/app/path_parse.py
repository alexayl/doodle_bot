from __future__ import annotations
import re
from typing import List, Tuple, Optional

__all__ = ["load_gcode_file", "ops_to_gcode", "moves_to_gcode", "convert_pathfinding_gcode"]

_NUM_RE = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)"

def load_gcode_file(path: str) -> List[Tuple]:
    """
    Returns ops as:
      - ("move", dx, dy, dz)   # incremental (G91) move in mm; dz ∈ {-1,0,1} meaning:
                               #   +1 pen UP after move, 0 no change, -1 pen DOWN after move
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
        dz = _match_float(r"\bZ(" + _NUM_RE + r")\b", u)

        if dx is None or dy is None or dz is None:
            tail = re.sub(r"^\s*G[01]\s*", "", line, flags=re.I)
            nums = re.findall(_NUM_RE, tail)
            if dx is None and len(nums) >= 1: dx = float(nums[0])
            if dy is None and len(nums) >= 2: dy = float(nums[1])
            if dz is None and len(nums) >= 3: dz = float(nums[2])

        dx = float(dx) if dx is not None else 0.0
        dy = float(dy) if dy is not None else 0.0
        dz_i = 0
        if dz is not None:
            zf = float(dz)
            dz_i = 1 if zf > 0 else (-1 if zf < 0 else 0)

        if inc_mode:
            if dx != 0.0 or dy != 0.0 or dz_i != 0:
                ops.append(("move", dx, dy, dz_i))
        else:
            if dz_i != 0:
                ops.append(("pen", False if dz_i > 0 else True))

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


def ops_to_gcode(ops: List[Tuple]) -> str:
    """
    Convert parsed operations back to G-code string format.
    
    Supported firmware commands:
    - G91: Enable relative positioning
    - G1 X<mm> Y<mm>: Linear interpolated movement  
    - M280 P<servo> S<position>: Move servo (P0=pen, P1=eraser, S=0-180)
    
    Args:
        ops: List of operations from load_gcode_file()
             - ("move", dx, dy, dz) where dz: +1=pen up, -1=pen down, 0=no change
             - ("pen", bool) where True=down, False=up
    
    Returns:
        G-code string ready to send via BTLink.send_gcode()
    
    Example:
        ops = [("pen", False), ("move", 10.0, -5.0, 0), ("pen", True)]
        gcode = ops_to_gcode(ops)
        # Returns: "G91\\nM280 P0 S90\\nG1 X10.0 Y-5.0\\nM280 P0 S0\\n"
    """
    lines = ["G91"]  # Always start in relative mode
    
    for op in ops:
        if op[0] == "pen":
            pen_down = op[1]
            if pen_down:
                # Pen down: M280 P0 (pen servo) S0 (down position)
                lines.append("M280 P0 S0")
            else:
                # Pen up: M280 P0 (pen servo) S90 (up position)
                lines.append("M280 P0 S90")
                
        elif op[0] == "move":
            _, dx, dy, dz = op
            
            # Handle pen change before move if dz != 0
            if dz < 0:  # Pen down before move
                lines.append("M280 P0 S0")
            elif dz > 0:  # Pen up before move
                lines.append("M280 P0 S90")
            
            # Add move command if there's actual movement
            if dx != 0.0 or dy != 0.0:
                lines.append(f"G1 X{dx:.3f} Y{dy:.3f}")
    
    return "\n".join(lines) + "\n"


def moves_to_gcode(moves: List[Tuple[float, float, int]]) -> str:
    """
    Convert list of (dx, dy, dz) moves to G-code string.
    
    This is a convenience function for converting the format used by
    send_moves_with_dz() to the G-code format for send_gcode().
    
    Args:
        moves: List of (dx, dy, dz) tuples where:
               dx, dy: Movement in mm
               dz: +1=pen up after, -1=pen down after, 0=no change
    
    Returns:
        G-code string
    
    Example:
        moves = [(10.0, 5.0, 0), (5.0, -3.0, 1)]
        gcode = moves_to_gcode(moves)
        bt.send_gcode(gcode)
    """
    ops = [("move", dx, dy, dz) for dx, dy, dz in moves]
    return ops_to_gcode(ops)


def convert_pathfinding_gcode(gcode_text: str) -> str:
    """
    Convert pathfinding G-code format to firmware-compatible format.
    
    Handles two pathfinding formats:
    1. M3/M5 format (manual files): M3 (pen down), M5 (pen up)
    2. G1 X Y Z format (generated files): Z=-1 (pen down), Z=1 (pen up), Z=0 (no change)
    
    Conversions:
    - M3 (pen down) → M280 P0 S0
    - M5 (pen up) → M280 P0 S90
    - G1 X Y -1 → M280 P0 S0 (pen down), then G1 X Y
    - G1 X Y 1 → G1 X Y, then M280 P0 S90 (pen up)
    - G1 X Y 0 → G1 X Y (no pen change)
    - G90 (absolute) → removed (firmware only supports G91)
    - Comments (;) → removed
    
    The firmware supports:
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
        bt.send_gcode(compatible, wait_for_ack=False)
    """
    lines = []
    
    for raw_line in gcode_text.splitlines():
        # Remove comments
        line = raw_line.split(";")[0].strip()
        
        # Skip empty lines
        if not line:
            continue
        
        line_upper = line.upper()
        
        # Convert M3 (pen down) to M280 P0 S0
        if line_upper.startswith("M3") or line_upper == "M03":
            lines.append("M280 P0 S0")
            continue
        
        # Convert M5 (pen up) to M280 P0 S90
        if line_upper.startswith("M5") or line_upper == "M05":
            lines.append("M280 P0 S90")
            continue
        
        # Skip G90 (firmware only supports relative mode)
        if line_upper.startswith("G90"):
            continue
        
        # Handle G1 X Y Z format (from path2gcode.py output)
        if line_upper.startswith(("G0 ", "G1 ")):
            # Parse the line to extract X, Y, Z values
            parts = line.split()
            if len(parts) >= 4:  # G1 X Y Z format
                try:
                    x = float(parts[1])
                    y = float(parts[2])
                    z = int(float(parts[3]))
                    
                    # Handle pen state before move
                    if z == -1:  # Pen down before move
                        lines.append("M280 P0 S0")
                    elif z == 1:  # Pen up after move (we'll add it after)
                        pass
                    
                    # Add the move command (without Z)
                    lines.append(f"G1 X{x} Y{y}")
                    
                    # Handle pen state after move
                    if z == 1:  # Pen up after move
                        lines.append("M280 P0 S90")
                    
                    continue
                except (ValueError, IndexError):
                    # If parsing fails, treat as regular G1 command
                    pass
            
            # Regular G1 X Y format (no Z) - keep as-is
            lines.append(line)
            continue
        
        # Keep G91 and M280 commands as-is
        if line_upper.startswith(("G91", "M280")):
            lines.append(line)
            continue
    
    return "\n".join(lines) + "\n" if lines else ""
