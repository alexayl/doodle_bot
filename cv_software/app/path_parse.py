from __future__ import annotations
import re
from typing import List, Tuple, Optional

__all__ = ["load_gcode_file"]

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
