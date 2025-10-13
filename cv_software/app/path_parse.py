from __future__ import annotations
import re
from typing import List, Tuple, Optional

__all__ = ["load_gcode_file"]

def load_gcode_file(path: str) -> List[Tuple]:
    """
    Load G-code commands from a file and parse them into a list of operations.

    Args:
        path (str): The file path to the G-code file.

    Returns:
        List[Tuple]: A list of parsed operations, where each operation is represented as a tuple.
    """
    text = _read_text(path).strip()
    ops: List[Tuple] = []
    inc_mode = False
    for raw in text.splitlines():
        line = raw.split(";")[0].strip().upper()
        if not line:
            continue
        if line.startswith("G91"):
            inc_mode = True
            continue
        if line.startswith("G90"):
            inc_mode = False
            continue
        if re.match(r"^M0?3\b", line):
            ops.append(("pen", True))
            continue
        if re.match(r"^M0?5\b", line):
            ops.append(("pen", False))
            continue
        if line.startswith("G1"):
            dx = _match_float(r"X(-?\d+(?:\.\d+)?)", line) or 0.0
            dy = _match_float(r"Y(-?\d+(?:\.\d+)?)", line) or 0.0
            ops.append(("move", float(dx), float(dy)))
    return ops

def _read_text(path: str) -> str:
    with open(path, "r", encoding="utf-8") as f:
        return f.read()

def _match_float(pat: str, s: str) -> Optional[float]:
    m = re.search(pat, s)
    if not m:
        return None
    return float(m.group(1))