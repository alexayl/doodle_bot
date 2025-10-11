from __future__ import annotations
import serial
from typing import Optional

class BTLink:
    def connect(self): pass
    def close(self): pass
    def stop(self): print("[BT] stop")

    def send_pose(self, x_mm: float, y_mm: float, heading_rad: float, confidence: float):
        print(f"[BT] pose x={x_mm:.1f} y={y_mm:.1f} th={math.degrees(heading_rad):.1f}° conf={confidence:.2f}")

    def send_path(self, moves: list[dict]):
        print(f"[BT] path len={len(moves)} first={moves[0] if moves else None}")

    def send_move(self, fwd_mm: float, left_mm: float):
        print(f"[BT] move fwd={fwd_mm:.1f} left={left_mm:.1f}")

    def send_rotate(self, rot_rad: float):
        print(f"[BT] rotate {math.degrees(rot_rad):.1f} deg")

    def send_pen(self, down: bool):
        print(f"[BT] pen {'DOWN' if down else 'UP'}")
