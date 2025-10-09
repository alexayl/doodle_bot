from __future__ import annotations
import serial
from typing import Optional

class BTLink:
    def __init__(self, port: str, baud: int = 115200, timeout: float = 0.2):
        self._ser: Optional[serial.Serial] = None
        self.port, self.baud, self.timeout = port, baud, timeout

    def connect(self):
        self._ser = serial.Serial(self.port, self.baud, timeout=self.timeout)

    def close(self):
        if self._ser:
            self._ser.close()
            self._ser = None

    def _write(self, pkt: bytes):
        if self._ser:
            self._ser.write(pkt)

    def send_move(self, fwd_mm: float, lef_mm: float):
        f, l = int(round(fwd_mm)), int(round(lef_mm))
        pkt = bytearray([0xAA, 0x01, (f >> 8) & 0xFF, f & 0xFF, (l >> 8) & 0xFF, l & 0xFF])
        pkt.append(sum(pkt) & 0xFF)
        self._write(pkt)

    def send_pen(self, down: bool):
        pkt = bytearray([0xAA, 0x02, 0x01 if down else 0x00])
        pkt.append(sum(pkt) & 0xFF)
        self._write(pkt)
