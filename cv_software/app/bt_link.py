from __future__ import annotations
import struct
import time
import serial
from typing import List, Tuple, Optional

class BTLink:
    """
    BLE/serial link.
    Frame: 0xAA <count> <float dx,dy pairs> <checksum(payload)>
    Firmware sends 0x55 <status> where bit0==1 => IDLE.
    """
    HDR_WAYPOINTS = 0xAA
    HDR_STATUS = 0x55

    def __init__(self, port: str = "/dev/rfcomm0", baud: int = 115200, read_timeout_s: float = 0.02):
        self.port = port
        self.baud = baud
        self.ser: Optional[serial.Serial] = None
        self._idle = True
        self._timeout = read_timeout_s

    def connect(self) -> None:
        self.ser = serial.Serial(self.port, self.baud, timeout=self._timeout)
        time.sleep(0.1)

    def close(self) -> None:
        if self.ser:
            self.ser.close()

    def _poll_status_bytes(self) -> None:
        if not self.ser:
            return
        try:
            data = self.ser.read(64)
        except Exception:
            return
        i = 0
        n = len(data)
        while i + 1 < n:
            if data[i] == self.HDR_STATUS:
                self._idle = bool(data[i + 1] & 0x01)
                i += 2
            else:
                i += 1

    def is_idle(self) -> bool:
        self._poll_status_bytes()
        return self._idle

    def wait_idle(self, timeout_s: float = 1.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            if self.is_idle():
                return True
            time.sleep(0.02)
        return self.is_idle()

    @staticmethod
    def _checksum(payload: bytes) -> int:
        return sum(payload) & 0xFF

    def send_waypoints(self, vertices: List[Tuple[float, float]]) -> None:
        if not self.ser or not self.ser.is_open or not vertices:
            return
        count = min(len(vertices), 255)
        payload = b"".join(struct.pack("<ff", float(dx), float(dy)) for dx, dy in vertices[:count])
        pkt = bytes([self.HDR_WAYPOINTS, count]) + payload + bytes([self._checksum(payload)])
        self.ser.write(pkt)
        self.ser.flush()

    def stop(self) -> None:
        if not self.ser or not self.ser.is_open:
            return
        payload = b""
        pkt = bytes([self.HDR_WAYPOINTS, 0x00]) + payload + bytes([self._checksum(payload)])
        self.ser.write(pkt)
        self.ser.flush()