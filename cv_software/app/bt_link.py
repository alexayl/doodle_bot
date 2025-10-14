from __future__ import annotations
import struct
import time
import serial
from typing import List, Tuple, Optional

class BTLink:
    """
    ble transport for waypoint batches and idle polling.

    Protocol
    --------
    Host → Firmware:
      [0xAA][count][<float dx,dy> * count][checksum(payload)]
      - count is number of (dx,dy) float pairs (max 255)
      - checksum is sum(payload) & 0xFF

    Firmware → Host:
      [0x55][status]
      - bit0==1 => IDLE (ready for more vertices)

    Methods
    -------
    - connect/close: open/close the serial port.
    - wait_idle(): poll status bytes until firmware reports idle (or timeout).
    - send_waypoints(vertices): send a bounded batch of (dx,dy) mm increments.
    - stop(): send an empty batch to signal halt.
    """
    HDR_WAYPOINTS = 0xAA
    HDR_STATUS = 0x55

    def __init__(self, port: str = "/dev/ttyUSB0", baud: int = 115200, read_timeout_s: float = 0.02):
        self.port = port
        self.baud = baud
        self.ser: Optional[serial.Serial] = None
        self._idle = True
        self._timeout = read_timeout_s

    def connect(self) -> None:
        """Open the serial port and allow the device a brief settle time."""
        self.ser = serial.Serial(self.port, self.baud, timeout=self._timeout)
        time.sleep(0.1)

    def close(self) -> None:
        """Close the serial port if open."""
        if self.ser:
            self.ser.close()

    def _poll_status_bytes(self) -> None:
        """Drain any status frames and update the cached idle flag."""
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
        """Return True if the last parsed status frame indicated IDLE."""
        self._poll_status_bytes()
        return self._idle

    def wait_idle(self, timeout_s: float = 1.0) -> bool:
        """Block (briefly) until firmware reports IDLE, or timeout; returns final idle state."""
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            if self.is_idle():
                return True
            time.sleep(0.02)
        return self.is_idle()

    @staticmethod
    def _checksum(payload: bytes) -> int:
        """Simple 8-bit checksum used by the waypoint packet."""
        return sum(payload) & 0xFF

    def send_waypoints(self, vertices: List[Tuple[float, float]]) -> None:
        """Send up to 255 (dx,dy) float pairs as a single packet; silently no-op if not open/empty."""
        if not self.ser or not self.ser.is_open or not vertices:
            return
        count = min(len(vertices), 255)
        payload = b"".join(struct.pack("<ff", float(dx), float(dy)) for dx, dy in vertices[:count])
        pkt = bytes([self.HDR_WAYPOINTS, count]) + payload + bytes([self._checksum(payload)])
        self.ser.write(pkt)
        self.ser.flush()

    def stop(self) -> None:
        """Send an empty waypoint packet to request a halt."""
        if not self.ser or not self.ser.is_open:
            return
        payload = b""
        pkt = bytes([self.HDR_WAYPOINTS, 0x00]) + payload + bytes([self._checksum(payload)])
        self.ser.write(pkt)
        self.ser.flush()
