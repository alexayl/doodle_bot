from __future__ import annotations
import struct
import time
import serial
from typing import List, Tuple, Optional

class BTLink:
    """
    Binary transport for batches of G1-like increments.

    Packet format (little-endian):
      [0xA1][count][(float32 dx, float32 dy, int8 dz) * count][checksum]

    Where dz ∈ {-1,0,1} and means **after the move**:
      -1 => pen DOWN
       0 => no change
      +1 => pen UP

    A pure pen change is encoded as (0,0,±1).

    Status polling (optional, tolerated if unsupported by FW):
      FW may asynchronously send frames [0x55][status] where bit0==1 means IDLE.
    """
    HDR_MOVES = 0xA1
    HDR_STATUS = 0x55

    def __init__(self, port: str = "/dev/ttyUSB0", baud: int = 115200, read_timeout_s: float = 0.02):
        self.port = port
        self.baud = baud
        self._timeout = read_timeout_s
        self.ser: Optional[serial.Serial] = None
        self._idle = True

    def connect(self) -> None:
        """Open the serial port and allow the device a brief settle time."""
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
        i, n = 0, len(data)
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

    # --- tx -------------------------------------------------------------------
    @staticmethod
    def _checksum(payload: bytes) -> int:
        return sum(payload) & 0xFF

    def send_moves_with_dz(self, moves: List[Tuple[float, float, int]]) -> None:
        """
        Send up to 170 triplets (dx,dy,dz) in one packet.
        """
        if not self.ser or not self.ser.is_open or not moves:
            return

        # dz to {-1,0,1}
        moves = [(float(dx), float(dy), 1 if int(dz) > 0 else (-1 if int(dz) < 0 else 0)) for dx, dy, dz in moves]

        max_per_pkt = min(len(moves), 170)
        payload = bytearray()
        for dx, dy, dz in moves[:max_per_pkt]:
            payload += struct.pack("<ffb", dx, dy, dz)

        pkt = bytes([self.HDR_MOVES, max_per_pkt]) + bytes(payload) + bytes([self._checksum(payload)])
        self.ser.write(pkt)
        self.ser.flush()

    def stop(self) -> None:
        """Raise pen and stop streaming (best-effort)."""
        if not self.ser or not self.ser.is_open:
            return
        # send a single pen-up change; host-side follower will stop too
        payload = struct.pack("<ffb", 0.0, 0.0, +1)
        pkt = bytes([self.HDR_MOVES, 1]) + payload + bytes([self._checksum(payload)])
        self.ser.write(pkt)
        self.ser.flush()
