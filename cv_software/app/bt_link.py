from __future__ import annotations
import os, time, struct, asyncio, threading
from typing import List, Tuple, Optional

from bleak import BleakClient, BleakScanner  # pip install bleak

# Nordic UART UUIDs
NUS_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
RX_CHAR_UUID     = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # Write
TX_CHAR_UUID     = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # Notify

# Targeting (substring match on name, or exact address if provided)
_BT_NAME = os.getenv("BT_DEVICE_NAME", "DoodleBot").strip()
_BT_ADDR = (os.getenv("BT_DEVICE_ADDR", "") or "").lower()
_SCAN_SECS = float(os.getenv("BT_SCAN_SECONDS", "8"))

class BTLink:
    """Minimal BLE (Nordic UART) transport with packet-ID prefix for each line."""

    HDR_MOVES  = 0xA1
    HDR_STATUS = 0x55
    HDR_ACK    = 0xAA
    HDR_RETRY  = 0xBB

    def __init__(self, port: str = "/dev/null", baud: int = 115200, read_timeout_s: float = 0.02):
        # Runtime state
        self._client: Optional[BleakClient] = None
        self._connected = False
        self._connected_target = "unknown"

        self._idle = True

        # Packet-ID counter (0..255 wrap)
        self._pid = -1

        self._rx_flag = False
        self._rx_time: float = 0.0
        self._rx_pid: Optional[int] = None
        self._rx_msg: str = ""

        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._thread: Optional[threading.Thread] = None
        self._ready_evt: Optional[threading.Event] = None

    # ---------------- Public API ----------------

    def connect(self) -> None:
        """Scan, connect, and start notifications (blocking until connected)."""
        self._start_loop()
        self._run(self._async_connect())
        self._connected = True

    def close(self) -> None:
        """Stop notifications, disconnect, and stop the loop."""
        try:
            self._run(self._async_close())
        finally:
            self._stop_loop()
            self._connected = False

    def is_idle(self) -> bool:
        return self._idle

    def wait_idle(self, timeout_s: float = 1.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            if self._idle:
                return True
            time.sleep(0.02)
        return self._idle

    def stop(self) -> None:
        """Send a simple 'pen up/stop' move using the binary header format."""
        if not self._connected:
            return
        payload = struct.pack("<ffb", 0.0, 0.0, +1)  # dz = +1 => pen up/stop
        pkt = bytes([self.HDR_MOVES, 1]) + payload + bytes([self._checksum(payload)])
        self._run(self._async_write(pkt, response=False))

    def send_moves_with_dz(self, moves: List[Tuple[float, float, int]]) -> None:
        """Binary move packets (not used when you send raw G-code — kept for compatibility)."""
        if not self._connected or not moves:
            return
        for dx, dy, dz in moves:
            dzc = 1 if int(dz) > 0 else (-1 if int(dz) < 0 else 0)
            payload = struct.pack("<ffb", float(dx), float(dy), dzc)
            pkt = bytes([self.HDR_MOVES, 1]) + payload + bytes([self._checksum(payload)])
            self._run(self._async_write(pkt, response=False))
            time.sleep(0.005)

    def send_gcode(
        self,
        gcode_str: str,
        *,
        chunk_bytes: int = 120,
        inter_chunk_s: float = 0.015,   # small pacing
        eol: str = "\n",
        write_with_response: bool = False,
        wait_for_ack: bool = False,
        max_retries: int = 3,
    ) -> None:
        """Line-oriented G-code with packet-id as the first byte, like your test script."""
        if not gcode_str or not self._connected:
            return

        lines = [
            ln.rstrip()
            for ln in gcode_str.replace("\r\n", "\n").replace("\r", "\n").splitlines()
            if ln.strip()
        ]
        if not lines:
            return

        for line in lines:
            # Build packet: <pid><utf8_line + eol>
            pid = self._next_pid()
            packet = bytes([pid]) + (line + eol).encode("utf-8")

            # Reset notify tracking
            self._rx_flag = False
            self._rx_pid = None
            self._rx_msg = ""
            t0 = time.time()

            # Try send (optionally flip response flag if peer rejects)
            self._run(self._async_write(packet, response=write_with_response))

            if not wait_for_ack:
                time.sleep(inter_chunk_s)
                continue

            # Minimal ACK wait loop (expects the peer to echo 'ok' with same pid)
            ok = False
            timeout = 2.0
            end = t0 + timeout
            while time.time() < end:
                if self._rx_flag and self._rx_pid == pid and "ok" in self._rx_msg:
                    ok = True
                    break
                time.sleep(0.01)

            if not ok:
                # simple bounded retry
                retries = 0
                while retries < max_retries and not ok:
                    retries += 1
                    self._rx_flag = False
                    self._run(self._async_write(packet, response=write_with_response))
                    t0 = time.time()
                    end = t0 + timeout
                    while time.time() < end:
                        if self._rx_flag and self._rx_pid == pid and "ok" in self._rx_msg:
                            ok = True
                            break
                        time.sleep(0.01)

            # small pacing either way
            time.sleep(inter_chunk_s)

    # ---------------- Internals ----------------

    def _next_pid(self) -> int:
        self._pid = (self._pid + 1) % 256
        return self._pid

    @staticmethod
    def _checksum(payload: bytes) -> int:
        return sum(payload) & 0xFF

    def _start_loop(self) -> None:
        if self._loop:
            return
        self._loop = asyncio.new_event_loop()
        self._ready_evt = threading.Event()

        def _runner():
            asyncio.set_event_loop(self._loop)  # noqa
            self._ready_evt.set()
            self._loop.run_forever()

        self._thread = threading.Thread(target=_runner, daemon=True)
        self._thread.start()
        assert self._ready_evt is not None
        self._ready_evt.wait()

    def _stop_loop(self) -> None:
        if not self._loop:
            return
        self._loop.call_soon_threadsafe(self._loop.stop)
        if self._thread:
            self._thread.join(timeout=1.0)
        self._loop = None
        self._thread = None
        self._ready_evt = None

    def _run(self, coro):
        if not self._loop:
            raise RuntimeError("BLE loop not running")
        return asyncio.run_coroutine_threadsafe(coro, self._loop).result()

    async def _async_connect(self):
        # Choose target by exact addr, else by name substring, else any NUS advertiser
        devices = await BleakScanner.discover(timeout=_SCAN_SECS)
        print("Discovered BLE devices:")
        target = None
        if _BT_ADDR:
            target = next((d for d in devices if (getattr(d, "address", "") or "").lower() == _BT_ADDR), None)
        if not target and _BT_NAME:
            target = next((d for d in devices if d.name and _BT_NAME.lower() in d.name.lower()), None)
        print(target)
        if not target:
            # fallback: any device advertising NUS service UUID
            for d in devices:
                meta = getattr(d, "metadata", {}) or {}
                uuids = [u.lower() for u in meta.get("uuids", []) if isinstance(u, str)]
                if any(u == NUS_SERVICE_UUID or u.startswith(NUS_SERVICE_UUID[:8]) for u in uuids):
                    target = d
                    break
        if not target:
            raise RuntimeError("No suitable BLE device found for Nordic UART")

        client = BleakClient(target)
        await client.connect(timeout=6.0)
        await client.start_notify(TX_CHAR_UUID, self._on_notify)

        self._client = client
        self._connected = True
        name = getattr(target, "name", "") or "(no name)"
        addr = getattr(target, "address", "") or "unknown"
        self._connected_target = f"{name} ({addr})"
        print(f"Connected to BLE target: {self._connected_target}")
        

    async def _async_close(self):
        if self._client:
            try:
                await self._client.stop_notify(TX_CHAR_UUID)
            except Exception:
                pass
            try:
                await self._client.disconnect()
            except Exception:
                pass
        self._client = None
        self._connected = False

    async def _async_write(self, data: bytes, response: bool = False):
        if not self._client or not self._client.is_connected:
            raise RuntimeError("BLE not connected")
        try:
            await self._client.write_gatt_char(RX_CHAR_UUID, data, response=response)
        except Exception as e:
            if "request not supported" in str(e).lower() or "CBATTErrorDomain Code=6" in str(e):
                await self._client.write_gatt_char(RX_CHAR_UUID, data, response=not response)
            else:
                raise

    def _on_notify(self, _uuid: str, data: bytearray):
        self._rx_time = time.time()
        self._rx_flag = True
        if not data:
            self._rx_pid, self._rx_msg = None, ""
            return

        b0 = data[0]
        if b0 == self.HDR_STATUS and len(data) >= 2:
            self._idle = bool(data[1] & 0x01)
            return

        self._rx_pid = b0
        self._rx_msg = data[1:].decode(errors="ignore").strip()
