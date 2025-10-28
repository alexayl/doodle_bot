from __future__ import annotations
import os, time, struct, asyncio, threading
from typing import List, Tuple, Optional

from bleak import BleakClient, BleakScanner

# Nordic UART UUIDs
NUS_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
RX_CHAR_UUID     = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # Write
TX_CHAR_UUID     = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # Notify

# Targeting (substring match on name, or exact address if provided)
_BT_NAME = os.getenv("BT_DEVICE_NAME", "DoodleBot").strip()
_BT_ADDR = (os.getenv("BT_DEVICE_ADDR", "") or "").lower()
_SCAN_SECS = float(os.getenv("BT_SCAN_SECONDS", "8"))

class BTLink:
    """ BLE (Nordic UART) transport with packet-ID prefix for each line."""

    HDR_STATUS = 0x55

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


    def stop(self) -> None:
        """Send a stop command (kept for compatibility but not used with G-code flow)."""
        if not self._connected:
            return
        pass

    def send_gcode(
        self,
        gcode_str: str,
        *,
        inter_line_s: float = 0.02,
        write_with_response: bool = False,
        wait_for_ack: bool = True,
        max_retries: int = 5,
        ack_timeout_s: float = 2.0,
    ) -> None:

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
            pid = self._next_pid()
            
            packet = bytes([pid]) + (line + "\n").encode("utf-8")

            self._rx_flag = False
            self._rx_pid = None
            self._rx_msg = ""

            print(f"Sending packet {pid}: {line}")
            self._run(self._async_write(packet, response=write_with_response))

            if not wait_for_ack:
                time.sleep(inter_line_s)
                continue

            ok = False
            retries = 0
            
            while retries <= max_retries and not ok:
                if retries > 0:
                    # Retry: resend the same packet
                    self._rx_flag = False
                    self._rx_pid = None
                    self._rx_msg = ""
                    self._run(self._async_write(packet, response=write_with_response))
                    time.sleep(0.05)  # Brief delay after retry
                
                t0 = time.time()
                end = t0 + ack_timeout_s
                while time.time() < end:
                    if self._rx_flag and self._rx_pid == pid:
                        if "ok" in self._rx_msg.lower():
                            ok = True
                            break
                        elif "fail" in self._rx_msg.lower():
                            break
                    time.sleep(0.01)
                
                if ok:
                    break
                    
                retries += 1
            
            if not ok:
                print(f"Warning: Failed to get ACK for packet {pid} after {max_retries} retries: {line}")

            # Pacing between lines
            time.sleep(inter_line_s)

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
        devices = await BleakScanner.discover(timeout=_SCAN_SECS)
        print("Discovered BLE devices:")
        target = None
        if _BT_ADDR:
            target = next((d for d in devices if (getattr(d, "address", "") or "").lower() == _BT_ADDR), None)
        if not target and _BT_NAME:
            target = next((d for d in devices if d.name and _BT_NAME.lower() in d.name.lower()), None)
        print(target)
        if not target:
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
        """
        Parse firmware responses per spec:
        - <pid_byte><text>: where first byte is numeric pid, remainder is "ok" or "fail"
        - HDR_STATUS: device status (idle/busy)
        """
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
        self._rx_msg = data[1:].decode("utf-8", errors="ignore").strip()
