# app/bt_link.py
"""
BLE Packet Communication Library for Doodle Bot
- Scans for DEVICE_NAME
- Connects and subscribes to TX notifications
- Sends line-by-line with packet IDs and retries
- Expects "ok" ACK tagged with the same packet ID
"""

from __future__ import annotations

import asyncio
import os
import time
import threading
from typing import Optional, Iterable

from bleak import BleakClient, BleakScanner

# Nordic UART UUIDs
RX_CHAR_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # Write (NUS RX)
TX_CHAR_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # Notify (NUS TX)

# Target device
DEVICE_NAME = os.getenv("BT_DEVICE_NAME", "BOO").strip()
DEVICE_ADDR = os.getenv("BT_DEVICE_ADDR", "").strip()  # optional MAC/UUID hint

# Tuning
DEFAULT_TIMEOUT_S = float(os.getenv("BT_TIMEOUT_S", "2.0"))
RETRY_SLEEP_S = float(os.getenv("BT_RETRY_SLEEP_S", "0.10"))
ERROR_COOLDOWN_S = float(os.getenv("BT_ERROR_COOLDOWN_S", "0.50"))

class BTLink:
    """
    Threaded BLE link that mirrors the behavior of the standalone BLEPacketHandler,
    but exposes a simple synchronous API for your Flask routes.
    """

    def __init__(self, device_name: str = DEVICE_NAME):
        self.device_name = device_name
        self.client: Optional[BleakClient] = None

        # Packet/ACK state
        self.packet_id_counter: int = -1
        self.response_received: bool = False
        self.last_received_packet_id: Optional[int] = None
        self.last_received_message: Optional[str] = None

        # Timeouts
        self.timeout: float = DEFAULT_TIMEOUT_S

        # Background loop
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._thread: Optional[threading.Thread] = None
        self._ready_evt: Optional[threading.Event] = None

    # ---------------- Core helpers ----------------

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

    # ---------------- Public API ----------------

    def connect(self) -> None:
        """Scan, connect, and start notifications (blocking)."""
        self._start_loop()
        self._run(self._async_connect())

    def close(self) -> None:
        """Stop notifications, disconnect, and stop the loop."""
        try:
            self._run(self._async_close())
        finally:
            self._stop_loop()

    def disconnect(self) -> None:
        self.close()

    def send_gcode(self, gcode_text: str) -> None:
        """
        Send a G-code string line by line with packet IDs and retries.
        Ignores blank lines and pure comment lines.
        """
        if not gcode_text:
            return
        if not self.client or not self.client.is_connected:
            raise RuntimeError("Not connected to BLE device")

        # Normalize line endings, keep only non-empty & non-comment lines
        lines = [
            ln.strip()
            for ln in gcode_text.replace("\r\n", "\n").replace("\r", "\n").split("\n")
        ]
        lines = [ln for ln in lines if ln and not ln.startswith(";")]

        for line in lines:
            data = (line + "\n").encode("utf-8")  # ensure trailing \n
            self._run(self._async_send_packet(data))

    def send_lines(self, lines: Iterable[str]) -> None:
        """
        Send an iterable of already-formed G-code lines (str). Newlines appended automatically.
        """
        if not self.client or not self.client.is_connected:
            raise RuntimeError("Not connected to BLE device")
        for line in lines:
            payload = (line.rstrip("\r\n") + "\n").encode("utf-8")
            self._run(self._async_send_packet(payload))

    # ---------------- Async internals ----------------

    async def _async_connect(self):
        """
        Discover and connect to the target device; subscribe to TX notifications.
        """
        print("Scanning for BLE devices...")
        devices = await BleakScanner.discover(timeout=5.0)

        # Log discovered devices
        if devices:
            print("Discovered devices:")
            for d in devices:
                print(f"  - Name: {repr(d.name)} | Address: {d.address}")
        else:
            print("No BLE devices found.")

        target = None

        # First allow exact/partial name match
        for d in devices:
            name = d.name or ""
            if DEVICE_ADDR and d.address and d.address.lower() == DEVICE_ADDR.lower():
                target = d
                print(f"Selected by address: {d.address} ({name})")
                break
            if name and (name == self.device_name or self.device_name in name or name in self.device_name):
                target = d
                print(f"Selected by name: {name} @ {d.address}")
                break

        if not target:
            raise RuntimeError(f"Device '{self.device_name}' not found.")

        print(f"Connecting to {target.name} ({target.address})...")
        self.client = BleakClient(target)
        await self.client.connect()

        if not self.client.is_connected:
            raise RuntimeError("BLE connection failed")

        # Subscribe to notifications
        await self.client.start_notify(TX_CHAR_UUID, self._on_notify)
        # Small delay to let CCCD settle on some stacks
        await asyncio.sleep(0.4)

        print("BLE connected and notifications started.")

    async def _async_close(self):
        if self.client and self.client.is_connected:
            try:
                await self.client.stop_notify(TX_CHAR_UUID)
            except Exception:
                pass
            try:
                await self.client.disconnect()
            except Exception:
                pass
            print("BLE disconnected.")

    def _next_pid(self) -> int:
        self.packet_id_counter = (self.packet_id_counter + 1) % 256
        return self.packet_id_counter

    def _on_notify(self, _: int, data: bytearray):
        """
        Expected firmware ACK format:
          [pid_byte][utf8 message...], where message is e.g. "ok"
        """
        if not data:
            self.last_received_packet_id = None
            self.last_received_message = None
        else:
            self.last_received_packet_id = int(data[0])
            try:
                msg = data[1:].decode("utf-8", errors="ignore")
            except Exception:
                msg = ""
            self.last_received_message = "".join(c for c in msg if c.isprintable()).strip()
        self.response_received = True

    async def _async_send_packet(self, payload: bytes):
        """
        Send a single packet with retry-until-ACK behavior.
        ACK condition: last_received_packet_id == packet_id and last_received_message.lower() == "ok"
        """
        if not self.client or not self.client.is_connected:
            raise RuntimeError("BLE not connected")

        packet_id = self._next_pid()
        packet = bytes([packet_id]) + payload
        expected = "ok"

        while True:
            try:
                if not self.client.is_connected:
                    raise RuntimeError("BLE connection lost")

                # Important for macOS/CoreBluetooth + NUS: write without response
                await self.client.write_gatt_char(RX_CHAR_UUID, packet, response=False)
                t0 = time.time()
                self.response_received = False

                print(f"SEND::SUCCESS pid:{packet_id} cmd:{payload.decode(errors='ignore').strip()}")

                # Wait for ACK in the notification callback
                while not self.response_received and (time.time() - t0) < self.timeout:
                    await asyncio.sleep(0.01)

                if (
                    self.last_received_packet_id == packet_id
                    and (self.last_received_message or "").lower() == expected
                ):
                    print(f"RECEIVE::SUCCESS pid:{packet_id} msg:'{self.last_received_message}'")
                    return
                else:
                    print(
                        f"RECEIVE::FAIL pid:{packet_id} nack:'{self.last_received_message}' "
                        f"(expected '{expected}')"
                    )
                    await asyncio.sleep(RETRY_SLEEP_S)
                    # retry
            except Exception as e:
                print(f"BLE Error sending packet {packet_id}: {e}")
                # Propagate hard failures that usually indicate unsupported write/transport loss
                if "not supported" in str(e).lower() or "connection" in str(e).lower():
                    raise
                await asyncio.sleep(ERROR_COOLDOWN_S)