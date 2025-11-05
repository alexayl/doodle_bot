# app/bt_link.py
"""
BLE Packet Communication Library for Doodle Bot
- Scans for DEVICE_NAME
- Connects and subscribes to TX notifications
- Sends line-by-line with packet IDs

New policy:
- ACK/NACK is emitted upon stepper movement completion (not BLE parse).
- Host should only queue N instructions ahead (sliding window). Default: 5.
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
# NOTE: DEFAULT_TIMEOUT_S applies to write/connection errors only; ACK is asynchronous.
DEFAULT_TIMEOUT_S = float(os.getenv("BT_TIMEOUT_S", "2.0"))
RETRY_SLEEP_S = float(os.getenv("BT_RETRY_SLEEP_S", "0.10"))
ERROR_COOLDOWN_S = float(os.getenv("BT_ERROR_COOLDOWN_S", "0.50"))

# Max number of in-flight instructions (movement-completion ACK controlled)
# Firmware needs at least 10 commands queued to avoid running out during motion.
# This accounts for BLE transmission latency between host send and firmware receive.
# Can be tuned via BT_MAX_INFLIGHT environment variable.
try:
    _max_inflight_env = int(os.getenv("BT_MAX_INFLIGHT", "7"))
except Exception:
    _max_inflight_env = 7
MAX_INFLIGHT = max(1, _max_inflight_env)

# Maximum time to wait for a motion credit to return (movement ACK) before failing
MOTION_ACK_TIMEOUT_S = float(os.getenv("BT_MOTION_ACK_TIMEOUT_S", "20.0"))

# Delay after servo commands to allow physical movement
SERVO_COMMAND_DELAY_S = float(os.getenv("BT_SERVO_COMMAND_DELAY_S", "0.05"))

# Optional micro-pacing for BLE writes (seconds). Useful on platforms that don't
# support write-with-response for the RX characteristic and may drop writes when flooded.
# If unset, we'll auto-tune a tiny delay only when write-with-response is unsupported.
BLE_WRITE_PAUSE_S_ENV = os.getenv("BT_BLE_WRITE_PAUSE_S", "")

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
        self.response_received: bool = False  # legacy; maintained for logs only
        self.last_received_packet_id: Optional[int] = None
        self.last_received_message: Optional[str] = None

        # Flow control (host-side sliding window)
        # Acquire a credit before sending a movement instruction; release on ACK.
        # Use threading.Semaphore to bridge async callback and sync API safely.
        self._credits = threading.Semaphore(max(1, MAX_INFLIGHT))
        self._error_lock = threading.Lock()
        self._last_error: Optional[str] = None
        self._inflight_lock = threading.Lock()
        self._inflight_is_motion: dict[int, bool] = {}
    # Store payloads for potential resend on NACK
        self._inflight_payload: dict[int, bytes] = {}
        # Track if we've already retried a given PID once
        self._resent_once: set[int] = set()
        # Pause new sends when a NACK occurs until the failing PID is resolved
        self._send_paused: threading.Event = threading.Event()
        self._blocked_pid: Optional[int] = None

    # Timeouts
        self.timeout: float = DEFAULT_TIMEOUT_S
        self.motion_ack_timeout: float = MOTION_ACK_TIMEOUT_S

        # Background loop
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._thread: Optional[threading.Thread] = None
        self._ready_evt: Optional[threading.Event] = None

    # BLE write capability flags (set after connect)
        self._rx_supports_write_with_resp: bool = False
        self._rx_supports_write_wo_resp: bool = True  # NUS typically supports this
        # Micro-pause after write when using write-without-response only
        self._ble_write_pause: float = 0.0

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
        Send a G-code string line by line with packet IDs.
        - Ignores blank lines and pure comment lines
        - Uses a sliding window: only N movement instructions ahead (default 5)
        - ACK/NACK is handled asynchronously from notifications
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

        # Reset any prior error before a new transfer
        with self._error_lock:
            self._last_error = None


        for line in lines:
            # If paused due to a prior NACK, wait until it's cleared
            while self._send_paused.is_set():
                time.sleep(0.001)
            u = line.strip().upper()

            # Normalize common modality commands for stability
            if u.startswith("G91"):
                line = "G91"
                u = "G91"
            elif u.startswith("G90"):
                line = "G90"
                u = "G90"

            # Only throttle on movement commands (G0/G1). Other commands
            # (e.g., G90/G91/M-codes) don't generate stepper ACKs and shouldn't
            # consume window credits.
            is_motion = u.startswith("G0") or u.startswith("G1")
            if is_motion:
                # Block if window full until an ACK releases credit
                acquired = self._credits.acquire(timeout=self.motion_ack_timeout)
                if not acquired:
                    raise TimeoutError(
                        f"Timed out waiting for movement ACK to free credit (>{self.motion_ack_timeout:.1f}s)."
                    )
            # Non-motion commands (M280, G91, etc.) don't need credits

            # Propagate any async failure prior to sending the next line
            with self._error_lock:
                if self._last_error:
                    # Release the credit we just acquired (if motion)
                    if is_motion:
                        self._credits.release()
                    raise RuntimeError(f"BLE/MCU error: {self._last_error}")

            data = (line + "\n").encode("utf-8")
            try:
                self._run(self._async_write_packet(data, is_motion=is_motion))
            except Exception as e:
                # On immediate write failure, if we consumed a credit for motion, return it
                if is_motion:
                    try:
                        self._credits.release()
                    except Exception:
                        pass
                raise
            # No finally block needed - non-motion never acquired a credit
            
            # Brief delay after servo commands for physical movement
            if u.startswith("M"):
                time.sleep(SERVO_COMMAND_DELAY_S)

    def send_lines(self, lines: Iterable[str]) -> None:
        """
        Send an iterable of already-formed G-code lines (str). Newlines appended automatically.
        Uses the same sliding-window behavior as send_gcode.
        """
        if not self.client or not self.client.is_connected:
            raise RuntimeError("Not connected to BLE device")
        with self._error_lock:
            self._last_error = None
        for line in lines:
            s = line.rstrip("\r\n")
            u = s.strip().upper()
            # If paused due to a prior NACK, wait until it's cleared
            while self._send_paused.is_set():
                time.sleep(0.001)
            is_motion = u.startswith("G0") or u.startswith("G1")
            if is_motion:
                acquired = self._credits.acquire(timeout=self.motion_ack_timeout)
                if not acquired:
                    raise TimeoutError(
                        f"Timed out waiting for movement ACK to free credit (>{self.motion_ack_timeout:.1f}s)."
                    )
            # Non-motion commands don't need credits
            
            with self._error_lock:
                if self._last_error:
                    if is_motion:
                        try:
                            self._credits.release()
                        except Exception:
                            pass
                    raise RuntimeError(f"BLE/MCU error: {self._last_error}")
            payload = (s + "\n").encode("utf-8")
            try:
                self._run(self._async_write_packet(payload, is_motion=is_motion))
            except Exception:
                if is_motion:
                    try:
                        self._credits.release()
                    except Exception:
                        pass
                raise
            # No finally block needed - non-motion never acquired a credit
            
            # Brief delay after servo commands for physical movement
            if u.startswith("M"):
                time.sleep(SERVO_COMMAND_DELAY_S)

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

        # Discover services/characteristics to determine write capabilities
        try:
            services = await self.client.get_services()
            rx_char = services.get_characteristic(RX_CHAR_UUID)
            props = set(rx_char.properties or []) if rx_char else set()
            # Bleak normalizes to strings like 'write', 'write-without-response'
            self._rx_supports_write_with_resp = ("write" in props)
            self._rx_supports_write_wo_resp = ("write-without-response" in props or "write-without-response" in props)
        except Exception:
            # Fallback assumptions for NUS
            self._rx_supports_write_with_resp = False
            self._rx_supports_write_wo_resp = True

        # Configure micro-pause only if write-with-response is unavailable
        if BLE_WRITE_PAUSE_S_ENV:
            try:
                self._ble_write_pause = max(0.0, float(BLE_WRITE_PAUSE_S_ENV))
            except Exception:
                self._ble_write_pause = 0.0
        else:
            # Auto-tune a very small pause when using write-without-response only
            self._ble_write_pause = 0.0015 if not self._rx_supports_write_with_resp else 0.0

        print(
            "BLE connected and notifications started. "
            f"RX props: write={'yes' if self._rx_supports_write_with_resp else 'no'}, "
            f"write-without-response={'yes' if self._rx_supports_write_wo_resp else 'no'}, "
            f"write-pause={self._ble_write_pause*1e3:.1f}ms"
        )

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
        Expected firmware notify format:
          [pid_byte][utf8 message...]
        Message semantics (examples):
          - "ok" or "ack"  → stepper movement completed
          - "nack" or "err" → stepper failed / rejected
        """
        if not data:
            self.last_received_packet_id = None
            self.last_received_message = None
            return

        self.last_received_packet_id = int(data[0])
        try:
            msg = data[1:].decode("utf-8", errors="ignore")
        except Exception:
            msg = ""
        msg_clean = "".join(c for c in msg if c.isprintable()).strip()
        self.last_received_message = msg_clean
        self.response_received = True  # for legacy tooling/logging

        m = (msg_clean or "").strip().lower()
        # Treat common success tokens as ACK
        if m in ("ok", "ack", "done", "complete"):
            released = False
            with self._inflight_lock:
                is_motion = self._inflight_is_motion.pop(self.last_received_packet_id, None)
                # Clear any stored payload and retry state on success
                self._inflight_payload.pop(self.last_received_packet_id, None)
                self._resent_once.discard(self.last_received_packet_id)
                # If this ACK clears a previously blocked PID, resume sending
                if self._blocked_pid is not None and self._blocked_pid == self.last_received_packet_id:
                    self._blocked_pid = None
                    self._send_paused.clear()
            if is_motion:
                try:
                    self._credits.release()
                    released = True
                except ValueError:
                    pass
            print(
                f"ACK::MOTION_DONE pid:{self.last_received_packet_id} msg:'{self.last_received_message}'"
                + (" (credit released)" if released else "")
            )
        elif m in ("nack", "err", "error", "fail", "failed"):
            pid = self.last_received_packet_id
            # Attempt a single automatic resend with the same PID to satisfy firmware sequencing
            do_resend = False
            payload: Optional[bytes] = None
            is_motion = False
            with self._inflight_lock:
                is_motion = bool(self._inflight_is_motion.get(pid))
                payload = self._inflight_payload.get(pid)
                if payload is not None and pid not in self._resent_once:
                    self._resent_once.add(pid)
                    do_resend = True
                # Pause further sending until this PID is resolved
                self._blocked_pid = pid
                self._send_paused.set()

            if do_resend and payload is not None and self._loop:
                # Schedule resend on the BLE loop; keep the credit held
                def _schedule():
                    asyncio.create_task(self._async_resend_packet(pid, payload))
                try:
                    self._loop.call_soon_threadsafe(_schedule)
                except Exception:
                    do_resend = False  # fallback to surfacing error

            if do_resend:
                print(f"ACK::MOTION_FAIL pid:{pid} msg:'{self.last_received_message}' -> RESEND queued")
                return
            # Give up after one retry: surface error and free credit to avoid deadlock
            with self._error_lock:
                self._last_error = f"NACK from device (pid {pid}) after retry: {msg_clean}"
            with self._inflight_lock:
                self._inflight_is_motion.pop(pid, None)
                self._inflight_payload.pop(pid, None)
                self._resent_once.discard(pid)
                # Keep paused; sender will see error and stop
            if is_motion:
                try:
                    self._credits.release()
                except Exception:
                    pass
            print(f"ACK::MOTION_FAIL pid:{pid} msg:'{self.last_received_message}' (fatal)")
        else:
            # Unknown message; log it but do not change credits
            print(f"ACK::UNKNOWN pid:{self.last_received_packet_id} msg:'{self.last_received_message}'")

    async def _async_write_packet(self, payload: bytes, is_motion: bool = False):
        """
        Send a single packet without waiting for ACK. ACK/NACK will arrive later via notify.
        """
        if not self.client or not self.client.is_connected:
            raise RuntimeError("BLE not connected")

        packet_id = self._next_pid()
        packet = bytes([packet_id]) + payload

        try:
            if not self.client.is_connected:
                raise RuntimeError("BLE connection lost")
            # Track this packet's motion nature for credit accounting on ACK
            with self._inflight_lock:
                self._inflight_is_motion[packet_id] = bool(is_motion)
                self._inflight_payload[packet_id] = payload
            # Prefer write-with-response if supported to ensure ATT-level ordering.
            wrote = False
            last_err: Optional[Exception] = None
            if self._rx_supports_write_with_resp:
                try:
                    await self.client.write_gatt_char(RX_CHAR_UUID, packet, response=True)
                    wrote = True
                except Exception as e:
                    # Some stacks return 'not supported' despite props; fall back
                    last_err = e

            if not wrote:
                # Fallback to write-without-response
                await self.client.write_gatt_char(RX_CHAR_UUID, packet, response=False)
                wrote = True
                # Optional tiny pause to avoid flooding OS/peripheral TX queue
                if self._ble_write_pause > 0:
                    await asyncio.sleep(self._ble_write_pause)
            print(f"SEND::QUEUED pid:{packet_id} cmd:{payload.decode(errors='ignore').strip()}")
        except Exception as e:
            print(f"BLE Error sending packet {packet_id}: {e}")
            if "not supported" in str(e).lower() or "connection" in str(e).lower():
                raise
            # Best-effort backoff before surfacing; the caller will decide to retry at higher level
            await asyncio.sleep(ERROR_COOLDOWN_S)
            # Clean up inflight record on failure
            with self._inflight_lock:
                self._inflight_is_motion.pop(packet_id, None)
                self._inflight_payload.pop(packet_id, None)
            raise

    async def _async_resend_packet(self, pid: int, payload: bytes):
        """Resend a packet with the same PID to satisfy firmware sequencing after a NACK."""
        if not self.client or not self.client.is_connected:
            return
        packet = bytes([pid]) + payload
        try:
            if self._rx_supports_write_with_resp:
                try:
                    await self.client.write_gatt_char(RX_CHAR_UUID, packet, response=True)
                except Exception:
                    await self.client.write_gatt_char(RX_CHAR_UUID, packet, response=False)
            else:
                await self.client.write_gatt_char(RX_CHAR_UUID, packet, response=False)
                if self._ble_write_pause > 0:
                    await asyncio.sleep(self._ble_write_pause)
            print(f"RESEND::QUEUED pid:{pid} cmd:{payload.decode(errors='ignore').strip()}")
        except Exception as e:
            print(f"BLE Error resending packet {pid}: {e}")