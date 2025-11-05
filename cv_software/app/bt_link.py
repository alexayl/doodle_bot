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
from typing import Optional, Iterable, List, Tuple, Callable, Sequence
import threading
from collections import deque

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

EXEC_ACK_SUCCESS = ["ok"]
EXEC_ACK_FAIL = [s.strip().lower() for s in (os.getenv("BT_EXEC_ACK_FAIL", "fail,error,nack") or "fail").split(",") if s.strip()]

# Write behavior: auto-detect characteristic capability; add a pace to avoid RX overflow
TX_PACE_S = float(os.getenv("BT_TX_PACE_S", "0.100"))  # seconds, configurable via env (increased to 100ms for write-without-response reliability)

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

    # Notification log (pid, message, timestamp)
        self._notif_log: deque[Tuple[Optional[int], str, float]] = deque(maxlen=512)
        self._notif_lock = threading.Lock()

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
            if line.strip().upper().startswith("G91"):
                line = "G91"
            if line.strip().upper().startswith("G90"):
                line = "G90"

            data = (line + "\n").encode("utf-8")
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

        # Determine write properties for RX characteristic
        # Force write-without-response since write-with-response consistently fails after first packet
        self._use_write_with_response = False
        try:
            svcs = await self.client.get_services()
            ch = svcs.get_characteristic(RX_CHAR_UUID)
            props = set(getattr(ch, "properties", []) or []) if ch else set()
            can_write = ("write" in props)
            can_wo = ("write-without-response" in props) or ("write-without" in props)
            
            # Always prefer write-without-response for Nordic UART - write-with-response fails intermittently
            if can_wo:
                self._use_write_with_response = False
                print(f"BLE connected. RX props={list(props)} - Using write-without-response for reliability")
            elif can_write:
                self._use_write_with_response = True
                print(f"BLE connected. RX props={list(props)} - Using write-with-response (write-without not available)")
            else:
                print(f"BLE connected. RX props={list(props)} - No write properties found, defaulting to write-without-response")
        except Exception as e:
            print(f"BLE connected and notifications started. (write props unknown: {e}; using write-without-response)")
        
        # Give BLE stack extra time to stabilize after connection
        print("Waiting 2s for BLE connection to stabilize...")
        await asyncio.sleep(2.0)

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
        # record notification
        try:
            with self._notif_lock:
                self._notif_log.append((self.last_received_packet_id, (self.last_received_message or "").strip().lower(), time.time()))
        except Exception:
            pass

    async def _async_send_packet(self, payload: bytes) -> int:
        """
        Send a single packet with retry-until-ACK behavior.

        Resync rule: if device replies 'ok' but PID differs, treat as success
        and snap our counter to the device's PID to realign.
        """
        if not self.client or not self.client.is_connected:
            raise RuntimeError("BLE not connected")

        packet_id = self._next_pid()
        packet = bytes([packet_id]) + payload
        expected = None  # We do not wait for receive-level ACK; only execution-level ACK is used

        while True:
            try:
                if not self.client.is_connected:
                    raise RuntimeError("BLE connection lost")

                # Choose write mode based on characteristic capability
                use_resp = bool(getattr(self, "_use_write_with_response", True))
                print("payload:", payload)
                await self.client.write_gatt_char(RX_CHAR_UUID, packet, response=use_resp)
                
                # Add pacing delay after write
                # Write-without-response needs longer delays since there's no flow control
                delay = TX_PACE_S
                if not use_resp:
                    # Double the delay for write-without-response to give firmware BLE stack time
                    delay = TX_PACE_S * 2.0
                
                if delay > 0:
                    await asyncio.sleep(delay)
                    
                print(f"SEND::SUCCESS pid:{packet_id} cmd:{payload.decode(errors='ignore').strip()}")

                if expected is None:
                    return packet_id
                # We don't wait for receive-level ACK here
                return packet_id

            except Exception as e:
                print(f"BLE Error sending packet {packet_id}: {e}")
                print(packet)
                emsg = str(e).lower()
                # If write-with-response is not supported, fall back to without-response and retry once
                if "not supported" in emsg:
                    prev = bool(getattr(self, "_use_write_with_response", True))
                    if prev:
                        self._use_write_with_response = False
                        print("BLE write-with-response unsupported; switching to write-without-response and retrying...")
                        # Give firmware significant time to drain its command queue before switching modes
                        # Write-without-response has no flow control, so we need the queue empty
                        print(f"Waiting 5s for firmware to drain command queue after mode switch...")
                        await asyncio.sleep(5.0)
                        continue
                    else:
                        # Already using without-response; give up
                        raise
                if "connection" in emsg:
                    raise
                await asyncio.sleep(ERROR_COOLDOWN_S)


    def _wait_for_exec_ack(self, pid: int, expected: Optional[Sequence[str]] = None, timeout: Optional[float] = None) -> None:
        """Block until we see a notification with (pid, one of expected). Also raises on explicit fail tokens."""
        expected_list: List[str] = list(expected) if expected else EXEC_ACK_SUCCESS
        timeout = timeout or float(os.getenv("BT_EXEC_TIMEOUT_S", "30.0"))
        t0 = time.time()
        start_idx = 0
        last_log_time = t0
        while True:
            # scan notif log
            with self._notif_lock:
                entries = list(self._notif_log)
            for p, msg, ts in entries[start_idx:]:
                # allow None pid? then skip
                if p is not None and int(p) == int(pid):
                    if msg in expected_list:
                        print(f"EXEC::DONE pid:{pid} msg:'{msg}'")
                        return
                    if msg in EXEC_ACK_FAIL:
                        raise RuntimeError(f"Execution NACK for pid {pid}: '{msg}'")
            start_idx = max(0, len(entries) - 4)  # keep scanning near tail
            elapsed = time.time() - t0
            if elapsed > timeout:
                # Show recent notifications before timeout
                recent = [(p, msg) for p, msg, ts in entries[-10:]]
                print(f"EXEC::TIMEOUT pid:{pid} after {elapsed:.1f}s. Recent notifications: {recent}")
                raise TimeoutError(f"Timed out waiting for exec ack {expected_list} on pid {pid}")
            # Log every 5 seconds while waiting
            if elapsed - (last_log_time - t0) >= 5.0:
                print(f"EXEC::WAITING pid:{pid} for {elapsed:.1f}s...")
                last_log_time = time.time()
            time.sleep(0.02)

    def send_gcode_windowed(
        self,
        gcode_text: str,
        window_size: Optional[int] = None,
        exec_timeout_s: Optional[float] = None,
        on_head_executed: Optional[Callable[[int, str, Optional[str]], Iterable[str]]] = None,
    ) -> None:
        """
        Send G-code with a sliding window. Maintains up to `window_size` lines in-flight.
        After seeding the window, waits for EXEC-ACK of the first PID, then slides by one.

    Assumptions:
    - Receive-level ACK is not used (we don't wait per-receive)
    - Firmware sends an execution-complete notify with same PID and message in exec_ack list (default 'ok')
    - Only motion lines (G0/G1 with X/Y) produce execution-complete ACK; non-motion lines (G90/G91/M-codes) are not waited on
        """
        if not gcode_text:
            return
        if not self.client or not self.client.is_connected:
            raise RuntimeError("Not connected to BLE device")

        win = int(window_size or 5)
        exec_expected = EXEC_ACK_SUCCESS
        if win < 1:
            win = 1

        lines = [
            ln.strip()
            for ln in gcode_text.replace("\r\n", "\n").replace("\r", "\n").split("\n")
        ]
        lines = [ln for ln in lines if ln and not ln.startswith(";")]
        
        # No additional filtering needed - convert_pathfinding_gcode already removed G90/G91
        # and ensured all commands are firmware-compatible

        print(f"DEBUG: Total lines after filtering: {len(lines)}")
        print(f"DEBUG: First 3 lines: {lines[:3] if len(lines) > 3 else lines}")
        print(f"DEBUG: Last 3 lines: {lines[-3:] if len(lines) > 3 else lines}")

        def send_one(line: str) -> int:
            # No normalization needed - lines are already filtered
            data = (line + "\n").encode("utf-8")
            pid = self._run(self._async_send_packet(data))
            print(f"SEND::WINDOWED pid:{pid} line:'{line}'")
            # Add extra delay after each send to ensure firmware processes before next
            time.sleep(0.5)  # 500ms additional delay between commands
            return int(pid)

        def is_motion(line: Optional[str]) -> bool:
            if not line:
                return False
            u = line.strip().upper()
            if not (u.startswith("G0") or u.startswith("G1")):
                return False
            return ("X" in u) or ("Y" in u)

        inflight: deque[Tuple[int, str]] = deque()
        i = 0

        # Seed the window
        while i < len(lines) and len(inflight) < win:
            pid = send_one(lines[i])
            inflight.append((pid, lines[i]))
            i += 1

        # Slide the window
        while inflight:
            head_pid, head_line = inflight[0]
            # Wait for execution completion for motion lines
            # For non-motion commands that may send ACKs (M280, etc.), give them time to respond
            if is_motion(head_line):
                try:
                    self._wait_for_exec_ack(head_pid, expected=exec_expected, timeout=exec_timeout_s)
                except RuntimeError as e:
                    msg = str(e)
                    if "Execution NACK" in msg:
                        raise RuntimeError(f"{msg} | line: '{head_line}'")
                    raise
            else:
                # Non-motion command: wait briefly for potential ACK (servo, etc.)
                # This provides natural pacing and ensures firmware isn't overwhelmed
                head_upper = head_line.strip().upper()
                if head_upper.startswith("M280"):
                    # Servo command - wait for ACK
                    try:
                        self._wait_for_exec_ack(head_pid, expected=exec_expected, timeout=5.0)
                    except TimeoutError:
                        # Servo might not ACK, that's okay - just pace the sending
                        print(f"EXEC::SERVO pid:{head_pid} no ACK (continuing)")
                        pass
                else:
                    # Other non-motion (G91, etc.) - add significant delay for pacing
                    print(f"EXEC::PACING pid:{head_pid} non-motion, waiting 1.0s...")
                    time.sleep(1.0)  # 1000ms pause to let firmware fully process
            # Slide window: remove head first to free a slot
            inflight.popleft()
            # Optionally inject correction lines after this head is executed, but don't exceed window
            if on_head_executed is not None:
                try:
                    next_line = lines[i] if i < len(lines) else None
                    extra_lines = list(on_head_executed(head_pid, head_line, next_line) or [])
                except Exception as e:
                    print(f"WARN: on_head_executed callback error: {e}")
                    extra_lines = []
                for el in extra_lines:
                    if len(inflight) >= win:
                        break
                    pid = send_one(el)
                    inflight.append((pid, el))
            # Send next path line to keep window full
            while i < len(lines) and len(inflight) < win:
                pid = send_one(lines[i])
                inflight.append((pid, lines[i]))
                i += 1
        print(f"SEND::WINDOWED complete - sent {i} of {len(lines)} lines")