"""
BLE Packet Communication Library for Doodle Bot
"""

import asyncio
import os
import time
import threading
from typing import Optional

from bleak import BleakClient, BleakScanner  # pip install bleak

# Nordic UART UUIDs
RX_CHAR_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # Write
TX_CHAR_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # Notify

# Device targeting
DEVICE_NAME = os.getenv("BT_DEVICE_NAME", "BOO").strip()


class BTLink:
    """
    Handles reliable packet transmission over BLE.
    Simplified to match packetlib.py reference implementation.
    """

    def __init__(self, device_name: str = DEVICE_NAME):
        self.device_name = device_name
        self.packet_id_counter = -1
        self.response_received = False
        self.last_received_packet_id: Optional[int] = None
        self.last_received_message: Optional[str] = None
        self.client: Optional[BleakClient] = None
        self.timeout: float = 2.0
        
        # Threading for async event loop
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._thread: Optional[threading.Thread] = None
        self._ready_evt: Optional[threading.Event] = None

    def get_next_packet_id(self) -> int:
        """Generate sequential packet ID with automatic wraparound at 255."""
        self.packet_id_counter = (self.packet_id_counter + 1) % 256
        return self.packet_id_counter

    def create_packet_with_id(self, command_bytes: bytes) -> bytes:
        """Prepend packet ID to command bytes for transmission."""
        packet_id = self.get_next_packet_id()
        return bytes([packet_id]) + command_bytes

    # ---------------- Public API ----------------


    def connect(self) -> None:
        """Scan, connect, and start notifications (blocking until connected)."""
        self._start_loop()
        self._run(self._async_connect())

    def close(self) -> None:
        """Stop notifications, disconnect, and stop the loop."""
        try:
            self._run(self._async_close())
        finally:
            self._stop_loop()

    def disconnect(self) -> None:
        """Alias for close() to match packetlib.py API."""
        self.close()


    def send_gcode(self, gcode_str: str) -> None:
        """
        Send G-code string line-by-line with packet IDs and retry logic.
        Matches packetlib.py send_packets() behavior.
        """
        if not gcode_str or not self.client or not self.client.is_connected:
            return

        lines = [
            ln.rstrip()
            for ln in gcode_str.replace("\r\n", "\n").replace("\r", "\n").splitlines()
            if ln.strip()
        ]
        if not lines:
            return

        for line in lines:
            command_bytes = line.encode("utf-8")
            if not command_bytes.endswith(b'\n'):
                command_bytes += b'\n'
            self._run(self.send_packet(command_bytes))


    async def send_packet(self, command_bytes: bytes):
        """
        Send a packet with retry logic until successful acknowledgment.
        Matches packetlib.py send_packet() behavior exactly.
        """
        expected_response = "ok"

        if not self.client or not self.client.is_connected:
            raise RuntimeError("Not connected to BLE device")
            
        packet_with_id = self.create_packet_with_id(command_bytes)
        packet_id = packet_with_id[0]
        self.response_received = False
        
        while True:
            try:
                if not self.client.is_connected:
                    raise RuntimeError("BLE connection lost")
                
                await self.client.write_gatt_char(RX_CHAR_UUID, packet_with_id)
                start_wait = time.time()
                print(f"SEND::SUCCESS pid:{packet_id} cmd:{command_bytes.decode().strip()}")
                
                while not self.response_received and (time.time() - start_wait) < self.timeout:
                    await asyncio.sleep(0.01)
                
                if self.last_received_packet_id == packet_id and self.last_received_message == expected_response:
                    print(f"RECEIVE::SUCCESS: pid: {packet_id} msg '{self.last_received_message}'")
                    break
                else:
                    print(f"RECEIVE::FAIL: pid: {packet_id} nack '{self.last_received_message}' (expected '{expected_response}')")
                    self.response_received = False
                    await asyncio.sleep(0.1)
                    
            except Exception as e:
                print(f"BLE Error sending packet {packet_id}: {e}")
                if "not supported" in str(e) or "connection" in str(e).lower():
                    raise RuntimeError(f"Critical BLE error: {e}")
                await asyncio.sleep(0.5)

    async def send_packets(self, all_commands):
        """
        Send multiple packets in sequence.
        """
        for command in all_commands:
            await self.send_packet(command)

    # ---------------- Internals ----------------

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
        """
        Discover and connect to the target BLE device.
        """
        print("Scanning for BLE devices...")
        devices = await BleakScanner.discover()
        target = None
        for d in devices:
            if d.name and self.device_name in d.name:
                target = d
                break

        if not target:
            print(f"Device '{self.device_name}' not found.")
            raise RuntimeError(f"Device '{self.device_name}' not found.")

        print(f"Connecting to {target.name} ({target.address})...")
        self.client = BleakClient(target)
        await self.client.connect()
        
        if not self.client.is_connected:
            print("Connection failed.")
            raise RuntimeError("Connection failed.")
            
        print("Connected.")
        await self.client.start_notify(TX_CHAR_UUID, self._handle_rx_with_timing)
        await asyncio.sleep(0.5)
        


    async def _async_close(self):
        """Clean up BLE connection and notifications."""
        if self.client and self.client.is_connected:
            await self.client.stop_notify(TX_CHAR_UUID)
            await self.client.disconnect()
            print("Disconnected.")

    def _handle_rx_with_timing(self, _, data: bytearray):
        """
        Process incoming acknowledgment packets from the device.
        """
        if len(data) > 0:
            self.last_received_packet_id = int(data[0])
            raw_message = data[1:].decode(errors='ignore')
            self.last_received_message = ''.join(c for c in raw_message if c.isprintable()).strip()
        else:
            self.last_received_packet_id = None
            self.last_received_message = None
        
        self.response_received = True
