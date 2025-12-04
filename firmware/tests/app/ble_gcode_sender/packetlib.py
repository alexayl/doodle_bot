"""
BLE Packet Communication Library for Doodle Bot
"""

import asyncio
import sys
import time
from bleak import BleakClient, BleakScanner
from typing import Optional

RX_CHAR_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
TX_CHAR_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

DEVICE_NAME = "DOO"

class BLEPacketHandler:
    """
    Handles reliable packet transmission.
    """
    
    def __init__(self, device_name: str = DEVICE_NAME):
        self.device_name = device_name
        self.packet_id_counter = -1
        self.response_received = False
        self.last_received_packet_id: Optional[int] = None
        self.last_received_message: Optional[str] = None
        self.client: Optional[BleakClient] = None
        self.timeout: float = 60.0  # Increased to 60 seconds for large movements
        self.ack_event: Optional[asyncio.Event] = None
        self.pending_acks: dict = {}  # packet_id -> (command_index, command_bytes, send_time)
        
    def get_next_packet_id(self) -> int:
        """Generate sequential packet ID with automatic wraparound at 255."""
        self.packet_id_counter = (self.packet_id_counter + 1) % 256
        return self.packet_id_counter

    def create_packet_with_id(self, command_bytes: bytes) -> bytes:
        """Prepend packet ID to command bytes for transmission."""
        packet_id = self.get_next_packet_id()
        return bytes([packet_id]) + command_bytes



    async def connect(self) -> bool:
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
            return False

        print(f"Connecting to {target.name} ({target.address})...")
        self.client = BleakClient(target)
        await self.client.connect()
        
        if not self.client.is_connected:
            print("Connection failed.")
            return False
            
        print("Connected.")
        await self.client.start_notify(TX_CHAR_UUID, self._handle_rx_with_timing)
        await asyncio.sleep(0.5)
        return True

    async def disconnect(self):
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
        
        # Signal windowed sender if active
        if self.ack_event:
            self.ack_event.set()

    async def send_packet(self, command_bytes: bytes, max_retries: int = 3):
        """
        Send a packet with retry logic until successful acknowledgment.
        """
        expected_response = "ok"

        if not self.client or not self.client.is_connected:
            raise RuntimeError("Not connected to BLE device")
            
        packet_with_id = self.create_packet_with_id(command_bytes)
        packet_id = packet_with_id[0]
        self.response_received = False
        
        for attempt in range(max_retries):
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
                    return True
                else:
                    print(f"RECEIVE::FAIL: pid: {packet_id} nack '{self.last_received_message}' (attempt {attempt + 1}/{max_retries})")
                    self.response_received = False
                    await asyncio.sleep(0.1)
                    
            except Exception as e:
                print(f"BLE Error sending packet {packet_id}: {e}")
                if "not supported" in str(e) or "connection" in str(e).lower():
                    raise RuntimeError(f"Critical BLE error: {e}")
                await asyncio.sleep(0.5)
        
        print(f"SEND::FAILED: pid:{packet_id} after {max_retries} attempts")
        return False
        
        

    async def send_packets(self, all_commands, window_size: int = 5):
        """
        Send multiple packets with a sliding window approach.
        Keeps up to window_size commands in flight at a time.
        """
        if not self.client or not self.client.is_connected:
            raise RuntimeError("Not connected to BLE device")
        
        total_commands = len(all_commands)
        next_to_send = 0  # Index of next command to send
        self.pending_acks = {}  # packet_id -> (command_index, command_bytes)
        completed = 0
        
        # Create event for ACK signaling
        self.ack_event = asyncio.Event()
        
        try:
            while completed < total_commands:
                # Send commands up to window size
                while len(self.pending_acks) < window_size and next_to_send < total_commands:
                    command = all_commands[next_to_send]
                    packet_with_id = self.create_packet_with_id(command)
                    packet_id = packet_with_id[0]
                    
                    await self.client.write_gatt_char(RX_CHAR_UUID, packet_with_id)
                    print(f"SEND pid:{packet_id} cmd:{command.decode().strip()} [{next_to_send+1}/{total_commands}] (in-flight: {len(self.pending_acks)+1})")
                    
                    self.pending_acks[packet_id] = (next_to_send, command)
                    next_to_send += 1
                
                # Wait for an ACK
                self.ack_event.clear()
                try:
                    await asyncio.wait_for(self.ack_event.wait(), timeout=self.timeout)
                except asyncio.TimeoutError:
                    print(f"Timeout waiting for ACK, {len(self.pending_acks)} commands pending")
                    break
                
                # Check if we got a valid ACK for a pending command
                if self.last_received_packet_id in self.pending_acks:
                    if self.last_received_message == "ok":
                        idx, cmd = self.pending_acks.pop(self.last_received_packet_id)
                        completed += 1
                        print(f"ACK pid:{self.last_received_packet_id} [{completed}/{total_commands}] (in-flight: {len(self.pending_acks)})")
                    else:
                        print(f"NACK pid:{self.last_received_packet_id} msg:'{self.last_received_message}'")
                        # Remove from pending and count as completed (or could retry)
                        self.pending_acks.pop(self.last_received_packet_id, None)
                        completed += 1
                else:
                    # ACK for unknown packet, might be duplicate
                    print(f"ACK for unknown pid:{self.last_received_packet_id}")
                    
        finally:
            self.ack_event = None
            self.pending_acks = {}