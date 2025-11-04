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
        self.timeout: float = 10.0
        
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

    async def send_packet(self, command_bytes: bytes):
        """
        Send a packet with retry logic until successful acknowledgment.
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