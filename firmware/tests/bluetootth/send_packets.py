import asyncio
import time
import argparse
import sys
from bleak import BleakClient, BleakScanner
from typing import Optional, List

# NUS UUIDs
RX_CHAR_UUID     = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # Write
TX_CHAR_UUID     = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # Notify

DEVICE_NAME = "DOO"

all_commands = [
    b"G1 X20 Y90\n",
    b"G1 X30 Y80\n", 
    b"G1 X40 Y70\n",
    b"G1 X50 Y60\n",
    b"G1 X20 Y90\n",
    b"G1 X30 Y80\n", 
    b"G1 X40 Y70\n",
    b"G1 X50 Y60\n",
    b"G1 X20 Y90\n",
    b"G1 X30 Y80\n", 
    b"G1 X40 Y70\n",
    b"G1 X50 Y60\n",
    b"G1 X20 Y90\n",
    b"G1 X30 Y80\n", 
    b"G1 X40 Y70\n",
    b"G1 X50 Y60\n",
    b"G1 X20 Y90\n",
    b"G1 X30 Y80\n", 
    b"G1 X40 Y70\n",
    b"G1 X50 Y60\n",
]

def load_gcode_file(filename: str) -> List[bytes]:
    """Load G-code commands from a file and return as list of bytes"""
    commands = []
    try:
        with open(filename, 'r') as file:
            for line in file:
                line = line.strip()
                # Skip empty lines and comments
                if line and not line.startswith(';'):
                    # Ensure line ends with newline
                    if not line.endswith('\n'):
                        line += '\n'
                    commands.append(line.encode())
        print(f"Loaded {len(commands)} commands from {filename}")
        return commands
    except FileNotFoundError:
        print(f"Error: File '{filename}' not found")
        sys.exit(1)
    except Exception as e:
        print(f"Error reading file '{filename}': {e}")
        sys.exit(1)

class BLEPacketHandler:
    """Generic BLE packet sending and receiving class"""
    
    def __init__(self, device_name: str = DEVICE_NAME):
        self.device_name = device_name
        self.packet_id_counter = -1  # Start at -1 so first increment gives 0
        self.response_received = False
        self.last_received_packet_id: Optional[int] = None
        self.last_received_message: Optional[str] = None
        self.client: Optional[BleakClient] = None
        self.timeout: float = 2.0  # Default timeout for responses
        
    def get_next_packet_id(self) -> int:
        """Get the next packet ID and increment counter (wraps at 255)"""
        self.packet_id_counter = (self.packet_id_counter + 1) % 256
        return self.packet_id_counter

    def create_packet_with_id(self, command_bytes: bytes) -> bytes:
        """Create a packet with packet ID as the first byte"""
        packet_id = self.get_next_packet_id()
        return bytes([packet_id]) + command_bytes



    async def connect(self) -> bool:
        """Connect to the BLE device"""
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
        # Small delay to ensure device is ready
        await asyncio.sleep(0.5)
        return True

    async def disconnect(self):
        """Disconnect from the BLE device"""
        if self.client and self.client.is_connected:
            await self.client.stop_notify(TX_CHAR_UUID)
            await self.client.disconnect()
            print("Disconnected.")

    def _handle_rx_with_timing(self, _, data: bytearray):
        """Handle incoming ack/nack for processing"""

        # Parse packet: first byte is packet ID, rest is message
        if len(data) > 0:
            self.last_received_packet_id = int(data[0])
            # More aggressive cleaning - remove all whitespace and control characters
            raw_message = data[1:].decode(errors='ignore')
            self.last_received_message = ''.join(c for c in raw_message if c.isprintable()).strip()
            print(f"RX -> Packet ID: {self.last_received_packet_id}, Message: '{self.last_received_message}' (cleaned from raw: {repr(raw_message)})")
        else:
            print(f"RX -> Empty packet received")
            self.last_received_packet_id = None
            self.last_received_message = None
        
        # Set flag for send to validate
        self.response_received = True

    async def send_packet(self, command_bytes: bytes):
        """
        Send a packet and retry indefinenitely until success
        """
        expected_response = "ok"

        if not self.client or not self.client.is_connected:
            raise RuntimeError("Not connected to BLE device")
            
        # Create packet with ID
        packet_with_id = self.create_packet_with_id(command_bytes)
        packet_id = packet_with_id[0]
        
        # Reset response tracking
        self.response_received = False
        
        while True:
            try:
                # Check connection before sending
                if not self.client.is_connected:
                    raise RuntimeError("BLE connection lost")
                
                # Record send time and send packet
                await self.client.write_gatt_char(RX_CHAR_UUID, packet_with_id)
                start_wait = time.time()
                print(f"SEND::SUCCESS pid:{packet_id} cmd:{command_bytes.decode().strip()}")
                
                # Wait for response with timeout
                while not self.response_received and (time.time() - start_wait) < self.timeout:
                    await asyncio.sleep(0.01)
                
                # Check if we got the expected response
                if self.last_received_packet_id == packet_id and self.last_received_message == expected_response:
                    print(f"RECEIVE::SUCCESS: pid: {packet_id} ack")
                    break
                else:
                    print(f"RECEIVE::FAIL: pid: {packet_id} nack '{self.last_received_message}' (expected '{expected_response}')")
                    self.response_received = False
                    await asyncio.sleep(0.1)  # Brief delay before retry
                    
            except Exception as e:
                print(f"BLE Error sending packet {packet_id}: {e}")
                # For critical errors, don't retry indefinitely
                if "not supported" in str(e) or "connection" in str(e).lower():
                    raise RuntimeError(f"Critical BLE error: {e}")
                await asyncio.sleep(0.5)  # Wait longer before retry on error
        
        

    async def send_packets(self, all_commands):
        total = len(all_commands)
        for i, command in enumerate(all_commands, 1):
            if total > 10:  # Show progress for larger files
                print(f"Progress: {i}/{total} ({i/total*100:.1f}%)")
            await self.send_packet(command)
            
            # Pause after every 10 commands and wait for user input
            if i % 100 == 0:
                print(f"\n--- Sent {i} commands ---")
                input("Press Enter to continue sending the next batch...")
                print("Continuing...\n")
        

async def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Send G-code commands via BLE')
    parser.add_argument('filename', nargs='?', help='G-code file to send (.gcode)')
    
    args = parser.parse_args()
    
    # Determine commands to send
    if args.filename:
        commands = load_gcode_file(args.filename)
    else:
        commands = all_commands
        print(f"No file specified, using default commands ({len(commands)} commands)")
    
    # Create BLE packet handler
    packet_handler = BLEPacketHandler(DEVICE_NAME)
    try:
        # Connect to device
        if not await packet_handler.connect():
            return
        
        # Send packets
        await packet_handler.send_packets(commands)
                
        # Keep connection open briefly to ensure all responses are received
        await asyncio.sleep(2)
        
    finally:
        # Ensure successful disconnection
        await packet_handler.disconnect()

if __name__ == "__main__":
    asyncio.run(main())
