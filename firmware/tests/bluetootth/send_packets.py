import asyncio
import time
import struct
from bleak import BleakClient, BleakScanner
from typing import Optional, Callable, Dict, Any

# NUS UUIDs
NUS_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
RX_CHAR_UUID     = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # Write
TX_CHAR_UUID     = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # Notify

DEVICE_NAME = "DoodleBot"

all_commands = [
    b"G0 X20 Y90\n",
    b"G0 X30 Y80\n", 
    b"G0 X40 Y70\n",
    b"G0 X50 Y60\n",
    b"G0 X60 Y50\n",
    b"M280 P0 S90\n",  # Servo engage
    b"M280 P0 S90\n",  # Servo engage (repeat for consistency)
    b"M280 P0 S0\n",   # Servo disengage
    b"M280 P0 S0\n",   # Servo disengage (repeat for consistency)
]

class BLEPacketHandler:
    """Generic BLE packet sending and receiving class"""
    
    def __init__(self, device_name: str = DEVICE_NAME):
        self.device_name = device_name
        self.packet_id_counter = -1  # Start at -1 so first increment gives 0
        self.last_response_time: Optional[float] = None
        self.expected_response: Optional[str] = None
        self.response_received = False
        self.last_received_packet_id: Optional[int] = None
        self.last_successfully_sent_packet_id: Optional[int] = None
        self.client: Optional[BleakClient] = None
        self.response_callback: Optional[Callable] = None
        
    def get_next_packet_id(self) -> int:
        """Get the next packet ID and increment counter (wraps at 255)"""
        self.packet_id_counter = (self.packet_id_counter + 1) % 256
        return self.packet_id_counter

    def create_packet_with_id(self, command_bytes: bytes) -> bytes:
        """Create a packet with packet ID as the first byte"""
        packet_id = self.get_next_packet_id()
        return bytes([packet_id]) + command_bytes

    def rollback_packet_id(self):
        """Rollback packet ID counter when packet fails to send"""
        self.packet_id_counter = (self.packet_id_counter - 1) % 256
        
    def set_response_callback(self, callback: Callable[[int, str], None]):
        """Set custom callback for handling responses"""
        self.response_callback = callback

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
        return True

    async def disconnect(self):
        """Disconnect from the BLE device"""
        if self.client and self.client.is_connected:
            await self.client.stop_notify(TX_CHAR_UUID)
            await self.client.disconnect()
            print("Disconnected.")

    def _handle_rx_with_timing(self, _, data: bytearray):
        """Internal notification callback that tracks response timing"""
        self.last_response_time = time.time()
        self.response_received = True
        
        # Parse packet: first byte is packet ID, rest is message
        if len(data) > 0:
            packet_id = data[0]
            self.last_received_packet_id = packet_id
            message = data[1:].decode(errors='ignore').strip()
            print(f"RX -> Packet ID: {packet_id}, Message: '{message}'")
            
            # Call custom callback if set
            if self.response_callback:
                self.response_callback(packet_id, message)
            
            # Check if this matches expected response for timing tests
            if self.expected_response and self.expected_response in message:
                print(f"  ✅ Expected response received: {self.expected_response} (Packet ID: {packet_id})")
        else:
            print(f"RX -> Empty packet received")
            self.last_received_packet_id = None

    async def send_packet_with_response(self, command_bytes: bytes, expected_response: str = "ok", timeout: float = 2.0) -> Dict[str, Any]:
        """
        Send a packet and wait for response with timing measurement
        
        Returns:
            Dict containing timing info and success status
        """
        if not self.client or not self.client.is_connected:
            return {"success": False, "error": "Not connected"}
            
        # Create packet with ID
        packet_with_id = self.create_packet_with_id(command_bytes)
        packet_id = packet_with_id[0]
        
        # Reset response tracking
        self.response_received = False
        self.expected_response = expected_response
        
        # Record send time and send packet
        send_time = time.time()
        await self.client.write_gatt_char(RX_CHAR_UUID, packet_with_id)
        
        # Wait for response with timeout
        start_wait = time.time()
        while not self.response_received and (time.time() - start_wait) < timeout:
            await asyncio.sleep(0.01)
        
        result = {
            "success": self.response_received,
            "packet_id": packet_id,
            "send_time": send_time,
            "timeout": timeout
        }
        
        if self.response_received and self.last_response_time:
            rtt = (self.last_response_time - send_time) * 1000  # Convert to ms
            result.update({
                "response_time": self.last_response_time,
                "round_trip_time_ms": rtt,
                "estimated_one_way_ms": rtt / 2
            })
        else:
            result["error"] = f"No response received within {timeout}s"
            
        return result

    async def send_packet(self, command_bytes: bytes) -> int:
        """Send a packet without waiting for response, returns packet ID"""
        if not self.client or not self.client.is_connected:
            raise RuntimeError("Not connected")
            
        packet_with_id = self.create_packet_with_id(command_bytes)
        packet_id = packet_with_id[0]
        await self.client.write_gatt_char(RX_CHAR_UUID, packet_with_id)
        return packet_id

    async def send_packets(self, all_commands):
        


        # Separate measurement collections
        movement_rtt_measurements = []
        servo_engage_rtt_measurements = []
        servo_disengage_rtt_measurements = []
        all_rtt_measurements = []
        
        for i, command in enumerate(all_commands):
            # Determine command type for categorization
            command_str = command.decode().strip()
            if "M280 P0 S90" in command_str:
                command_type = "servo_engage"
                print(f"\n🔧 Servo Engage {i+1}: {command_str}")
            elif "M280 P0 S0" in command_str:
                command_type = "servo_disengage"
                print(f"\n🔧 Servo Disengage {i+1}: {command_str}")
            else:
                command_type = "movement"
                print(f"\n📐 Movement {i+1}: {command_str}")
            
            # Send command with packet ID and measure response time
            result = await self.send_packet_with_response(command, "ok", 2.0)
            
            if result["success"]:
                rtt = result["round_trip_time_ms"]
                one_way_estimate = result["estimated_one_way_ms"]
                
                # Add to appropriate measurement collection
                all_rtt_measurements.append(rtt)
                if command_type == "movement":
                    movement_rtt_measurements.append(rtt)
                elif command_type == "servo_engage":
                    servo_engage_rtt_measurements.append(rtt)
                elif command_type == "servo_disengage":
                    servo_disengage_rtt_measurements.append(rtt)
                
                print(f"  Packet ID: {result['packet_id']}")
                print(f"  Send time: {result['send_time']:.6f}s")
                print(f"  Response time: {result['response_time']:.6f}s")
                print(f"  Round-trip time: {rtt:.2f}ms")
                print(f"  Estimated one-way time: {one_way_estimate:.2f}ms")
            else:
                print(f"  ❌ {result.get('error', 'Unknown error')}")
            
            # Small delay between commands
            await asyncio.sleep(0.5)

async def main():
    # Create BLE packet handler
    packet_handler = BLEPacketHandler(DEVICE_NAME)
    try:
        # Connect to device
        if not await packet_handler.connect():
            return
        
        # Send packets
        await packet_handler.send_packets(all_commands)
                
        # Keep connection open briefly to ensure all responses are received
        await asyncio.sleep(2)
        
    finally:
        # Ensure successful disconnection
        await packet_handler.disconnect()

if __name__ == "__main__":
    asyncio.run(main())
