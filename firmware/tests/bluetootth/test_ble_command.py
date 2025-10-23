import asyncio
import time
import struct
from bleak import BleakClient, BleakScanner

# NUS UUIDs
NUS_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
RX_CHAR_UUID     = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # Write
TX_CHAR_UUID     = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # Notify

DEVICE_NAME = "DoodleBot"

# Global variables for tracking responses
last_response_time = None
expected_response = None
response_received = False
last_received_packet_id = None
last_successfully_sent_packet_id = None  # Track last packet that was actually sent
packet_id_counter = -1  # Start at -1 so first increment gives 0

def get_next_packet_id():
    """Get the next packet ID and increment counter (wraps at 255)"""
    global packet_id_counter
    packet_id_counter = (packet_id_counter + 1) % 256
    return packet_id_counter

def create_packet_with_id(command_bytes):
    """Create a packet with packet ID as the first byte"""
    packet_id = get_next_packet_id()
    # Prepend packet ID as first byte
    return bytes([packet_id]) + command_bytes

def rollback_packet_id():
    """Rollback packet ID counter when packet fails to send"""
    global packet_id_counter
    packet_id_counter = (packet_id_counter - 1) % 256

async def test_1_round_trip_time(client):
    """
    Test 1: Measure round-trip time for BLE commands and acknowledgements

    This test sends commands and measures the time between sending and
    receiving acknowledgements to calculate one-way latency.
    """
    print("\n=== TEST 1: Round-Trip Time Measurement ===")
    
    global last_response_time, expected_response, response_received
    
    # Test commands to send - categorized by type
    movement_commands = [
        b"G0 X20 Y90\n",
        b"G0 X30 Y80\n", 
        b"G0 X40 Y70\n",
        b"G0 X50 Y60\n",
        b"G0 X60 Y50\n"
    ]
    
    servo_engage_commands = [
        b"M280 P0 S90\n",  # Servo engage
        b"M280 P0 S90\n",  # Servo engage (repeat for consistency)
    ]
    
    servo_disengage_commands = [
        b"M280 P0 S0\n",   # Servo disengage
        b"M280 P0 S0\n",   # Servo disengage (repeat for consistency)
    ]
    
    # Combine all commands for overall testing
    all_commands = movement_commands + servo_engage_commands + servo_disengage_commands
    
    # Separate measurement collections
    movement_rtt_measurements = []
    servo_engage_rtt_measurements = []
    servo_disengage_rtt_measurements = []
    all_rtt_measurements = []
    
    for i, command in enumerate(all_commands):
        # Create packet with ID as first byte
        packet_with_id = create_packet_with_id(command)
        packet_id = packet_with_id[0]
        
        # Determine command type for categorization
        command_str = command.decode().strip()
        if "M280 P0 S90" in command_str:
            command_type = "servo_engage"
            print(f"\n🔧 Servo Engage {i+1} (Packet ID: {packet_id}): {command_str}")
        elif "M280 P0 S0" in command_str:
            command_type = "servo_disengage"
            print(f"\n🔧 Servo Disengage {i+1} (Packet ID: {packet_id}): {command_str}")
        else:
            command_type = "movement"
            print(f"\n📐 Movement {i+1} (Packet ID: {packet_id}): {command_str}")
            
        print(f"  Full packet: {packet_with_id}")
        
        # Reset response tracking
        response_received = False
        expected_response = f"ok"  # Expected acknowledgement
        
        # Record send time
        send_time = time.time()
        
        # Send command with packet ID
        await client.write_gatt_char(RX_CHAR_UUID, packet_with_id)
        
        # Wait for response (with timeout)
        timeout = 2.0  # 2 second timeout
        start_wait = time.time()
        
        while not response_received and (time.time() - start_wait) < timeout:
            await asyncio.sleep(0.01)  # Small delay to prevent busy waiting
        
        if response_received and last_response_time:
            rtt = (last_response_time - send_time) * 1000  # Convert to ms
            one_way_estimate = rtt / 2
            
            # Add to appropriate measurement collection
            all_rtt_measurements.append(rtt)
            if command_type == "movement":
                movement_rtt_measurements.append(rtt)
            elif command_type == "servo_engage":
                servo_engage_rtt_measurements.append(rtt)
            elif command_type == "servo_disengage":
                servo_disengage_rtt_measurements.append(rtt)
            
            print(f"  Send time: {send_time:.6f}s")
            print(f"  Response time: {last_response_time:.6f}s")
            print(f"  Round-trip time: {rtt:.2f}ms")
            print(f"  Estimated one-way time: {one_way_estimate:.2f}ms")
        else:
            print(f"  ❌ No response received within {timeout}s")
        
        # Small delay between commands
        await asyncio.sleep(0.5)
    
    # Calculate and display statistics by command type
    print(f"\n📊 Round-Trip Time Statistics:")
    
    # Overall statistics
    if all_rtt_measurements:
        avg_rtt = sum(all_rtt_measurements) / len(all_rtt_measurements)
        min_rtt = min(all_rtt_measurements)
        max_rtt = max(all_rtt_measurements)
        
        print(f"\n� Overall RTT:")
        print(f"  Average RTT: {avg_rtt:.2f}ms (one-way ≈ {avg_rtt/2:.2f}ms)")
        print(f"  Min RTT: {min_rtt:.2f}ms (one-way ≈ {min_rtt/2:.2f}ms)")
        print(f"  Max RTT: {max_rtt:.2f}ms (one-way ≈ {max_rtt/2:.2f}ms)")
        print(f"  Total measurements: {len(all_rtt_measurements)}/{len(all_commands)}")
    
    # Movement command statistics
    if movement_rtt_measurements:
        avg_mov = sum(movement_rtt_measurements) / len(movement_rtt_measurements)
        min_mov = min(movement_rtt_measurements)
        max_mov = max(movement_rtt_measurements)
        
        print(f"\n📐 Movement Commands (G0):")
        print(f"  Average RTT: {avg_mov:.2f}ms (one-way ≈ {avg_mov/2:.2f}ms)")
        print(f"  Min RTT: {min_mov:.2f}ms (one-way ≈ {min_mov/2:.2f}ms)")
        print(f"  Max RTT: {max_mov:.2f}ms (one-way ≈ {max_mov/2:.2f}ms)")
        print(f"  Measurements: {len(movement_rtt_measurements)}/{len(movement_commands)}")
    
    # Servo engage statistics  
    if servo_engage_rtt_measurements:
        avg_eng = sum(servo_engage_rtt_measurements) / len(servo_engage_rtt_measurements)
        min_eng = min(servo_engage_rtt_measurements)
        max_eng = max(servo_engage_rtt_measurements)
        
        print(f"\n🔧 Servo Engage (M280 P0 S90):")
        print(f"  Average RTT: {avg_eng:.2f}ms (one-way ≈ {avg_eng/2:.2f}ms)")
        print(f"  Min RTT: {min_eng:.2f}ms (one-way ≈ {min_eng/2:.2f}ms)")
        print(f"  Max RTT: {max_eng:.2f}ms (one-way ≈ {max_eng/2:.2f}ms)")
        print(f"  Measurements: {len(servo_engage_rtt_measurements)}/{len(servo_engage_commands)}")
        
        # Performance assessment for servo engage
        if avg_eng < 50:
            print(f"  🚀 Excellent servo engage speed!")
        elif avg_eng < 100:
            print(f"  ✅ Good servo engage speed")
        else:
            print(f"  ⚠️  Servo engage could be faster")
    
    # Servo disengage statistics
    if servo_disengage_rtt_measurements:
        avg_dis = sum(servo_disengage_rtt_measurements) / len(servo_disengage_rtt_measurements)
        min_dis = min(servo_disengage_rtt_measurements)
        max_dis = max(servo_disengage_rtt_measurements)
        
        print(f"\n🔧 Servo Disengage (M280 P0 S0):")
        print(f"  Average RTT: {avg_dis:.2f}ms (one-way ≈ {avg_dis/2:.2f}ms)")
        print(f"  Min RTT: {min_dis:.2f}ms (one-way ≈ {min_dis/2:.2f}ms)")
        print(f"  Max RTT: {max_dis:.2f}ms (one-way ≈ {max_dis/2:.2f}ms)")
        print(f"  Measurements: {len(servo_disengage_rtt_measurements)}/{len(servo_disengage_commands)}")
        
        # Performance assessment for servo disengage
        if avg_dis < 50:
            print(f"  🚀 Excellent servo disengage speed!")
        elif avg_dis < 100:
            print(f"  ✅ Good servo disengage speed")
        else:
            print(f"  ⚠️  Servo disengage could be faster")
    
    # Comparison between servo operations
    if servo_engage_rtt_measurements and servo_disengage_rtt_measurements:
        avg_eng = sum(servo_engage_rtt_measurements) / len(servo_engage_rtt_measurements)
        avg_dis = sum(servo_disengage_rtt_measurements) / len(servo_disengage_rtt_measurements)
        
        print(f"\n⚖️  Servo Operation Comparison:")
        if avg_eng < avg_dis:
            print(f"  Engage is {avg_dis - avg_eng:.2f}ms faster than disengage")
        elif avg_dis < avg_eng:
            print(f"  Disengage is {avg_eng - avg_dis:.2f}ms faster than engage")
        else:
            print(f"  Engage and disengage have similar timing")


async def test_2_corrupted_packets(client):
    """
    Test 2: Send intentionally corrupted packets and measure retry timing
    
    This test sends malformed data and measures how quickly the device
    detects corruption and sends retry requests.
    """
    print("\n=== TEST 2: Corrupted Packet Retry Timing ===")
    
    global response_received, expected_response, last_response_time, last_successfully_sent_packet_id
    
    # Test cases with intentionally corrupted data
    corrupted_tests = [
        {
            "name": "Corrupted packet ID",
            "data": b"\xFF\xFF" + b"G0 X10 Y10\n",  # Invalid packet ID (255, 255)
            "description": "Packet with corrupted/invalid packet ID"
        },
        {
            "name": "Truncated packet", 
            "data": b"INCOMPLETE_COM",  # No newline terminator
            "description": "Incomplete command without proper termination"
        },
        {
            "name": "Non-ASCII data",
            "data": b"\x80\x81\x82BINARY_DATA\x83\x84\n",
            "description": "Command with binary/non-printable characters"
        },
        {
            "name": "Empty packet",
            "data": b"\n",
            "description": "Empty command"
        }
    ]
    
    retry_times = []  # Collect retry timing measurements
    
    for i, test_case in enumerate(corrupted_tests):
        print(f"\nTest 2.{i+1}: {test_case['name']}")
        print(f"  Description: {test_case['description']}")
        
        # For corrupted packet ID test, send raw data; for others, add valid packet ID
        if test_case['name'] == "Corrupted packet ID":
            # Send raw corrupted data (including corrupted packet ID)
            packet_to_send = test_case['data']
            expected_packet_id = None  # No valid packet ID expected for corrupted data
            print(f"  Sending RAW corrupted data (no valid packet ID added)")
        elif test_case['name'] in ["Truncated packet", "Non-ASCII data"]:
            # Force packet ID 9 for tests 2.3 and 2.4 since 2.2 failed to send
            packet_id = 9
            packet_to_send = bytes([packet_id]) + test_case['data']
            expected_packet_id = packet_id
            print(f"  Using FORCED packet ID: {expected_packet_id} (device expects this after 2.2 failed)")
        else:
            # Add valid packet ID to corrupted payload
            packet_to_send = create_packet_with_id(test_case['data'])
            expected_packet_id = packet_to_send[0]
            print(f"  Using valid packet ID: {expected_packet_id}")
            
        print(f"  Sending: {packet_to_send}")
        
        # Reset response tracking
        response_received = False
        expected_response = "error"  # Expect "<packet_id>error" response
        
        # Record send time with high precision
        send_time = time.time()
        
        # Send corrupted data
        packet_sent_successfully = False
        try:
            await client.write_gatt_char(RX_CHAR_UUID, packet_to_send)
            print(f"  ✅ Packet sent successfully")
            packet_sent_successfully = True
            
            # Track the last successfully sent packet ID for validation
            if expected_packet_id is not None:
                last_successfully_sent_packet_id = expected_packet_id
                
        except Exception as e:
            print(f"  ❌ Failed to send packet: {e}")
            print(f"     Device never received packet ID {expected_packet_id}, rolling back counter")
            
            # Rollback packet ID since this packet never reached the device
            if expected_packet_id is not None:
                rollback_packet_id()
                print(f"     Next packet will reuse packet ID {expected_packet_id}")
            continue
        
        # Wait for retry request
        timeout = 3.0  # 3 second timeout for error handling
        start_wait = time.time()
        
        while not response_received and (time.time() - start_wait) < timeout:
            await asyncio.sleep(0.001)  # Smaller delay for precise timing
        
        if response_received and last_response_time:
            retry_time = (last_response_time - send_time) * 1000  # Convert to ms
            retry_times.append(retry_time)
            
            print(f"  Retry request received at: {last_response_time:.6f}s")
            print(f"  ⏱️  Retry detection time: {retry_time:.2f}ms")
            print(f"  ✅ Device responded with retry request")
            
            # Validate packet ID based on what was actually sent successfully
            if expected_packet_id is not None and last_received_packet_id is not None:
                # Check if device echoed the packet ID we just sent
                if last_received_packet_id == expected_packet_id:
                    print(f"  ✅ Packet ID correctly echoed: {expected_packet_id}")
                elif last_successfully_sent_packet_id is not None and last_received_packet_id == last_successfully_sent_packet_id:
                    print(f"  ⚠️  Device echoed previous successful packet ID: {last_received_packet_id}")
                    print(f"     This is expected if previous packets failed to send")
                    print(f"     Current packet ID {expected_packet_id} was sent, but device may be responding to earlier packet")
                else:
                    print(f"  ❌ Unexpected packet ID! Sent: {expected_packet_id}, Received: {last_received_packet_id}")
                    if last_successfully_sent_packet_id is not None:
                        print(f"     Last successful packet ID was: {last_successfully_sent_packet_id}")
                    print(f"     ⚠️  Check packet sequencing logic")
            elif expected_packet_id is None:
                print(f"  ℹ️  Corrupted packet test - packet ID validation skipped")
        else:
            print(f"  ❌ No retry request received within {timeout}s")
        
        # Small delay between tests
        await asyncio.sleep(0.5)
    
    # Calculate retry timing statistics
    if retry_times:
        avg_retry = sum(retry_times) / len(retry_times)
        min_retry = min(retry_times)
        max_retry = max(retry_times)
        
        print(f"\n� Retry Timing Statistics:")
        print(f"  Average retry time: {avg_retry:.2f}ms")
        print(f"  Fastest retry time: {min_retry:.2f}ms") 
        print(f"  Slowest retry time: {max_retry:.2f}ms")
        print(f"  Successful retry measurements: {len(retry_times)}/{len(corrupted_tests)}")
        
        # Performance assessment
        if avg_retry < 50:
            print(f"  🚀 Excellent: Average retry detection under 50ms")
        elif avg_retry < 100:
            print(f"  ✅ Good: Average retry detection under 100ms")
        elif avg_retry < 200:
            print(f"  ⚠️  Acceptable: Average retry detection under 200ms")
        else:
            print(f"  🐌 Slow: Average retry detection over 200ms - consider optimization")
    else:
        print(f"\n❌ No successful retry measurements obtained")
        print(f"   Check if device firmware implements error detection and retry requests")


def handle_rx_with_timing(_, data: bytearray):
    """Enhanced notification callback that tracks response timing"""
    global last_response_time, response_received, expected_response, last_received_packet_id
    
    last_response_time = time.time()
    response_received = True
    
    # Parse packet: first byte is packet ID, rest is message
    if len(data) > 0:
        packet_id = data[0]
        last_received_packet_id = packet_id  # Store for validation
        message = data[1:].decode(errors='ignore').strip()
        print(f"RX -> Packet ID: {packet_id}, Message: '{message}'")
        
        # Check if this matches expected response for timing tests
        if expected_response and expected_response in message:
            print(f"  ✅ Expected response received: {expected_response} (Packet ID: {packet_id})")
    else:
        print(f"RX -> Empty packet received")
        last_received_packet_id = None


async def main():
    print("Scanning for BLE devices...")
    devices = await BleakScanner.discover()
    target = None
    for d in devices:
        if d.name and DEVICE_NAME in d.name:
            target = d
            break

    if not target:
        print(f"Device '{DEVICE_NAME}' not found.")
        return

    print(f"Connecting to {target.name} ({target.address})...")
    async with BleakClient(target) as client:
        if not client.is_connected:
            print("Connection failed.")
            return
        print("✅ Connected!")

        # Enhanced notification callback for timing tests
        await client.start_notify(TX_CHAR_UUID, handle_rx_with_timing)

        # Test 1: Round-trip time measurement for BLE commands
        await test_1_round_trip_time(client)

        # Test 2: Corrupted packet handling and retry logic
        await test_2_corrupted_packets(client)

        print("\n=== Tests completed ===")
        
        # Keep connection open briefly to ensure all responses are received
        await asyncio.sleep(2)

        await client.stop_notify(TX_CHAR_UUID)
        print("🔌 Disconnected.")

asyncio.run(main())
