"""
Measures BLE-related specifications firmware in DD3.

Usage:
    python main.py [spec #]
"""

import asyncio
import sys
import argparse
from typing import List
from packetlib import BLEPacketHandler

DEVICE_NAME = "BOO"

normal_commands = [
    b"G1 X100 Y0\n",
    b"G1 X0 Y100\n", 
    b"G1 X-100 Y0\n",
    b"G1 X0 Y-100\n",
]

corrupted_commands = [
    b"G4 X100 Y0\n",
    b"G1 C0 Y-100\n",
    b"abdasd",
    b"\n\n\n",
]

def load_gcode_file(filename: str) -> List[bytes]:
    """
    Parse G-code file into transmittable command list.
    """
    commands = []
    try:
        with open(filename, 'r') as file:
            for line in file:
                line = line.strip()
                if line and not line.startswith(';'):
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


async def main():
    # Command-line argument parsing
    parser = argparse.ArgumentParser(
        description='Test packet transmission and retry times.',
        epilog='Example: python packetsender.py 1'
    )
    parser.add_argument(
        'spec_num', 
        nargs='?', 
        help='Specification # to test (1 or 2)'
    )
    
    args = parser.parse_args()
    
    if args.spec_num == '1':
        commands = normal_commands
    elif args.spec_num == '2':
        commands = corrupted_commands
    else:
        commands = normal_commands + corrupted_commands
        print(f"No spec number specified, running both tests.")
    
    # transmit packets
    packet_handler = BLEPacketHandler(DEVICE_NAME)
    try:
        if not await packet_handler.connect():
            print("Failed to establish BLE connection")
            return
        
        print(f"Starting transmission of {len(commands)} commands...")
        await packet_handler.send_packets(commands)
        print("All commands sent successfully!")
        
        await asyncio.sleep(2)
        
    except KeyboardInterrupt:
        print("\nTransmission interrupted by user")
    except Exception as e:
        print(f"Transmission failed: {e}")
    finally:
        await packet_handler.disconnect()

if __name__ == "__main__":
    asyncio.run(main())
