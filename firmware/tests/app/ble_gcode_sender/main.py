"""
G-Code File Sender for Doodle Bot

Usage:
    python main.py [filename.gcode]
"""

import asyncio
import sys
import argparse
from typing import List
from packetlib import BLEPacketHandler

DEVICE_NAME = "BOO"

default_commands = [
    b"G1 X100 Y0\n",
    b"M280 P0 S90\n",
    b"G1 X0 Y100\n", 
    b"G1 X-100 Y0\n",
    b"G1 X0 Y-100\n",
    b"M280 P0 S0\n",
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
        description='Send G-code files to Doodle Bot via BLE',
        epilog='Example: python packetsender.py drawing.gcode'
    )
    parser.add_argument(
        'filename', 
        nargs='?', 
        help='G-code file to send (.gcode extension recommended)'
    )
    
    args = parser.parse_args()
    
    if args.filename:
        commands = load_gcode_file(args.filename)
    else:
        commands = default_commands
        print(f"No file specified, using default test pattern ({len(commands)} commands)")
    
    # Transmit packets
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
