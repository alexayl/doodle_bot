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
    b"M280 P1 S50\n",
    b"G1 X20 Y0\n",
    b"M280 P0 S0\n",
    b"G1 X0 Y100\n", 
    b"G1 X-100 Y0\n",
    b"G1 X0 Y-100\n",
    b"M280 P0 S0\n",
]

servo_test_commands = [
    b"M280 P0 S50\n",
    b"M280 P1 S0\n",
    # b"M280 P0 S45\n",
    # b"M280 P0 S90\n",
    # b"M280 P0 S135\n",
]

dashed_line_commands = [
    b"M280 P0 S40\n",    # Marker DOWN (draw)
    b"G1 X20 Y0\n",      # Move while drawing
    b"M280 P0 S0\n",     # Marker UP (stop drawing)
    b"G1 X20 Y0\n",      # Move without drawing
    b"M280 P0 S40\n",    # Marker DOWN (draw)
    b"G1 X20 Y0\n",
    b"M280 P0 S0\n",     # Marker UP
    b"G1 X20 Y0\n",
    b"M280 P0 S40\n",    # Marker DOWN (draw)
    b"G1 X20 Y0\n",
    b"M280 P0 S0\n",     # Marker UP
    b"G1 X20 Y0\n",
    b"M280 P0 S40\n",    # Marker DOWN (draw)
    b"G1 X20 Y0\n",
    b"M280 P0 S0\n",     # Marker UP (end lifted)
]

esap5_commands = [
    b"M280 P0 S0\n",
    b"G1 X45 Y503 \n",
    b"M280 P0 S40\n",
    b"G1 X-30 Y0\n",
    b"G1 X-5 Y5\n",
    b"G1 X-15 Y0\n",
    b"G1 X-10 Y-10\n",
    b"G1 X0 Y-20\n",
    b"G1 X10 Y-10\n",
    b"G1 X20 Y0\n",
    b"G1 X5 Y5\n",
    b"G1 X20 Y0\n",
    b"G1 X10 Y10\n",
    b"G1 X0 Y15\n",
    b"G1 X5 Y5\n",
    b"M280 P0 S0\n",
    b"G1 X40 Y-20\n",   
    b"M280 P0 S40\n",
    b"G1 X0 Y5\n",
    b"G1 X5 Y5\n",
    b"G1 X0 Y5\n",
    b"G1 X0 Y5\n",
    b"G1 X5 Y5\n",
    b"G1 X0 Y5\n",
    b"G1 X5 Y5\n",
    b"G1 X80 Y0\n",
    b"G1 X10 Y10\n",
    b"G1 X0 Y25\n",
    b"G1 X-5 Y5\n",
    b"G1 X-90 Y0\n",
    b"G1 X-5 Y-5\n",
    b"G1 X0 Y-10\n",    
    b"G1 X-5 Y-5\n",
    b"G1 X0 Y-15\n",
    b"G1 X-5 Y-5\n",
    b"G1 X0 Y-15\n",
    b"G1 X-5 Y-5\n",
    b"G1 X0 Y-40\n",
    b"G1 X-5 Y-5\n",
    b"G1 X0 Y-10\n",
    b"G1 X10 Y-10\n",
    b"G1 X135 Y0\n",
    b"M280 P0 S0\n",
    b"G1 X5 Y-15\n",
    b"M280 P0 S40\n",
    b"G1 X0 Y10\n",
    b"G1 X5 Y5\n",
    b"G1 X40 Y0\n",
    b"G1 X5 Y5\n",
    b"G1 X30 Y0\n",
    b"G1 X5 Y-5\n",
    b"G1 X15 Y0\n",
    b"G1 X5 Y5\n",
    b"G1 X5 Y0\n",
    b"G1 X10 Y10\n",
    b"G1 X0 Y15\n",
    b"G1 X5 Y5\n",
    b"G1 X0 Y10\n",
    b"G1 X-10 Y10\n",
]

commands_to_send = esap5_commands

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
        commands = commands_to_send
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
        
    except KeyboardInterrupt:
        print("\nTransmission interrupted by user")
    except Exception as e:
        print(f"Transmission failed: {e}")
    finally:
        await packet_handler.disconnect()

if __name__ == "__main__":
    asyncio.run(main())
