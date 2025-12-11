#!/usr/bin/env python3
"""
Simple Wheelbase Calibration Test

Sets wheelbase via BLE and draws a square.
If the square doesn't close, adjust the wheelbase.

Usage:
    python calibrate_wheelbase.py 183.0      # Test with wheelbase=183mm
    python calibrate_wheelbase.py 185.0      # Test with wheelbase=185mm
"""

import asyncio
import sys
from packetlib import BLEPacketHandler


SQUARE_GCODE = [
    b"G91\n",
    # Lift marker, move to start
    b"M280 P0 S0\n",
    b"G1 X50 Y0\n",
    # Set down marker
    b"M280 P0 S50\n",
    # Draw square 1
    b"G1 X100 Y0\n",
    b"G1 X0 Y100\n",
    b"G1 X-100 Y0\n",
    b"G1 X0 Y-100\n",
    # Draw square 2
    b"G1 X100 Y0\n",
    b"G1 X0 Y100\n",
    b"G1 X-100 Y0\n",
    b"G1 X0 Y-100\n",
    # Lift marker
    b"M280 P0 S0\n",
]


async def main():
    if len(sys.argv) < 2:
        print("Usage: python calibrate_wheelbase.py <wheelbase_mm>")
        print("Example: python calibrate_wheelbase.py 183.0")
        return
    
    wheelbase = float(sys.argv[1])
    
    print(f"\n=== WHEELBASE CALIBRATION TEST ===")
    print(f"Testing wheelbase: {wheelbase:.1f} mm")
    print(f"Drawing two 100mm squares")
    print()
    
    handler = BLEPacketHandler()
    
    try:
        print("Connecting...")
        if not await handler.connect():
            print("Failed to connect!")
            return
        
        # Set wheelbase
        print(f"Setting wheelbase to {wheelbase:.1f} mm...")
        await handler.send_packet(f"M504 W{wheelbase:.2f}\n".encode())
        await asyncio.sleep(0.5)
        
        input("\nPress Enter to draw squares...")
        
        # Draw squares
        print("\nDrawing...")
        await handler.send_packets(SQUARE_GCODE)
        
        input("\nPress Enter when done...")
        
    finally:
        await handler.disconnect()
    
    print("\n=== RESULT ===")
    print("Check if the square closes properly:")
    print("  - Square extends past start → wheelbase too LARGE, decrease it")
    print("  - Square doesn't reach start → wheelbase too SMALL, increase it")
    print()
    print("Try again with adjusted value:")
    print(f"    python calibrate_wheelbase.py <new_wheelbase>")
    print()
    print("When calibrated, update config.h:")
    print(f"    #define WHEELBASE (<final_value>f)")


if __name__ == "__main__":
    asyncio.run(main())
