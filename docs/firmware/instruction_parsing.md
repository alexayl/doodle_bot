# How the firmware parses movements

## Why G-Code?

Industry-standard for CNC machines, 3D printers, etc with well-defined and battle-tested set of commands for traversing 3d space.

## Subset of G-Code supported in MVP

- G91 - relative positioning (moves specified to bot's current position)
    - `G91`
- G1 - Linear-interpolated movement
    - `G1 X<mm> Y<mm>`
- M280 - Moves servos eraser and marker
    - `M280 P<pin_or_servo_index> S<position>` with `0` for pen servo and `1` for the eraser servo as pins, and the position [0, 180]

## How is G-Code transmitted?

G-code is transmitted via BLE using the Nordic UART Service (NUS) this exposes and RX and TX characteristic allowing for packets to be send both ways. The host will send each G-code line one at a time in plain text, and the firmware will send an acknowledgement back for each successful command that has been issued to the appropriate peripheral.

The first byte of each packet is the packet id. This value is an 8 bit integer (0-255) that increments with each packet sent.

## Example G-Code packets being sent

```
<pid>M280 P1 S90    // set eraser up
<pid>G91            // turn on relative positioning
<pid>M280 P0 S0     // set the pen down
<pid>G1 X10 Y-10    // move to the right 10 mm and down 10 mm
```

## Packet acknowledgement

After each packet is received by the firmware, it is parsed and queued for operation. Once the operation begins on the appropriate peripheral (e.g. step commands are sent for G1), these are the commands sent:

```
<pid>ok             // packet successfully parsed
<pid>fail           // packet unsuccessfully parsed     
```

## Protecting against corrupted packets

If the G-code parser fails to parse a command (it received a packet with the incorrect packet id, or the command was invalid) it will ask for a retry by sending the fail command.
