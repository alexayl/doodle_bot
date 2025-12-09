# Doodlebot Firmware Application

## Overview

The main application firmware for the DoodleBot project. This firmware implements a multi-threaded architecture for controlling the robot's movement, communication, and state management. The application coordinates between Bluetooth communication, navigation control, and system state management through a message queue system.

## Architecture

The application uses a threaded architecture with three main threads:

- **Communications Thread** (`comms_thread`) - Handles Bluetooth Low Energy (BLE) communication via Nordic UART Service (NUS)
- **Navigation Thread** (`motion_planner_thread`) - Processes movement instructions and controls servo motors
- **State Management Thread** (`state_thread`) - Manages overall system state and coordination

Communication between threads is handled through Zephyr message queues, specifically the `gcode_cmd_msgq` for navigation instructions.

## Features

- **Bluetooth Low Energy (BLE) Communication**
  - Nordic UART Service (NUS) implementation
  - Wireless command reception from computer vision system
  - Real-time data transmission

- **Multi-threaded Architecture** 
  - Concurrent processing of communication, navigation, and state management
  - Message queue-based inter-thread communication
  - Priority-based thread scheduling

- **Hardware Control**
  - PWM-based servo motor control
  - LED status indicators
  - Custom servo driver integration

- **Navigation System**
  - G-code instruction parsing and execution
  - Servo position control for drawing operations
  - Coordinated movement patterns

## Usage

### Building and Running

From the firmware directory, use the provided just commands:

```bash
# Build and run on nrf52840dk (complete workflow)
just run-nrf

# Individual steps:
just build-nrf     # Build the application
just flash-nrf     # Flash to board
just monitor-nrf   # Monitor serial output

# Build and run in QEMU simulator
just run-sim

# Build and run BLE sample
just run-ble
```
