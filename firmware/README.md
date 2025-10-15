# Doodlebot Firmware
 
## Overview

The firmware contains two main parts. Please see the READMEs for each individual part for more information.

- [Application](app/README.md) (Alex Aylward) - The high-level logic
- [Drivers](drivers/README.md) (Jain Iftesam) - Hardware drivers and peripheral interfaces

## Features

- Bluetooth Low Energy (BLE) Nordic UART Service (NUS) support
- Real-time sensor data collection and processing
- Motor control and movement coordination
- Wireless communication with computer vision system
- Hardware abstraction layer for multiple board targets

## Installation

### Setup to work on firmware

Note: If you are on a Windows PC, use WSL2 (look up install instructions, ubuntu recommended) and follow the instructions for linux

1. Install prerequisites

    - Python3

    - Complete **only** the [Install Dependencies](https://docs.zephyrproject.org/latest/develop/getting_started/index.html#install-dependencies) section of the [Getting Started Guide](https://docs.zephyrproject.org/latest/develop/getting_started/index.html#getting-started-guide).

    - [Just command runner](https://github.com/casey/just?tab=readme-ov-file#installation) - This is used like a makefile as a hotkey for all of our project-specific commands
        - If you are using Windows, installing chocolatey package manager is recommended. After installing chocolatey, you can install just by running `choco install just`.
        - If you are on Mac, use the [Homebrew](https://brew.sh/) command `brew install just`.

    - **Windows + WSL2**: usbipd-win - This is used to expose the USB ports to WSL2 so that flashing works properly

2. Clone this repository:
    ```bash
    git clone git@github.com:alexayl/doodle_bot.git
    cd doodle_bot
    ```

3. Create and activate a Python virtual environment:
    ```bash
    python -m venv .venv
    
    source .venv/bin/activate     # Linux/macOS
    .venv\Scripts\activate.bat    # PC, Command Prompt
    .venv\Scripts\Activate.ps1    # PC, Powershell
    source .venv/Scripts/activate # PC. Git bash
    ```

4. Install west
    ```bash
    pip install west
    ```

5. Update the workspace
    ```bash
    cd firmware
    west update
    west packages pip --install
    ```

6. Install the Zephyr SDK
    1. navigate to `doodle_bot/firmware/zephyr`
        ```bash
        cd zephyr
        ```
    2. Perform installation 
        ```bash
        west sdk install
        ```

7. Try to build and run
    1. navigate to `doodle_bot/firmware`
        ```bash
        cd ..
        ```
    2. Build the sim to verify it works 
        ```bash
        just run-sim
        ```

## Usage

### Building and Flashing

For nRF52840 DK:
```bash
# Build for nrf52840dk
west build -b nrf52840dk/nrf52840 app

# Flash to board
west flash
```

For other targets, see individual application READMEs for board-specific instructions.

### Testing BLE Connectivity

1. Flash the peripheral_nus sample to your board
2. Use nRF Connect mobile app to scan and connect
3. Subscribe to NUS notifications to receive "Hello World!" messages
4. Send data to test bidirectional communication
