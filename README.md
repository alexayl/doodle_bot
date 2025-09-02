# Doodle Bot

## Quick Start

### Prerequisites

- [Python3](https://www.python.org/)
- [Zephyr SDK](https://docs.zephyrproject.org/latest/getting_started/index.html)
- [Just command runner](https://github.com/casey/just)

### Setup Instructions

Create workspace and clone code
```bash
mkdir -p doodle_bot_workspace && cd doodle_bot_workspace

python3 -m venv .venv

source .venv/bin/activate # or windows command

pip install west

cd .. && west init -m https://github.com/alexayl/doodle_bot --mr main doodle_bot_workspace

cd doodle_bot_workspace

west update

cd doodle-bot

west packages pip --install

cp -r doodle-bot/scripts/.vscode .vscode
```

### Building and Flashing

#### For ESP32-S3:
```bash
# Build the application
just build-esp32

# Flash to device
just flash-esp32

# Monitor serial output
just monitor-esp32

# Or do all in one command
just run-esp32
```

## Usage

## License

This project is licensed under the Apache License 2.0. See the LICENSE file for details.

---
