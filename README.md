# Doodle Bot

DoodleBot device for autonomous whiteboard management.

This repository is setup as a **monorepo** that contains the code for the different subsystems that are flashed to different devices. Please see the links below for each subsystems' README:

- [Firmware](firmware/README.md) (Alex Aylward and Jain Iftesam) is responsible for all of the code on the device.
- [CV Software](cv_software/README.md) (Trevor Antle) is responsible for...
- [Pathfinding](pathfinding/README.md) (Tom Concannon) is responsible for.



## Setup

### Docs

Requirements: graphviz

### Firmware

_Note: If you are on a PC, Zephyr does not work well with WSL. Please use Windows native, or consider eventually dual-booting/VM linux!_

1. Install prerequisites

    - [Python3](https://www.geeksforgeeks.org/python/download-and-install-python-3-latest-version/)

    - Complete **only** the [Install Dependencies](https://docs.zephyrproject.org/latest/develop/getting_started/index.html#install-dependencies) section of the [Getting Started Guide](https://docs.zephyrproject.org/latest/develop/getting_started/index.html#getting-started-guide).

    - [Just command runner](https://github.com/casey/just?tab=readme-ov-file#installation) - This is used like a makefile as a hotkey for all of our project-specific commands
        - If you are using Windows, installing chocolatey package manager is recommended. After installing chocolatey, you can install just by running `choco install just`.
        - If you are on Mac, use the [Homebrew](https://brew.sh/) command `brew install just`.

If building/flsahing for [nrf52840 dk](https://docs.zephyrproject.org/latest/boards/nordic/nrf52840dk/doc/index.html):
    - SEGGER J-Link DLL
    - nrfutil (there is a python lib that allegedly has it; I needed to install it from the website though due to python dependency version conflicts)
    - 
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
    2.  Perform installation 
        ```bash
        west sdk install
        ```

7. Try to build and run
    1. navigate to `doodle_bot/firmware`
        ```bash
        cd ..
        ```
    2.  Build the sim to verify it works 
        ```bash
        just run-sim
        ```
