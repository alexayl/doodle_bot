# DoodleBot Computer Vision

## Overview
The Computer Vision (CV) subsystem provides real-time localization and feedback for the DoodleBot platform.  
It runs on a Raspberry Pi 5 and integrates camera-based perception, geometric calibration, and marker-based pose estimation using **OpenCV** and **ArUco** detection.  
The subsystem continuously tracks the robot’s position and orientation on the drawing board and communicates live correction data to the firmware over Bluetooth.  
A Flask-based web interface enables real-time monitoring through annotated video streams and provides REST APIs for control, status, and telemetry.

This folder hosts the camera, perception, and motion-correction stack that gives the robot real-time feedback. The `app/` package is the active codebase—each module below describes a specific responsibility so you can dive in quickly.


### `app/server.py`
The Flask entry point. It spins up the threaded camera, owns the `CVPipeline`, tracks board limits, and exposes routes for:
- Streaming JPEG frames produced by `encode_jpeg`.
- Uploading/normalizing G-code paths, including homing and board fitting helpers.
- Windowed G-code transmission to firmware with live CV-based corrections.
It also manages runtime state such as the active mode, metrics buffer, Bluetooth link, and in-flight path followers.

### `app/camera.py`
Defines `CameraConfig` and `ThreadedCamera`. The helper config masks ROI, handles MJPEG/back-end selection, and optionally locks Linux camera controls. The threaded reader continuously captures frames, filters by Laplacian sharpness, surfaces diagnostics (`get_stats`), and provides safe snapshots for the rest of the stack.

### `app/cv_core.py`
Implements the heavy CV logic:
- `BoardCalibrator` detects ArUco corners, builds/validates the homography, and enforces reprojection quality.
- `BotTracker` estimates pose from robot-mounted markers and smooths heading/position.
- `CVPipeline` orchestrates calibration, pose smoothing, board bounds, per-path stats, G-code clamping, waypoint fitting, and exposes the `PathCorrectionEngine` that nudges firmware commands based on cross-track error.

### `app/control.py`
Holds motion-planning data models and the waypoint follower. `PathFollower` converts board-frame deltas into robot forward/left step commands (via `board_to_robot`), debounces waypoint hits, and enforces margins. `PathRun` is a lightweight manager for loading, starting, and monitoring waypoint sequences.

### `app/path_parse.py`
Parses incremental G-code from the pathfinding tools into semantic operations (`("move", dx, dy)` and pen toggles). Also provides `scale_gcode_to_board` so uploaded paths can be resized to the detected board dimensions while preserving the intended aspect.

### `app/bt_link.py`
Asynchronous Bluetooth Low Energy transport based on `bleak`. It scans for the configured device, opens the Nordic UART service, and exposes synchronous helpers (`send_gcode`, `send_lines`) backed by a sliding-window packet/ACK loop with retries, per-packet IDs, and notification callbacks.

### `app/utils.py`
Miscellaneous helpers: a TurboJPEG-backed `encode_jpeg` fallback, `board_to_robot` for rotating board deltas into robot commands, and a rolling `Metrics` collector that tracks frame timing, uptime, and processing rate for the web UI.

## Installation
1. Clone the repository:
   ```bash
   git clone https://github.com/alexayl/doodle_bot.git
   cd cv_software
   ```

2. Create a virtual environment and install dependencies:
   ```bash
   python3 -m venv venv
   source venv/bin/activate
   pip install -r requirements.txt
   ```

3. (Optional) To improve latency, enable MJPEG mode for UVC cameras on Linux:
   ```bash
   v4l2-ctl -d /dev/video0 --set-fmt-video=width=1280,height=720,pixelformat=MJPG
   ```

4. Launch the application:
   ```bash
   python3 -m app.server
   ```

## Usage
- Access the web interface at `http://<raspberrypi-ip>:5000`.
- Modes:
  - **/draw:** Upload an image for path rendering.
  - **/erase:** Command the robot to clear the workspace.
- Logs and captured images are stored in the `uploads/` directory.
