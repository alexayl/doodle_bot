# DoodleBot Computer Vision

## Overview
The Computer Vision (CV) subsystem provides real-time localization and feedback for the DoodleBot platform.  
It runs on a Raspberry Pi 5 and integrates camera-based perception, geometric calibration, and marker-based pose estimation using **OpenCV** and **ArUco** detection.  
The subsystem continuously tracks the robot’s position and orientation on the drawing board and communicates live correction data to the firmware over Bluetooth.  
A Flask-based web interface enables real-time monitoring through annotated video streams and provides REST APIs for control, status, and telemetry.

## Features
- **Multi-threaded Frame Capture:**  
  Captures frames asynchronously at up to 30 fps using a low-latency video backend with sharpness and exposure filtering.
- **Board Calibration and Homography Mapping:**  
  Uses four fixed ArUco markers to compute a perspective transform that aligns the camera’s image space to the board coordinate system.
- **Marker-Based Localization:**  
  Detects two robot-mounted fiducial markers to determine position and heading in millimeter-scale precision.
- **Dynamic Scaling:**  
  Automatically derives pixel-to-millimeter conversion from board geometry or marker baseline.
- **Web Visualization and Telemetry:**  
  Streams annotated video, calibration metrics, and robot pose data via Flask endpoints for live visualization and integration with other subsystems.

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
- API Endpoints:
  - `/api/moves` – enqueue motion commands.
  - `/api/pen` – toggle pen state (up/down).
  - `/api/stop` – halt motion.
  - `/stream` – retrieve the live annotated camera feed.
  - `/health` – report calibration and detection status.
- Logs and captured images are stored in the `uploads/` directory.
