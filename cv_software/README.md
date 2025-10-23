
---

### Distribution Plan (Computer Vision Subsystem)

The Computer Vision subsystem will be distributed as a runnable Python application on the Raspberry Pi that integrates seamlessly with the Pathfinding and Firmware subsystems. No hardware modification or manual calibration is required beyond the printed ArUco board markers.

**Usage and Integration**

* The Flask server (`app/server.py`) provides `/stream`, `/draw`, and `/erase` endpoints for visualization, command input, and coordination with the robot.
* The CV pipeline (`app/cv_core.py`) automatically calibrates to the board, tracks robot pose, and computes real-time corrections.
* Pathfinding outputs `.gcode` files that are uploaded via `/draw` and automatically scaled to the board.
* The subsystem communicates with the firmware through BLE using the `BTLink` module, sending batched relative waypoints.

**Deployment**

* Clone the repository on the Raspberry Pi and run:

  ```bash
  python3 -m venv .venv
  source .venv/bin/activate
  pip install -r requirements.txt
  python -m app.server
  ```
* Once running, open a browser to `http://<pi>:5000/stream` to view live tracking and initiate drawing or erasing.

**Validation**

* System is considered operational when:

  * The board and robot markers are detected and stable on `/stream`.
  * Pose updates occur at ≥ 10 Hz with ≤ 120 ms median latency.
  * Uploaded paths execute within board bounds and respond to correction feedback.

**End-of-Semester Distribution**

* Deliverables include:

  * Final Git tag (`cv-v1.0`) containing all source files.
  * `requirements.txt` for setup.
  * Calibration PDF with ArUco marker layout.

---