from __future__ import annotations

import os, sys, time, math, threading, shutil
from pathlib import Path
from contextlib import contextmanager
from typing import List, Tuple, Optional

# Load environment variables from .env file
from dotenv import load_dotenv
load_dotenv()

import cv2
import numpy as np
from flask import Flask, Response, request, render_template, redirect, url_for
from werkzeug.utils import secure_filename
import faulthandler

from app.cv_core import CVPipeline
from app.cv_core import BOARD_MARGIN_MM as CV_BOARD_MARGIN_MM, BOARD_MARGIN_FRAC as CV_BOARD_MARGIN_FRAC
from app.control import PathRun, Waypoint
from app.camera import ThreadedCamera, CameraConfig
from app.path_parse import load_gcode_file, convert_pathfinding_gcode
from app.bt_link import BTLink
from app.utils import encode_jpeg, Metrics

# -----------------------------------------------------------------------------#
# Config
# -----------------------------------------------------------------------------#
USE_TURBOJPEG = True  # encode_jpeg handles fallback
CAMERA_SRC = int(os.getenv("CAMERA_SRC", "0"))
PATHFINDING_DIR = os.getenv(
    "PATHFINDING_DIR",
    os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "pathfinding")),
)
STREAM_JPEG_QUALITY = 75
# Bounds margin configuration: use centralized cv_core values
# Note: environment overrides should be handled in cv_core

# Pathfinding canvas size used by path2gcode.py (pixels/abstract units)
PATH_CANVAS_SIZE = (
    float(os.getenv("PATH_CANVAS_WIDTH", "300")),
    float(os.getenv("PATH_CANVAS_HEIGHT", "250")),
)

app = Flask(__name__, template_folder="../templates", static_folder="../static")
os.makedirs("uploads", exist_ok=True)

STATE = {"mode": "home", "target_image": None}
MET = Metrics(maxlen=240)

# -----------------------------------------------------------------------------#
# Camera
# -----------------------------------------------------------------------------#
_cam: Optional[ThreadedCamera] = None
def get_cam() -> Optional[ThreadedCamera]:
    global _cam
    if _cam is None:
        try:
            _cam = ThreadedCamera(CameraConfig(src=CAMERA_SRC))
        except Exception:
            _cam = None
    return _cam

# -----------------------------------------------------------------------------#
# CV + Control
# -----------------------------------------------------------------------------#
cvp = CVPipeline(cam=None)
RUN = PathRun()
bt = BTLink()  # Simplified - uses DEVICE_NAME from environment or default "DOO"
AXIS_MAP = {"J": None, "Ji": None, "probe_mm": 10.0}
@contextmanager
def _in_dir(path: str):
    old = os.getcwd()
    os.makedirs(path, exist_ok=True)
    os.chdir(path)
    try:
        yield
    finally:
        os.chdir(old)

def _as_waypoints(seq) -> List[Waypoint]:
    out: List[Waypoint] = []
    if not seq:
        return out
    for item in seq:
        if isinstance(item, Waypoint):
            out.append(item)
        elif isinstance(item, (tuple, list)) and len(item) >= 2:
            out.append(Waypoint(x_mm=float(item[0]), y_mm=float(item[1])))
    return out

# -----------------------------------------------------------------------------#
# Simple axis mapping (robot steps -> board mm) using CV probes
# -----------------------------------------------------------------------------#
def _compute_axis_mapping(probe_mm: float = 10.0, settle_s: float = 0.25, max_retries: int = 2) -> bool:
    """Probe +X and +Y robot steps and measure board mm deltas via CV.
    Builds 2x2 Jacobian J (robot->board) and its inverse Ji (board->robot).
    Returns True on success.
    """
    global AXIS_MAP
    bounds = cvp.get_board_bounds_mm()
    if bounds is None:
        print("AXIS_MAP: No board bounds available")
        return False

    pose0 = cvp.get_pose_mm()
    if not pose0:
        print("AXIS_MAP: No initial pose available")
        return False
    
    # Check if we're in safe zone to probe (not too close to actual marker edges)
    min_x, min_y, max_x, max_y = bounds
    width_mm = max_x - min_x
    height_mm = max_y - min_y
    margin = max(15.0, 0.08 * min(width_mm, height_mm))
    cx, cy = pose0[0]
    if cx < min_x + margin or cx > max_x - margin or cy < min_y + margin or cy > max_y - margin:
        print(f"AXIS_MAP: Skipping probe, too close to edge ({cx:.1f}, {cy:.1f})")
        return False

    # Ensure pen up
    # try:
    #     bt.send_gcode_windowed("M280 P0 S90\n", window_size=1)
    #     time.sleep(0.2)
    # except Exception:
    #     pass

    # Retry loop with increasing settle times
    for attempt in range(max_retries + 1):
        current_settle = settle_s * (1 + attempt)  # 0.25s, 0.5s, 0.75s
        
        def _delta_after(cmd: str) -> Optional[Tuple[float, float]]:
            bt.send_gcode_windowed(cmd + "\n", window_size=1)
            time.sleep(current_settle)
            pose = cvp.get_pose_mm()
            return None if not pose else (pose[0][0], pose[0][1])

        # Measure X+
        p_before = cvp.get_pose_mm()
        if not p_before:
            continue
        x1 = _delta_after(f"G1 X{int(round(probe_mm))}")
        if x1 is None:
            continue
        # Return X-
        _ = _delta_after(f"G1 X{-int(round(probe_mm))}")
        if _ is None:
            continue

        # Measure Y+
        p_before2 = cvp.get_pose_mm()
        if not p_before2:
            continue
        y1 = _delta_after(f"G1 Y{int(round(probe_mm))}")
        if y1 is None:
            continue
        # Return Y-
        _ = _delta_after(f"G1 Y{-int(round(probe_mm))}")
        if _ is None:
            continue

        (cx0, cy0) = (p_before[0][0], p_before[0][1])
        (cx1, cy1) = x1
        (cx2, cy2) = (p_before2[0][0], p_before2[0][1])
        y1_pos = y1
        (cx2_after, cy2_after) = y1_pos
        
        # Verify robot actually moved (CV detected movement > 1mm)
        x_delta_mag = math.sqrt((cx1 - cx0)**2 + (cy1 - cy0)**2)
        y_delta_mag = math.sqrt((cx2_after - cx2)**2 + (cy2_after - cy2)**2)
        
        print(f"AXIS_MAP: Attempt {attempt + 1}/{max_retries + 1} (settle={current_settle:.2f}s):")
        print(f"  +X probe: sent {probe_mm:.1f}mm, measured ({cx1-cx0:.2f}, {cy1-cy0:.2f})mm delta (mag={x_delta_mag:.2f}mm)")
        print(f"  +Y probe: sent {probe_mm:.1f}mm, measured ({cx2_after-cx2:.2f}, {cy2_after-cy2:.2f})mm delta (mag={y_delta_mag:.2f}mm)")
        
        # Sanity check: measured deltas shouldn't be >3x commanded (indicates CV tracking error)
        max_reasonable_delta = probe_mm * 3.0
        if x_delta_mag > max_reasonable_delta or y_delta_mag > max_reasonable_delta:
            print(f"AXIS_MAP: Measured delta exceeds {max_reasonable_delta:.1f}mm - likely CV tracking error")
            if attempt < max_retries:
                print(f"AXIS_MAP: Retrying...")
                continue
            else:
                print("AXIS_MAP: All attempts show unreasonable deltas - aborting")
                return False
        
        # If CV detected reasonable movement, break retry loop
        if x_delta_mag > 1.0 and y_delta_mag > 1.0:
            print(f"AXIS_MAP: Movement detected, proceeding with calibration")
            break
        else:
            print(f"AXIS_MAP: Insufficient movement detected (X={x_delta_mag:.2f}mm, Y={y_delta_mag:.2f}mm)")
            if attempt < max_retries:
                print(f"AXIS_MAP: Retrying with longer settle time...")
    else:
        print("AXIS_MAP: All probe attempts failed - CV not tracking movement reliably")
        return False
    
    # Use final successful probe results
    (cx0, cy0) = (p_before[0][0], p_before[0][1])
    (cx1, cy1) = x1
    (cx2, cy2) = (p_before2[0][0], p_before2[0][1])
    (cx2_after, cy2_after) = y1_pos

    dx_x = (cx1 - cx0) / max(1e-6, probe_mm)
    dy_x = (cy1 - cy0) / max(1e-6, probe_mm)
    dx_y = (cx2_after - cx2) / max(1e-6, probe_mm)
    dy_y = (cy2_after - cy2) / max(1e-6, probe_mm)

    J = np.array([[dx_x, dx_y], [dy_x, dy_y]], dtype=float)
    
    # Safety checks for near-singular or ill-conditioned Jacobian
    det_J = np.linalg.det(J)
    if abs(det_J) < 1e-6:
        print(f"AXIS_MAP: Rejected - determinant too small ({det_J:.2e}), matrix is near-singular")
        return False
    
    cond_J = np.linalg.cond(J)
    if cond_J > 1e3:
        print(f"AXIS_MAP: Rejected - condition number too high ({cond_J:.1f}), matrix is ill-conditioned")
        return False
    
    try:
        Ji = np.linalg.inv(J)
    except Exception as e:
        print(f"AXIS_MAP: Failed to invert J: {e}")
        return False
    
    # Sanity check: Ji values shouldn't be absurdly large (indicates bad calibration)
    if np.max(np.abs(Ji)) > 500:
        print(f"AXIS_MAP: Rejected - Ji has extreme values (max={np.max(np.abs(Ji)):.1f}), likely bad probe data")
        return False

    AXIS_MAP["J"] = J
    AXIS_MAP["Ji"] = Ji
    AXIS_MAP["probe_mm"] = float(probe_mm)
    print(f"AXIS_MAP: J={J}, Ji={Ji}")
    print(f"AXIS_MAP: det={det_J:.4f}, cond={cond_J:.1f} - ACCEPTED")
    return True

def _map_board_to_robot_steps(bx: float, by: float, max_seg: int, min_step: int) -> Optional[Tuple[int, int]]:
    """Use Ji to convert desired board delta (mm) into robot step ints with clamping.
    ENSURES result won't violate actual marker bounds by checking current position + desired delta.
    """
    Ji = AXIS_MAP.get("Ji")
    if Ji is None:
        return None
    
    # Verify this delta won't exceed actual marker bounds
    pose = cvp.get_pose_mm()
    bounds = cvp.get_board_bounds_mm()
    if pose and bounds:
        cx, cy = pose[0]
        min_x, min_y, max_x, max_y = bounds
        # Use configured margins
        width_mm = max_x - min_x
        height_mm = max_y - min_y
        margin = max(CV_BOARD_MARGIN_MM, CV_BOARD_MARGIN_FRAC * min(width_mm, height_mm))
        
        # Clamp delta to stay within actual marker bounds
        next_x = cx + bx
        next_y = cy + by
        bx = max((min_x + margin) - cx, min(bx, (max_x - margin) - cx))
        by = max((min_y + margin) - cy, min(by, (max_y - margin) - cy))
    
    rr = Ji.dot(np.array([float(bx), float(by)], dtype=float))
    rx, ry = float(rr[0]), float(rr[1])
    sx_mag = max(min_step, min(max_seg, int(round(abs(rx)))))
    sy_mag = max(min_step, min(max_seg, int(round(abs(ry)))))
    sx = sx_mag if rx >= 0 else -sx_mag
    sy = sy_mag if ry >= 0 else -sy_mag
    if sx == 0 and sy == 0:
        return None
    return sx, sy

# -----------------------------------------------------------------------------#
# G-code helpers
# -----------------------------------------------------------------------------#
def _ops_to_normalized_wps(ops: List[tuple]) -> List[Waypoint]:
    """
    Convert ("move", dx, dy) incremental ops to [0..1] normalized waypoints.
    """
    x = y = 0.0
    pts: List[tuple[float, float]] = []
    for op in ops:
        if not op:
            continue
        if op[0] == "move":
            x += float(op[1]); y += float(op[2])
            pts.append((x, y))
    if not pts:
        return []
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    minx, maxx, miny, maxy = min(xs), max(xs), min(ys), max(ys)
    w = max(maxx - minx, 1e-6)
    h = max(maxy - miny, 1e-6)

    out: List[Waypoint] = []
    for px, py in pts:
        nx = (px - minx) / w
        ny0 = (py - miny) / h
        ny = ny0
        out.append(Waypoint(x_mm=float(nx), y_mm=float(ny)))
    return out

def _load_named_gcode_normalized(name: str) -> List[Waypoint]:
    bases = [
        os.path.join(PATHFINDING_DIR, f"{name}.gcode"),
        os.path.join(PATHFINDING_DIR, "gcode", f"{name}.gcode"),
    ]
    ops: List[tuple] = []
    for p in bases:
        if os.path.isfile(p):
            try:
                ops = load_gcode_file(p)
                break
            except Exception:
                pass
    if not ops:
        return []
    return _ops_to_normalized_wps(ops)

def _home_bot(starting_node_inset_p: float = 0.05, timeout_sec: float = 15.0) -> bool:
    """
    SIMPLIFIED: Move bot toward top-left corner with safety margin.
    Takes several steps to get close, uses CV feedback for positioning.
    
    Returns True if completed movement steps, False on error.
    """
    bounds = cvp.get_board_bounds_mm()
    if not bounds:
        print("HOMING FAILED: Board not calibrated")
        return False
    
    min_x, min_y, max_x, max_y = bounds
    width_mm = max_x - min_x
    height_mm = max_y - min_y
    
    margin = max(CV_BOARD_MARGIN_MM, CV_BOARD_MARGIN_FRAC * min(width_mm, height_mm))
    bp = getattr(cvp.cal, 'board_pose', None)
    target_x = min_x + margin + width_mm * starting_node_inset_p
    target_y = min_y + margin + height_mm * starting_node_inset_p
    # Clamp to safe zone
    target_x = max(min_x + margin, min(target_x, max_x - margin))
    target_y = max(min_y + margin, min(target_y, max_y - margin))

    print(f"HOMING: Target ({target_x:.1f}, {target_y:.1f})mm with {margin:.1f}mm margin (matches pathfinding start)")
    
    pose = cvp.get_pose_mm()
    if not pose:
        print("HOMING: No bot pose, sending blind move to top-left")
        # Blind homing: just move up-left with conservative steps
        target_x = min_x + margin + width_mm * starting_node_inset_p
        target_y = min_y + margin + height_mm * starting_node_inset_p
        bp = getattr(cvp.cal, 'board_pose', None)
        # Removed pixel-space TL corner fallback (mm_per_px dependency); bounds already supply mm coordinates
        print("HOMING: No bot pose, sending blind move to top-left")
        # Blind homing: just move up-left with conservative steps
        # bt.send_gcode_windowed("M280 P0 S90\n", window_size=1)  # Pen up
        bt.send_gcode_windowed("G1 X-40\nG1 Y-40\n", window_size=1)
        time.sleep(1.0)
        return True
    
    (current_x, current_y), heading, conf = pose
    dx_board = target_x - current_x
    dy_board = target_y - current_y
    
    print(f"HOMING: From ({current_x:.1f},{current_y:.1f}), need to move ({dx_board:.1f},{dy_board:.1f})mm")
    
    # Pen up
    # bt.send_gcode_windowed("M280 P0 S90\n", window_size=1)
    time.sleep(0.2)
    
    # Helper to force fresh CV update
    def get_fresh_pose():
        cam = get_cam()
        if cam:
            for _ in range(5):  # Try up to 5 frames to get valid pose
                frame = cam.get_latest_good()
                if frame is not None:
                    cvp.update_bot(frame)
                    pose = cvp.get_pose_mm()
                    if pose:
                        return pose
                time.sleep(0.05)
        return cvp.get_pose_mm()  # Fallback to cached
    
    # Iterate toward target with adaptive steps and pose waits
    max_steps = 20
    step_size_cap_mm = 60.0
    position_threshold = 15.0  # Consider arrived when within 15mm
    
    # Build axis mapping (robot -> board) once if not already available.
    if AXIS_MAP.get("J") is None or AXIS_MAP.get("Ji") is None:
        print("HOMING: Axis mapping not present; probing axes")
        if not _compute_axis_mapping(probe_mm=15.0, settle_s=0.30, max_retries=1):
            print("HOMING WARNING: Axis mapping failed; will fall back to naive heading rotation")
    use_axis_map = AXIS_MAP.get("Ji") is not None

    for i in range(max_steps):
        # Re-check position BEFORE move with FRESH CV update
        pose_before = get_fresh_pose()
        if not pose_before:
            print(f"HOMING: Lost pose tracking after {i} steps")
            break
        (cx, cy), heading, _ = pose_before
        
        # Calculate remaining distance in board frame
        remaining_x = target_x - cx
        remaining_y = target_y - cy
        distance = math.sqrt(remaining_x**2 + remaining_y**2)
        
        # Close enough? Stop early
        if distance < position_threshold:
            print(f"HOMING: Reached target ({distance:.1f}mm away) after {i+1} steps")
            return True
        
        # Adaptive step size (larger when far)
        step_size_mm = min(step_size_cap_mm, max(20.0, 0.4 * distance))

        # Clamp next position to safe board margins
        nx = cx + remaining_x
        ny = cy + remaining_y
        nx = max(min_x + margin, min(nx, max_x - margin))
        ny = max(min_y + margin, min(ny, max_y - margin))

        step_x_board = nx - cx
        step_y_board = ny - cy
        step_len = math.hypot(step_x_board, step_y_board)
        if step_len > step_size_mm and step_len > 1e-6:
            scale = step_size_mm / step_len
            step_x_board *= scale
            step_y_board *= scale

        if use_axis_map:
            mapped = _map_board_to_robot_steps(step_x_board, step_y_board, max_seg=55, min_step=1)
            if mapped is None:
                print("HOMING: Axis map produced no movement; forcing 1mm diagonal toward target")
                sx = -1 if remaining_x < 0 else 1
                sy = -1 if remaining_y < 0 else 1
            else:
                sx, sy = mapped
        else:
            # Fallback: approximate rotation by heading (legacy path)
            cos_t = math.cos(heading)
            sin_t = math.sin(heading)
            step_x_robot = step_x_board * cos_t + step_y_board * sin_t
            step_y_robot = -step_x_board * sin_t + step_y_board * cos_t
            sx = int(round(step_x_robot))
            sy = int(round(step_y_robot))

        # Avoid zero-length commands
        if sx == 0 and sy == 0:
            # Nudge along dominant axis by 1mm to keep progress
            if abs(step_x_robot) >= abs(step_y_robot):
                sx = 1 if step_x_robot >= 0 else -1
            else:
                sy = 1 if step_y_robot >= 0 else -1

        print(f"HOMING[{i+1}/{max_steps}]: dist={distance:.1f}mm, board_step=({step_x_board:.1f},{step_y_board:.1f}), robot_mm=({sx},{sy}), axis_map={'ON' if use_axis_map else 'OFF'} heading={math.degrees(heading):.1f}°")

        # Send robot-frame move in mm
        cmd = f"G1 X{sx} Y{sy}\n"
        bt.send_gcode_windowed(cmd, window_size=1)
        
        # Wait for movement to complete with fresh pose
        pose_after = _wait_for_pose_change(prev_xy=(cx, cy), min_mm=2.0, timeout_s=2.0)
        if pose_after:
            (cx_after, cy_after), _, _ = pose_after
            actual_movement = math.sqrt((cx_after - cx)**2 + (cy_after - cy)**2)
            # Bounds safety: if new position is outside safe zone, immediately reverse and shrink next steps.
            out_of_bounds = not (min_x + margin <= cx_after <= max_x - margin and min_y + margin <= cy_after <= max_y - margin)
            if out_of_bounds:
                print(f"HOMING SAFETY: Pose ({cx_after:.1f},{cy_after:.1f}) outside safe zone; reversing last move")
                bt.send_gcode_windowed(f"G1 X{-sx} Y{-sy}\n", window_size=1)
                # Reduce cap aggressively to prevent future excursions
                step_size_cap_mm = max(15.0, step_size_cap_mm * 0.5)
            elif actual_movement < 1.0:
                print(f"HOMING WARNING: CV shows minimal movement ({actual_movement:.1f}mm) - may need longer settle time")
            else:
                new_remaining = math.sqrt((target_x - cx_after)**2 + (target_y - cy_after)**2)
                print(f"HOMING: CV detected {actual_movement:.1f}mm movement (remaining {new_remaining:.1f}mm)")
                # If movement unexpectedly large (>2x planned board step length) shrink future steps
                planned_len = math.hypot(step_x_board, step_y_board)
                if actual_movement > planned_len * 2.0:
                    step_size_cap_mm = max(20.0, step_size_cap_mm * 0.7)
                    print(f"HOMING ADJUST: Large actual movement vs planned ({actual_movement:.1f}>{planned_len*2.0:.1f}); reducing step cap to {step_size_cap_mm:.1f}mm")
    
    # Final position check
    pose = cvp.get_pose_mm()
    if pose:
        (cx, cy), _, _ = pose
        final_dist = math.sqrt((target_x - cx)**2 + (target_y - cy)**2)
        # If distance accidentally increased a lot, attempt Y sign inversion next iteration
        if pose_after:
            new_remaining = math.sqrt((target_x - cx_after)**2 + (target_y - cy_after)**2)
            print(f"HOMING: CV detected {actual_movement:.1f}mm movement (remaining {new_remaining:.1f}mm)")
            if new_remaining > distance * 1.10:
                # Invert next Y board component heuristic by nudging target slightly
                target_y = max(min_y + margin, min(target_y - 5.0, max_y - margin))
                print("HOMING ADJUST: Large distance increase detected, nudging target_y upward 5mm to reinforce top-left approach")
        print(f"HOMING: Completed {max_steps} steps, final distance: {final_dist:.1f}mm")
    else:
        print(f"HOMING: Completed {max_steps} steps toward top-left")
    
    return True

def _send_with_cv_correction(firmware_gcode: str, fitted_wps: List[Waypoint], window_size: int = 5):
    """Send G-code with real-time CV correction using centralized PathCorrectionEngine."""
    # Reset correction engine for new path
    cvp.correction.reset()
    
    def correction_callback(head_pid: int, head_line: str, next_line: str) -> List[str]:
        """Delegate to centralized correction engine."""
        if not RUN.active:
            return []
        
        current_idx = RUN.follower.index()
        next_is_g1 = next_line.strip().startswith("G1")
        
        correction = cvp.correction.compute_correction(
            waypoints=fitted_wps,
            current_index=current_idx,
            next_is_g1=next_is_g1
        )
        
        if correction is None:
            return []
        
        sx, sy = correction
        return [f"G1 X{sx} Y{sy}"]
    
    def on_head_executed(head_pid: int, head_line: str, ack_msg: str):
        """Update PathFollower with current pose when commands execute."""
        # Update PathFollower's waypoint tracking based on current position
        pose = cvp.get_pose_mm()
        if pose and RUN.active:
            reached, deviation = RUN.follower.update_and_check_reached(pose)
            if reached:
                print(f"WAYPOINT_REACHED: advanced to {RUN.follower.index()}/{RUN.follower.total()}")
    
    # Send with windowed flow control and CV correction
    bt.send_gcode_windowed(
        firmware_gcode,
        window_size=window_size,
        correction_cb=correction_callback,
        on_head_executed=on_head_executed
    )

def _convert_image_to_gcode(img_path: str, out_gcode_path: str, canvas_size=(575, 730)) -> str:
    """Run the pathfinding stack and write G-code next to the uploaded image."""
    if PATHFINDING_DIR not in sys.path:
        sys.path.insert(0, PATHFINDING_DIR)
    from img2graph import img2graph
    from graph2path import graph2path
    from path2gcode import path2gcode

    img_path = os.path.abspath(img_path)
    out_gcode_path = os.path.abspath(out_gcode_path)
    png_dir = os.path.join(PATHFINDING_DIR, "png")
    os.makedirs(png_dir, exist_ok=True)

    name = Path(img_path).stem
    canonical_png = os.path.join(png_dir, f"{name}.png")

    ext = Path(img_path).suffix.lower()
    if ext == ".png":
        if os.path.abspath(img_path) != os.path.abspath(canonical_png):
            shutil.copyfile(img_path, canonical_png)
    else:
        im = cv2.imread(img_path, cv2.IMREAD_UNCHANGED)
        if im is None:
            raise RuntimeError(f"Uploaded image could not be read: {img_path}")
        cv2.imwrite(canonical_png, im)

    with _in_dir(PATHFINDING_DIR):
        graph, endpoints = img2graph(name, canvas_size=canvas_size)
        path = graph2path(graph, endpoints, canvas_size=canvas_size)
        os.makedirs(os.path.dirname(out_gcode_path), exist_ok=True)
        path2gcode(path, filename=out_gcode_path)

    return out_gcode_path

def _read_gcode_text(path_or_name: str) -> str:
    """Return G-code file contents as a UTF-8 string."""
    if os.path.isfile(path_or_name):
        p = path_or_name
    else:
        p = os.path.join(PATHFINDING_DIR, f"{path_or_name}.gcode")
        if not os.path.isfile(p):
            p = os.path.join(PATHFINDING_DIR, "gcode", f"{path_or_name}.gcode")
    if not os.path.isfile(p):
        raise FileNotFoundError(f"G-code not found: {path_or_name}")
    with open(p, "r", encoding="utf-8") as f:
        return f.read()

# -----------------------------------------------------------------------------#
# CV worker (kept for metrics/overlay). Optional legacy move streaming gated.
# -----------------------------------------------------------------------------#
_cv_thread: Optional[threading.Thread] = None

def _process_frame_once(frame) -> tuple[bool, float]:
    t0 = time.perf_counter()
    try:
        valid, _ = cvp.process_frame(frame)
    except Exception:
        try:
            st = cvp.get_state()
            if (not st["calibrated"]) or ((time.monotonic() % 2.0) < 0.05):
                cvp.calibrate_board(frame)
            cvp.update_bot(frame)
            st2 = cvp.get_state()
            valid = bool(st2["calibrated"] and st2["bot_pose"] is not None)
        except Exception:
            valid = False
    return bool(valid), (time.perf_counter() - t0)

def _wait_for_pose_change(prev_xy: Optional[tuple[float, float]], min_mm: float = 2.0, timeout_s: float = 2.0) -> Optional[tuple[tuple[float,float], float, float]]:
    """Block until CV reports a sufficiently changed pose or timeout.
    Returns the latest pose tuple on success, or None on timeout.
    """
    t_end = time.time() + max(0.2, float(timeout_s))
    cam = get_cam()
    last = prev_xy
    while time.time() < t_end:
        if cam is not None:
            frame = cam.get_latest_good()
            if frame is not None:
                try:
                    _process_frame_once(frame)
                except Exception:
                    pass
        pose = cvp.get_pose_mm()
        if pose and last:
            (cx, cy), _, _ = pose
            if math.hypot(cx - last[0], cy - last[1]) >= float(min_mm):
                return pose
        elif pose:
            return pose
        time.sleep(0.04)
    return None

def _cv_worker():
    try:
        bt.connect()
    except Exception:
        pass

    cam = get_cam()
    # Legacy move streaming is removed; BLE now only receives full G-code packets.

    while True:
        # if not bt.client or not bt.client.is_connected:
        # try:
        #     bt.connect()
        # except Exception:
        #     time.sleep(1.0)
        #     continue
        if cam is None:
            time.sleep(0.02)
            cam = get_cam()
            continue

        frame = cam.get_latest_good()
        if frame is None:
            time.sleep(0.0004)
            continue

        valid, tproc = _process_frame_once(frame)
        MET.note(valid, tproc)

        time.sleep(0.004)

def _ensure_cv_thread():
    global _cv_thread
    if _cv_thread is None:
        _cv_thread = threading.Thread(target=_cv_worker, daemon=True)
        _cv_thread.start()

# -----------------------------------------------------------------------------#
# Overlay/Stream
# -----------------------------------------------------------------------------#
def overlay_fn():
    def _draw(frame):
        st = cvp.get_state()
        line1 = f"Mode: {STATE['mode'].upper()}"
        if st["calibrated"]:
            try:
                line1 += f" | reproj: {st['board_reproj_err_px']:.2f}"
            except Exception:
                pass
        else:
            line1 += " | CALIBRATING..."
        run_txt = f"Run: {'ON' if RUN.active else 'OFF'} {RUN.follower.index()}/{RUN.follower.total()}"
        line2 = f"Rate: {MET.rate_hz():.1f} Hz | Median cam→pose: {MET.median_latency_ms():.0f} ms | Uptime: {MET.uptime_pct:.1f}% | {run_txt}"
        cv2.putText(frame, line1, (16, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(frame, line2, (16, 64), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2, cv2.LINE_AA)
        return frame
    return _draw

def gen_frames(overlay_factory, quality=65):
    _ensure_cv_thread()
    draw = overlay_factory() if callable(overlay_factory) else overlay_factory
    boundary = b"--frame\r\n"
    last_ts = 0.0

    while True:
        cam = get_cam()
        if cam is None:
            time.sleep(0.02); continue
        frame, ts = cam.get_latest_with_ts()
        if frame is None:
            time.sleep(0.005); continue
        if ts <= last_ts:
            time.sleep(0.002); continue
        last_ts = ts

        if draw:
            frame = draw(frame)

        jpg = encode_jpeg(frame, quality=max(40, min(int(quality), 90)))
        if jpg is None:
            continue
        yield boundary + b"Content-Type: image/jpeg\r\nCache-Control: no-store\r\n\r\n" + jpg + b"\r\n"

@app.after_request
def _no_buffer(resp):
    resp.headers["Cache-Control"] = "no-store, no-cache, must-revalidate, max-age=0"
    resp.headers["Pragma"] = "no-cache"
    resp.headers["X-Accel-Buffering"] = "no"
    return resp

# -----------------------------------------------------------------------------#
# Routes
# -----------------------------------------------------------------------------#
@app.route("/")
def home():
    _ensure_cv_thread()
    STATE["mode"] = "home"
    return render_template("index.html", page="home")

@app.route("/erase", methods=["GET", "POST"])
def erase():
    _ensure_cv_thread()
    STATE["mode"] = "erase"
    msg = None; error = None

    if request.method == "POST":
        action = request.form.get("action", "start")
        if action == "stop":
            RUN.stop()
            return redirect(url_for("erase"))

        # 1) Build normalized path for local preview/control (unchanged)
        raw_norm = _load_named_gcode_normalized("erase")
        if not raw_norm:
            error = "No erase path found in pathfinding directory."
        else:
            fitted = cvp.fit_path_to_board(raw_norm, margin_frac=0.10)
            if not fitted:
                error = "Board not calibrated yet."
            else:
                # Home the bot to starting position first
                if not _home_bot():
                    error = "Failed to home bot to starting position"
                else:
                    try:
                        gcode_str = _read_gcode_text("erase")
                    
                        firmware_gcode = cvp.build_firmware_gcode(gcode_str, canvas_size=PATH_CANVAS_SIZE)
                        
                        # Send with CV correction enabled
                        _send_with_cv_correction(firmware_gcode, fitted, window_size=5)
                    except Exception as e:
                        error = f"Failed to send G-code: {e}"

                RUN.load(_as_waypoints(fitted))
                RUN.start()
                msg = f"Erase queued with {len(fitted)} vertices and sent over BLE with CV correction."

        return render_template("index.html", page="erase", msg=msg, error=error)

    return render_template("index.html", page="erase", msg=msg, error=error)

@app.route("/draw", methods=["GET", "POST"])
def draw():
    _ensure_cv_thread()
    STATE["mode"] = "draw"
    msg = None; error = None

    if request.method == "POST":
        action = request.form.get("action", "start")
        if action == "stop":
            RUN.stop()
            return redirect(url_for("draw"))

        uploaded_file = request.files.get("file")
        raw_norm: List[Waypoint] = []
        gcode_path: Optional[str] = None

        if uploaded_file and uploaded_file.filename:
            fname = secure_filename(uploaded_file.filename)
            img_path = os.path.join("uploads", fname)
            uploaded_file.save(img_path)
            STATE["target_image"] = fname
            try:
                base = os.path.splitext(fname)[0]
                gcode_path = os.path.join("uploads", f"{base}.gcode")
                gcode_path = _convert_image_to_gcode(img_path, gcode_path, canvas_size=(575, 730))
                ops = load_gcode_file(gcode_path)
                raw_norm = _ops_to_normalized_wps(ops)
            except Exception as e:
                raw_norm = []
                error = f"Image conversion failed: {e}"

        if not raw_norm:
            fallback = _load_named_gcode_normalized("draw")
            if not fallback and error is None:
                error = "No draw path found and no valid uploaded image."
            raw_norm = fallback

        if raw_norm:
            # Removed mm_per_px dependency: assume board scale already established via central calibration
            fitted = cvp.fit_path_to_board(raw_norm, margin_frac=0.10)
            if not fitted:
                error = "Board not calibrated yet (no homography or scale)."
            else:
                # Home the bot to starting position first
                if not _home_bot():
                    error = "Failed to home bot to starting position"
                else:
                    # Send raw G-code to MCU (use generated file if present, else named fallback)
                    try:
                        src_for_gcode = gcode_path if gcode_path else "draw"
                        gcode_str = _read_gcode_text(src_for_gcode)
                    
                    # Normalize and scale to firmware G-code using current calibration
                        firmware_gcode = cvp.build_firmware_gcode(gcode_str, canvas_size=PATH_CANVAS_SIZE)
                    
                    # Send with CV correction enabled
                        _send_with_cv_correction(firmware_gcode, fitted, window_size=5)
                    except Exception as e:
                        error = f"Failed to send G-code: {e}"

                RUN.load(_as_waypoints(fitted))
                RUN.start()
                msg = f"Draw queued with {len(fitted)} vertices and sent over BLE with CV correction."

        return render_template("index.html", page="draw",
                               has_target=bool(STATE["target_image"]),
                               target=STATE["target_image"],
                               msg=msg, error=error)

    return render_template("index.html", page="draw",
                           has_target=bool(STATE["target_image"]),
                           target=STATE["target_image"],
                           msg=msg, error=error)

@app.route("/stream")
def stream():
    q = int(request.args.get("q", STREAM_JPEG_QUALITY))
    q = max(40, min(q, 80))
    return Response(gen_frames(overlay_fn, quality=q),
                    mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/home", methods=["POST"]) 
def home_robot():
    _ensure_cv_thread()
    
    # Wait for calibration AND scale to be ready (up to 20 seconds for robot markers)
    timeout = 20.0
    start_time = time.time()
    last_log_time = 0.0
    
    while time.time() - start_time < timeout:
        state = cvp.get_state()
        
        # Log progress every 2 seconds
        if time.time() - last_log_time > 2.0:
            if not state["calibrated"]:
                print(f"HOME: Waiting for board calibration... ({time.time() - start_time:.1f}s elapsed)")
            last_log_time = time.time()
        
        if state["calibrated"]:
            print(f"HOME: Ready! Calibration and scale complete after {time.time() - start_time:.1f}s")
            break
        time.sleep(0.2)
    
    state = cvp.get_state()
    if not state["calibrated"]:
        return {"success": False, "error": "Board not calibrated after 20s timeout"}, 400
    # Removed mm_per_px dependency/error path
    
    ok = _home_bot()
    if ok:
        return {"success": True}, 200
    return {"success": False, "error": "homing failed"}, 500

@app.route("/test_send", methods=["POST"])
def test_send():
    """Send G-code directly without pathfinding pipeline (for testing)"""
    _ensure_cv_thread()
    try:
        gcode_file = request.form.get("gcode_file", "uploads/test_simple_square.gcode")
        
        if not os.path.isfile(gcode_file):
            return {"error": f"File not found: {gcode_file}"}, 404
        
        with open(gcode_file, 'r') as f:
            gcode_str = f.read()
        
        # Initialize CV scale and board size before sending
        if not cvp.get_state()["calibrated"]:
            return {"error": "Board not calibrated - wait for calibration to lock"}, 400
        
        # Trigger scale calculation and board size logging
        board_size = cvp.board_size_mm()
        if not board_size:
            return {"error": "No board scale available - robot markers not detected"}, 400
        
        bounds = cvp.get_board_bounds_mm()
        if bounds:
            min_x, min_y, max_x, max_y = bounds
            margin = max(CV_BOARD_MARGIN_MM, CV_BOARD_MARGIN_FRAC * min(max_x - min_x, max_y - min_y))
            print(f"TEST_SEND: Board {board_size[0]:.1f}mm × {board_size[1]:.1f}mm, margins={margin:.1f}mm")
            print(f"TEST_SEND: Safe zone ({min_x+margin:.1f},{min_y+margin:.1f}) to ({max_x-margin:.1f},{max_y-margin:.1f})")
        
        # Just normalize, don't scale (test files should already be in mm)
        normalized = convert_pathfinding_gcode(gcode_str)
        
        print(f"Sending test G-code from {gcode_file}:")
        for i, line in enumerate(normalized.split('\n')[:10]):
            if line.strip():
                print(f"  {i}: {line}")
        
        # Send with strict sequencing to avoid MCU aggregating vectors
        bt.send_gcode_windowed(normalized, window_size=1)
        
        return {"success": True, "lines_sent": len([l for l in normalized.split('\n') if l.strip()])}
    except Exception as e:
        return {"error": str(e)}, 500

def _shutdown():
    try:
        RUN.stop()
        bt.close()  # Use close() instead of stop()
    except Exception:
        pass
    cam = get_cam()
    if cam:
        cam.close()
    os._exit(0)

if __name__ == "__main__":
    try: faulthandler.enable()
    except Exception: pass
    _ensure_cv_thread()
    app.run(host="0.0.0.0", port=int(os.getenv("PORT", "5000")), threaded=True, debug=False)
