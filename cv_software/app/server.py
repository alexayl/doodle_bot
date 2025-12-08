from __future__ import annotations

import os, sys, time, math, threading, shutil
from pathlib import Path
from contextlib import contextmanager
from typing import List, Tuple, Optional

# Load environment variables from .env file
from dotenv import load_dotenv
load_dotenv()

import cv2
from flask import Flask, Response, request, render_template, redirect, url_for
from werkzeug.utils import secure_filename
import faulthandler

from app.cv_core import CVPipeline
from app.cv_core import BOARD_MARGIN_MM as CV_BOARD_MARGIN_MM, BOARD_MARGIN_FRAC as CV_BOARD_MARGIN_FRAC
from app.control import PathRun, Waypoint
from app.camera import ThreadedCamera, CameraConfig
from app.path_parse import load_gcode_file
from app.bt_link import BTLink
from app.utils import encode_jpeg, Metrics

# -----------------------------------------------------------------------------#
# Config
# -----------------------------------------------------------------------------#
USE_TURBOJPEG = True  # encode_jpeg handles fallback
CAMERA_SRC = int(os.getenv("CAMERA_SRC", "0"))

# Base directory for cv_software (parent of app/)
CV_SOFTWARE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

PATHFINDING_DIR = os.getenv(
    "PATHFINDING_DIR",
    os.path.abspath(os.path.join(CV_SOFTWARE_DIR, "..", "pathfinding")),
)

# Absolute path to uploads directory
UPLOADS_DIR = os.path.join(CV_SOFTWARE_DIR, "uploads")

STREAM_JPEG_QUALITY = 75
USE_CV_CORRECTION = bool(int(os.getenv("USE_CV_CORRECTION", "1")))  # Toggle CV correction on/off (default from env)

# Correction tuning via environment (with sensible defaults)
def _env_float(name: str, default: float) -> float:
    try:
        return float(os.getenv(name, str(default)))
    except Exception:
        return default

CORR_MAX_MM = _env_float("CORR_MAX_MM", 5.0)
CORR_MIN_MM = _env_float("CORR_MIN_MM", 1.0)
POSE_MAX_AGE_S = _env_float("POSE_MAX_AGE_S", 3.0)
ALIGN_ENABLE_MM = _env_float("ALIGN_ENABLE_MM", 60.0)
# When pose is slightly stale (age <= POSE_MAX_AGE_S * CACHED_POSE_MAX_FACTOR)
# allow a reduced-strength correction instead of skipping completely.
CACHED_POSE_SCALE = _env_float("CACHED_POSE_SCALE", 0.5)
CACHED_POSE_MAX_FACTOR = _env_float("CACHED_POSE_MAX_FACTOR", 2.0)


app = Flask(__name__, template_folder="../templates", static_folder="../static")
os.makedirs(UPLOADS_DIR, exist_ok=True)

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
bt = BTLink()

# Default orientation derived from board marker layout:
# - board X increases to the right (matches firmware)
# - board Y increases toward the physical top marker, while firmware Y increases downward.
# Axis signs auto-adjust in-flight if CV sees the robot responding opposite of the command.
AXIS_SIGN = {"X": 1, "Y": -1}
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
    """Convert sequence to waypoints with validation."""
    out: List[Waypoint] = []
    if not seq:
        return out
    import math
    for i, item in enumerate(seq):
        try:
            if isinstance(item, Waypoint):
                # Validate existing waypoint
                if math.isfinite(item.x_mm) and math.isfinite(item.y_mm):
                    out.append(item)
                else:
                    print(f"[AS_WAYPOINTS] WARNING: Skipping invalid waypoint {i}: ({item.x_mm}, {item.y_mm})")
            elif isinstance(item, (tuple, list)) and len(item) >= 2:
                x = float(item[0])
                y = float(item[1])
                if math.isfinite(x) and math.isfinite(y):
                    out.append(Waypoint(x_mm=x, y_mm=y))
                else:
                    print(f"[AS_WAYPOINTS] WARNING: Skipping invalid waypoint {i}: ({x}, {y})")
            else:
                print(f"[AS_WAYPOINTS] WARNING: Skipping invalid waypoint item {i}: {type(item)}")
        except (ValueError, TypeError, IndexError) as e:
            print(f"[AS_WAYPOINTS] ERROR: Failed to convert waypoint {i}: {e}")
    return out

# -----------------------------------------------------------------------------#
# Board delta → firmware steps (auto sign adjustment)
# -----------------------------------------------------------------------------#

def _map_board_to_robot_steps(bx: float, by: float, min_step: int = 1) -> Optional[Tuple[int, int]]:
    """Convert a small board delta (mm) into firmware step commands.

    Board coordinates:
      +X to the right
      +Y toward the physical top marker

    Firmware coordinates:
      +X to the right
      +Y downward

    AXIS_SIGN tracks any motor reversals relative to this convention.
    """
    pose = cvp.get_pose_mm()
    bounds = cvp.get_board_bounds_mm()

    if pose and bounds:
        (cx, cy), _, _ = pose
        min_x, min_y, max_x, max_y = bounds
        # Allow a small epsilon beyond computed safe bounds to avoid over-rejecting tiny corrections
        SAFETY_EPS_MM = 2.0
        min_x_eps = min_x - SAFETY_EPS_MM
        max_x_eps = max_x + SAFETY_EPS_MM
        min_y_eps = min_y - SAFETY_EPS_MM
        max_y_eps = max_y + SAFETY_EPS_MM
        final_x = cx + bx
        final_y = cy + by

        if final_x < min_x_eps or final_x > max_x_eps or final_y < min_y_eps or final_y > max_y_eps:
            print(
                f"AXIS SAFETY: Rejected move by ({bx:.1f},{by:.1f})mm "
                f"from ({cx:.1f},{cy:.1f}) to ({final_x:.1f},{final_y:.1f}) "
                f"outside bounds [{min_x:.1f}-{max_x:.1f}, {min_y:.1f}-{max_y:.1f}] (±{SAFETY_EPS_MM:.1f}mm eps)"
            )
            return None
    else:
        print("AXIS WARNING: No pose or bounds available in _map_board_to_robot_steps")

    def _quantize(val: float) -> int:
        if abs(val) < 1e-3:
            return 0
        mag = max(min_step, int(round(abs(val))))
        return mag if val >= 0 else -mag

    cmd_x = float(bx)
    cmd_y = float(by)
    sx = _quantize(cmd_x)
    sy = _quantize(cmd_y)

    print(
        f"STEP MAP: board_delta=({bx:.1f},{by:.1f}) "
        f"axis_sign=(X{AXIS_SIGN['X']:+d},Y{AXIS_SIGN['Y']:+d}) "
        f"cmd=(X{cmd_x:.1f},Y{cmd_y:.1f}) steps=(X{sx},Y{sy})"
    )

    if sx == 0 and sy == 0:
        return None
    return sx, sy



def _refresh_pose_from_cam(cam: Optional[ThreadedCamera]) -> Optional[tuple]:
    """Single CV update from the camera feed, returning the latest pose."""
    if cam is not None:
        frame = cam.get_latest_good()
        if frame is not None:
            try:
                cvp.update_bot(frame)
            except Exception:
                pass
    return cvp.get_pose_mm()


def _wait_for_pose(timeout_s: float) -> Optional[tuple]:
    """Wait until a pose is detected or timeout expires."""
    deadline = time.time() + max(0.2, timeout_s)
    cam = get_cam()
    start = time.time()
    attempts = 0
    last_report = start
    
    if cam is None:
        print("[POSE WAIT] ERROR: Camera not initialized")
        return None
    
    print(f"[POSE WAIT] Starting pose detection (timeout={timeout_s:.1f}s)")
    
    while time.time() < deadline:
        attempts += 1
        elapsed = time.time() - start
        
        # Get frame and check quality
        frame = cam.get_latest_good()
        if frame is None:
            if elapsed - (last_report - start) >= 2.0:
                stats = cam.get_stats()
                print(f"[POSE WAIT] {elapsed:.1f}s: No frame from camera (attempt {attempts})")
                print(f"  Camera stats: has_frame={stats.get('has_frame')}, has_good_frame={stats.get('has_good_frame')}, latest_ts={stats.get('latest_ts', 0):.2f}s")
                last_report = time.time()
            time.sleep(0.05)
            continue
        
        # Try to update bot pose
        try:
            cvp.update_bot(frame)
            pose = cvp.get_pose_mm()
            if pose:
                print(f"[POSE WAIT] SUCCESS: Detected pose at ({pose[0][0]:.1f}, {pose[0][1]:.1f})mm after {elapsed:.1f}s ({attempts} attempts)")
                return pose
            else:
                # Check if board is calibrated
                state = cvp.get_state()
                if not state.get("calibrated", False):
                    if elapsed - (last_report - start) >= 2.0:
                        print(f"[POSE WAIT] {elapsed:.1f}s: Board not calibrated (attempt {attempts})")
                        last_report = time.time()
                elif elapsed - (last_report - start) >= 2.0:
                    print(f"[POSE WAIT] {elapsed:.1f}s: Board calibrated but bot markers not detected (attempt {attempts})")
                    last_report = time.time()
        except Exception as e:
            if elapsed - (last_report - start) >= 2.0:
                print(f"[POSE WAIT] {elapsed:.1f}s: CV update error: {e}")
                last_report = time.time()
        
        time.sleep(0.05)
    
    # Final diagnostics on timeout
    stats = cam.get_stats()
    state = cvp.get_state()
    print(f"[POSE WAIT] TIMEOUT: No pose detected after {timeout_s:.1f}s ({attempts} attempts)")
    print(f"  Final camera stats: has_frame={stats.get('has_frame')}, has_good_frame={stats.get('has_good_frame')}")
    print(f"  Board calibrated: {state.get('calibrated', False)}")
    return None


def _detect_axis_signs(move_mm: float = 12.0, timeout_s: float = 3.0):
    """
    Detect firmware axis direction for +X and +Y using simple BLE sends.
    Uses G1 moves and observes motion via CV, then sets AXIS_SIGN.
    """

    print("AXIS DETECT: Starting axis detection")

    pose = _wait_for_pose(5.0)
    if not pose:
        print("AXIS DETECT FAIL: No pose available.")
        return False

    (x0, y0), _, _ = pose

    # ------------------ Detect X sign ------------------
    print("AXIS DETECT: probing +X ...")
    bt.send_gcode(f"G1 X{int(move_mm)} Y0")
    p2 = _wait_for_pose_change((x0, y0), min_mm=2.0, timeout_s=timeout_s)

    if not p2:
        print("AXIS DETECT WARNING: No movement detected on +X probe. Assuming +X is forward.")
        AXIS_SIGN["X"] = +1
        x1, y1 = x0, y0
    else:
        (x1, y1), _, _ = p2
        if x1 > x0:
            AXIS_SIGN["X"] = +1
            print("AXIS DETECT: +X increases X → AXIS_SIGN[X] = +1")
        else:
            AXIS_SIGN["X"] = -1
            print("AXIS DETECT: +X decreases X → AXIS_SIGN[X] = -1")

    # Move back to original X
    bt.send_gcode(f"G1 X{-AXIS_SIGN['X'] * int(move_mm)} Y0")
    _wait_for_pose_change((x1, y1), min_mm=1.0, timeout_s=timeout_s)

    # ------------------ Detect Y sign ------------------
    pose = _wait_for_pose(2.0)
    if not pose:
        print("AXIS DETECT FAIL: Lost pose before Y probe.")
        return False
    (x0, y0), _, _ = pose

    print("AXIS DETECT: probing +Y ...")
    bt.send_gcode(f"G1 X0 Y{int(move_mm)}")
    p3 = _wait_for_pose_change((x0, y0), min_mm=2.0, timeout_s=timeout_s)

    if not p3:
        print("AXIS DETECT WARNING: No movement detected on +Y probe. Assuming +Y is forward.")
        AXIS_SIGN["Y"] = +1
        x2, y2 = x0, y0
    else:
        (x2, y2), _, _ = p3
        if y2 > y0:
            AXIS_SIGN["Y"] = +1
            print("AXIS DETECT: +Y increases Y → AXIS_SIGN[Y] = +1")
        else:
            AXIS_SIGN["Y"] = -1
            print("AXIS DETECT: +Y decreases Y → AXIS_SIGN[Y] = -1")

    # Move back to original Y
    bt.send_gcode(f"G1 X0 Y{-AXIS_SIGN['Y'] * int(move_mm)}")
    _wait_for_pose_change((x2, y2), min_mm=1.0, timeout_s=timeout_s)

    print(f"AXIS DETECT COMPLETE: AXIS_SIGN = {AXIS_SIGN}")
    return True


# -----------------------------------------------------------------------------#
# G-code helpers
# -----------------------------------------------------------------------------#
def _ops_to_normalized_wps(
    ops: List[tuple],
    return_origin: bool = False,
) -> List[Waypoint] | tuple[List[Waypoint], tuple[float, float]]:
    """
    Convert ("move", dx, dy) incremental ops to [0..1] normalized waypoints.
    """
    x = y = 0.0
    pts: List[tuple[float, float]] = []
    for op in ops:
        if not op:
            continue
        if op[0] == "move":
            x += float(op[1])
            y += float(op[2])
            pts.append((x, y))
    if not pts:
        return []
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    minx, maxx, miny, maxy = min(xs), max(xs), min(ys), max(ys)
    w = max(maxx - minx, 1e-6)
    h = max(maxy - miny, 1e-6)
    origin_norm = ((0.0 - minx) / w, (0.0 - miny) / h)

    out: List[Waypoint] = []
    for px, py in pts:
        nx = (px - minx) / w
        ny = (py - miny) / h
        out.append(Waypoint(x_mm=float(nx), y_mm=float(ny)))

    if return_origin:
        return out, origin_norm
    return out


def _load_named_gcode_normalized(
    name: str,
    return_origin: bool = False,
) -> List[Waypoint] | tuple[List[Waypoint], tuple[float, float]]:
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
        return ([], (0.0, 0.0)) if return_origin else []
    norm = _ops_to_normalized_wps(ops, return_origin=return_origin)
    return norm

def _home_bot(
    inset_mm: float = 20.0,
    step_coarse: float = 35.0,
    step_fine: float = 12.0,
    arrival_mm: float = 15.0,
    timeout_s: float = 45.0,
) -> bool:
    """
    bounded homing toward top-left corner.
    Uses board-frame deltas only.
    NO probes, NO heading, NO sign guessing.
    All moves are safety-checked through _map_board_to_robot_steps().
    """

    st = cvp.get_state()
    if not st["calibrated"]:
        print("HOME FAIL: Board not calibrated")
        return False

    min_x, min_y, max_x, max_y = cvp.get_board_bounds_mm()
    tx = min_x + inset_mm
    ty = max_y - inset_mm
    print(f"HOME: TL target = ({tx:.1f},{ty:.1f})mm")

    pose = cvp.get_pose_mm()
    if not pose:
        pose = _wait_for_pose(10)
    if not pose:
        print("HOME FAIL: No initial pose")
        return False
    if not _detect_axis_signs():
        print("HOME FAIL: Could not detect axis directions.")
        return False
    (cx, cy), _, _ = pose
    print(f"HOME: Start pose=({cx:.1f},{cy:.1f})")

    # ---- MAIN LOOP ----
    t0 = time.time()
    stuck = 0
    max_stuck = 4

    while time.time() - t0 < timeout_s:

        pose = cvp.get_pose_mm()
        if not pose:
            print("HOME: pose lost, waiting…")
            time.sleep(0.05)
            continue

        (cx, cy), _, _ = pose

        dx = tx - cx
        dy = ty - cy
        dist = math.hypot(dx, dy)
        print(f"HOME: pos=({cx:.1f},{cy:.1f}) dist={dist:.1f}")

        if dist < arrival_mm:
            print("HOME ✓ reached TL target")
            return True

        step = step_coarse if dist > 120 else step_fine
        step = max(6, min(step, dist * 0.45))

        norm = math.hypot(dx, dy)
        ux = dx / norm
        uy = dy / norm

        bx = ux * step
        by = uy * step

        steps = _map_board_to_robot_steps(bx, by)
        if steps is None:
            print("HOME: Step rejected by safety map (would exit board). Stopping.")
            return True

        sx, sy = steps
        cmd = f"G1 X{sx} Y{sy}"
        print(f"HOME → {cmd}")
        bt.send_gcode(cmd)

        before = (cx, cy)
        p2 = _wait_for_pose_change(before, 1.5, 3.5)
        if not p2:
            stuck += 1
            print(f"HOME WARN: stuck {stuck}/{max_stuck}")
            if stuck >= max_stuck:
                print("HOME FAIL: robot not moving")
                return False
        else:
            print("HOME: movement confirmed")
            stuck = 0

    print("HOME FAIL: timeout")
    return False

def _send_with_cv_correction(
    firmware_gcode: str,
    fitted_wps: List[Waypoint],
    start_pos: Optional[Tuple[float, float]] = None,
    window_size: int = 1,
):
    """Send G-code with optional real-time CV correction.
    
    If USE_CV_CORRECTION is False, just sends the G-code directly without correction callbacks.
    """
    if not USE_CV_CORRECTION:
        print("[SEND] CV correction disabled - sending G-code directly")
        bt.send_gcode_windowed(firmware_gcode, window_size=window_size)
        return

    print("[SEND] CV correction enabled")
    path_wps = _as_waypoints(fitted_wps)
    if not path_wps:
        raise RuntimeError("No fitted waypoints available for correction run.")
    
    # Validate waypoint count
    if len(path_wps) < 2:
        raise RuntimeError(f"Insufficient waypoints for path following: {len(path_wps)} (need at least 2)")
    # for i, wp in enumerate(path_wps):
    #     if not (isinstance(wp.x_mm, float) and isinstance(wp.y_mm, float)):
    #         raise RuntimeError(f"Invalid waypoint at index {i}: ({wp.x_mm}, {wp.y_mm})")
    #     else:
    #         print(f"[SEND] Waypoint {i}: ({wp.x_mm:.1f}, {wp.y_mm:.1f})mm")
    print(f"[SEND] Validated {len(path_wps)} waypoints for path following")

    # Safety tuning: limit per-injection correction magnitude (mm)
    MAX_CORR_MM = CORR_MAX_MM
    MIN_CORR_MM = CORR_MIN_MM

    MAX_INCREASING_CORRECTIONS = 5
    last_deviation = None
    increasing_count = 0
    correction_disabled = False
    
    # Apply corrections periodically, not on every instruction
    CORRECTION_INTERVAL = 5  # Apply correction every N instructions
    correction_counter = 0
    corrected_pids = set()

    bounds = cvp.get_board_bounds_mm()
    
    # Validate waypoints before loading
    if not path_wps:
        raise RuntimeError("No waypoints provided for path run")
    
    # Load waypoints with error handling
    if not RUN.load(path_wps, bounds=bounds):
        raise RuntimeError("Failed to load waypoints into path follower")
    
    # Verify waypoint count matches
    if RUN.follower.total() != len(path_wps):
        print(f"[SEND] WARNING: Waypoint count mismatch: loaded {RUN.follower.total()}, provided {len(path_wps)}")
    
    # Start path following
    if not RUN.start():
        raise RuntimeError("Failed to start path follower")
    
    cvp.correction.reset()

    lines = firmware_gcode.splitlines()
    expected_pos_after_pid: List[Optional[Tuple[float, float]]] = [None] * len(lines)
    try:
        import re
        # Start from anchor (fitted_wps[0] now contains bot position after replacement)
        if fitted_wps and len(fitted_wps) > 0:
            pos_x = float(fitted_wps[0].x_mm)
            pos_y = float(fitted_wps[0].y_mm)
            print(f"[EXPECT] Using anchor from fitted waypoints[0]: ({pos_x:.1f},{pos_y:.1f})")
        elif start_pos is not None:
            pos_x, pos_y = start_pos[0], start_pos[1]
            print(f"[EXPECT] Using start_pos as anchor: ({pos_x:.1f},{pos_y:.1f})")
        else:
            pos_x = pos_y = 0.0
            print(f"[EXPECT] No anchor available, using origin")
        
        # Now integrate all moves (firmware coordinates → board coordinates)
        # Clamp to bounds to prevent out-of-bounds expected positions
        bounds_for_clamp = cvp.get_board_bounds_mm()
        min_x_bound = bounds_for_clamp[0] if bounds_for_clamp else 0.0
        min_y_bound = bounds_for_clamp[1] if bounds_for_clamp else 0.0
        max_x_bound = bounds_for_clamp[2] if bounds_for_clamp else 800.0
        max_y_bound = bounds_for_clamp[3] if bounds_for_clamp else 400.0
        
        for i, raw in enumerate(lines):
            line = raw.strip()
            upper = line.upper()
            
            # Parse incremental G0/G1 moves and integrate
            if upper.startswith("G0") or upper.startswith("G1"):
                dx_match = re.search(r'\bX\s*([-+]?\d+)', upper)
                dy_match = re.search(r'\bY\s*([-+]?\d+)', upper)
                
                # Apply axis signs to convert firmware → board coordinates
                if dx_match:
                    firmware_dx = float(dx_match.group(1))
                    pos_x += firmware_dx * AXIS_SIGN['X']
                if dy_match:
                    firmware_dy = float(dy_match.group(1))
                    pos_y += firmware_dy * AXIS_SIGN['Y']
                
                # Clamp to bounds to prevent invalid expected positions
                pos_x = max(min_x_bound, min(pos_x, max_x_bound))
                pos_y = max(min_y_bound, min(pos_y, max_y_bound))
            
            expected_pos_after_pid[i] = (pos_x, pos_y)
    except Exception as e:
        print(f"[EXPECT] Failed to compute expected positions: {e}")
        pass

    def correction_callback(head_pid: int, head_line: str, next_line: Optional[str]) -> List[str]:
        nonlocal last_deviation, increasing_count, correction_disabled, correction_counter, corrected_pids
        
        if not RUN.active:
            return []
        
        if correction_disabled:
            print("[CORRECTION] DISABLED due to runaway behavior")
            return []
        
        # Only correct periodically (every CORRECTION_INTERVAL instructions)
        correction_counter += 1
        if correction_counter % CORRECTION_INTERVAL != 0:
            return []
        
        # Only correct each instruction once
        if head_pid in corrected_pids:
            return []
        
        print(f"[CORRECTION CALLBACK] called for pid={head_pid} (interval check passed)")
        try:
            if 0 <= head_pid < len(expected_pos_after_pid):
                exp = expected_pos_after_pid[head_pid]
                if exp is not None:
                    print(f"[EXPECT NEXT] pid={head_pid} after=({exp[0]:.1f},{exp[1]:.1f})")
        except Exception:
            pass

        # Get current index with validation
        current_idx = RUN.follower.index()
        total_wps = RUN.follower.total()
        
        # Validate index is within bounds
        if not RUN.follower.is_index_valid(current_idx):
            print(f"[CORRECTION] WARNING: Invalid waypoint index {current_idx}/{total_wps}, skipping correction")
            return []
        
        # Validate fitted_wps index is also valid
        if current_idx >= len(fitted_wps):
            print(f"[CORRECTION] WARNING: Index {current_idx} >= fitted_wps length {len(fitted_wps)}, skipping correction")
            return []
        
        next_is_g1 = bool(next_line and next_line.strip().startswith("G1"))
        # Compute distance to current target for better diagnostics
        dist_to_target = None
        try:
            tgt = RUN.follower.current_target()
            pose_dbg = cvp.get_pose_mm()
            if tgt is not None and pose_dbg:
                (cx_dbg, cy_dbg), _, _ = pose_dbg
                dist_to_target = math.hypot(float(tgt.x_mm) - float(cx_dbg), float(tgt.y_mm) - float(cy_dbg))
        except Exception as e:
            print(f"[CORRECTION] Error computing dist_to_target: {e}")
            dist_to_target = None

        print(
            f"[CORRECTION] follower at waypoint {current_idx}/{total_wps}"
            + (f" dist_to_target={dist_to_target:.1f}mm" if dist_to_target is not None else "")
        )
        
        # Verify current target is within bounds
        try:
            tgt = RUN.follower.current_target()
            if tgt is not None:
                bounds_chk = cvp.get_board_bounds_mm()
                if bounds_chk:
                    min_x_chk, min_y_chk, max_x_chk, max_y_chk = bounds_chk
                    if not (min_x_chk <= tgt.x_mm <= max_x_chk and min_y_chk <= tgt.y_mm <= max_y_chk):
                        print(
                            f"[CORRECTION] WARNING: Target waypoint {current_idx} out of bounds: "
                            f"({tgt.x_mm:.1f},{tgt.y_mm:.1f}) outside [{min_x_chk:.1f}-{max_x_chk:.1f}, {min_y_chk:.1f}-{max_y_chk:.1f}]"
                        )
                        return []  # Skip correction for out-of-bounds target
        except Exception as e_bounds:
            print(f"[CORRECTION] Bounds check failed: {e_bounds}")

        # Compute correction with validated index
        try:
            correction = cvp.correction.compute_correction(
                waypoints=fitted_wps,
                current_index=current_idx,
                next_is_g1=next_is_g1,
            )
        except Exception as e_corr:
            print(f"[CORRECTION] Error computing correction: {e_corr}")
            import traceback
            traceback.print_exc()
            return []
        print(f"[CORRECTION CALLBACK] correction={correction}")
        if correction is None:
            # Extra diagnostics when the correction engine returns None
            try:
                pose_age_dbg = cvp.pose_age_s()
            except Exception:
                pose_age_dbg = None
            try:
                dev_dbg = cvp.correction.rms_deviation()
            except Exception:
                dev_dbg = None
            print(
                "[CORRECTION] engine returned None "
                + (f"pose_age={pose_age_dbg:.2f}s " if pose_age_dbg is not None else "")
                + (f"rms_dev={dev_dbg:.2f}mm " if dev_dbg is not None else "")
                + f"idx={current_idx} next_is_g1={next_is_g1}"
            )
            return []

        # Only skip if pose is missing. If pose exists but is slightly stale,
        # allow a reduced-strength correction (cached-pose fallback).
        pose = cvp.get_pose_mm()
        if not pose:
            print("[CORRECTION] skipping: no current pose")
            return []

        pose_age = cvp.pose_age_s()
        using_cached_pose = False
        cached_scale = 1.0
        if pose_age is not None and pose_age > POSE_MAX_AGE_S:
            # If within allowable cached factor, use reduced-strength correction
            if pose_age <= (POSE_MAX_AGE_S * CACHED_POSE_MAX_FACTOR):
                using_cached_pose = True
                cached_scale = float(CACHED_POSE_SCALE)
                now = time.time()
                if now - getattr(cvp, "_last_cached_log", 0.0) > 2.0:
                    print(
                        f"[CORRECTION] using cached pose (age={pose_age:.2f}s) — scaling corrections by {cached_scale:.2f}"
                    )
                    cvp._last_cached_log = now
            else:
                print(f"[CORRECTION] skipping: pose very stale ({pose_age:.2f}s > {POSE_MAX_AGE_S:.2f}s)")
                return []

        bx, by = correction
        # Scale corrections down when relying on a cached (stale) pose
        if using_cached_pose and cached_scale != 1.0:
            bx *= cached_scale
            by *= cached_scale
        # Clamp very small jitters and very large spikes
        if abs(bx) < MIN_CORR_MM and abs(by) < MIN_CORR_MM:
            return []
        if abs(bx) > MAX_CORR_MM or abs(by) > MAX_CORR_MM:
            scale = min(MAX_CORR_MM / max(abs(bx), 1e-9), MAX_CORR_MM / max(abs(by), 1e-9))
            bx *= scale
            by *= scale
            print(f"[CORRECTION] clamped to ({bx:.2f},{by:.2f})mm")

        steps = _map_board_to_robot_steps(bx, by, min_step=1)
        if steps is None:
            return []

        sx, sy = steps
        if sx == 0 and sy == 0:
            return []
        
        current_dev = cvp.correction.rms_deviation()
        if last_deviation is not None and current_dev > last_deviation:
            increasing_count += 1
            print(f"[CORRECTION] WARNING: deviation increasing {increasing_count}/{MAX_INCREASING_CORRECTIONS} (was {last_deviation:.1f}mm, now {current_dev:.1f}mm)")
            if increasing_count >= MAX_INCREASING_CORRECTIONS:
                correction_disabled = True
                print("[CORRECTION] EMERGENCY STOP: Deviation increased for 5+ corrections. Disabling CV correction.")
                return []
        else:
            increasing_count = 0
        last_deviation = current_dev
        
        corrected_pids.add(head_pid)
        print(f"[CORRECTION] Injecting correction for pid={head_pid}, corrected_pids={len(corrected_pids)}")
        
        return [f"G1 X{int(sx)} Y{int(sy)}"]

    def on_head_executed(head_pid: int, head_line: str, ack_msg: str):
        pose = cvp.get_pose_mm()
        if not pose or not RUN.active:
            return

        # Expected endpoint after executing this pid (based on fitted waypoints)
        try:
            if 0 <= head_pid < len(expected_pos_after_pid):
                exp = expected_pos_after_pid[head_pid]
                if exp is not None:
                    (cx2, cy2), _, _ = pose
                    diff = math.hypot(exp[0] - float(cx2), exp[1] - float(cy2))
                    print(
                        f"[EXPECT] pid={head_pid} expected=({exp[0]:.1f},{exp[1]:.1f}) "
                        f"pose=({float(cx2):.1f},{float(cy2):.1f}) diff={diff:.1f}mm"
                    )
        except Exception:
            pass

        # Print expected vs actual before follower update
        try:
            (cx, cy), _, _ = pose
            pose_age_check = cvp.pose_age_s()
            cur_idx = RUN.follower.index()
            cur_tgt = RUN.follower.current_target()
            if cur_tgt is not None:
                dist = math.hypot(cur_tgt.x_mm - float(cx), cur_tgt.y_mm - float(cy))
                age_str = f" age={pose_age_check:.1f}s" if pose_age_check is not None else ""
                print(
                    f"[TARGET] pid={head_pid} line='{head_line.strip()}' idx={cur_idx} "
                    f"target=({cur_tgt.x_mm:.1f},{cur_tgt.y_mm:.1f}) pose=({float(cx):.1f},{float(cy):.1f}) dist={dist:.1f}mm{age_str}"
                )
        except Exception:
            pass

        try:
            reached, dev_mm = RUN.follower.update_and_check_reached(pose)
        except Exception as e:
            print(f"[FOLLOWER] update error: {e}")
            import traceback
            traceback.print_exc()
            # Don't return - continue processing even if update fails
            return

        if dev_mm is not None:
            dev_mm = float(dev_mm)
            cvp.correction.log_deviation(dev_mm)
            if cvp.correction.correction_count % 10 == 0:
                rms = cvp.correction.rms_deviation()
                print(f"[DEVIATION] inst={dev_mm:.2f}mm RMS={rms:.2f}mm")

        if reached:
            print(f"WAYPOINT_REACHED: {RUN.follower.index()}/{RUN.follower.total()}")
        else:
            # Also print next target after update for reference
            try:
                nxt_idx = RUN.follower.index()
                nxt_tgt = RUN.follower.current_target()
                if nxt_tgt is not None:
                    print(
                        f"[TARGET NEXT] idx={nxt_idx} target=({nxt_tgt.x_mm:.1f},{nxt_tgt.y_mm:.1f})"
                    )
            except Exception:
                pass

    try:
        bt.send_gcode_windowed(
            firmware_gcode,
            window_size=window_size,
            correction_cb=correction_callback,
            on_head_executed=on_head_executed,
        )
    finally:
        RUN.stop()



def _first_waypoint_pos(wps: List[Waypoint]) -> Optional[Tuple[float, float]]:
    if not wps:
        return None
    first = wps[0]
    print(f"FIRST_WP: ({first.x_mm:.1f},{first.y_mm:.1f})")
    return (first.x_mm, first.y_mm)

def _assert_fitted_in_bounds(fitted):
    bounds = cvp.get_board_bounds_mm()
    if not bounds:
        raise RuntimeError("Board bounds not available")
    min_x, min_y, max_x, max_y = bounds

    for i, wp in enumerate(fitted):
        x = wp.x_mm
        y = wp.y_mm
        if not (min_x <= x <= max_x and min_y <= y <= max_y):
            raise RuntimeError(
                f"Waypoint {i} out of bounds: ({x:.1f},{y:.1f}) "
                f"safe=({min_x:.1f}-{max_x:.1f}, {min_y:.1f}-{max_y:.1f})"
            )


def _convert_image_to_gcode(img_path: str, out_gcode_path: str, canvas_size=(800, 400)) -> str:
    """Run the pathfinding stack and write G-code next to the uploaded image."""
    print(f"[PATHFINDING] Starting conversion:")
    print(f"  Input image: {img_path}")
    print(f"  Output gcode: {out_gcode_path}")
    print(f"  PATHFINDING_DIR: {PATHFINDING_DIR}")
    
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
            print(f"[PATHFINDING] Copying PNG: {img_path} -> {canonical_png}")
            shutil.copyfile(img_path, canonical_png)
    else:
        print(f"[PATHFINDING] Converting {ext} to PNG: {canonical_png}")
        im = cv2.imread(img_path, cv2.IMREAD_UNCHANGED)
        if im is None:
            raise RuntimeError(f"Uploaded image could not be read: {img_path}")
        cv2.imwrite(canonical_png, im)

    print(f"[PATHFINDING] Running pathfinding stack in {PATHFINDING_DIR}...")
    with _in_dir(PATHFINDING_DIR):
        graph, endpoints = img2graph(name, granularity=3, canvas_size=canvas_size)
        print(f"[PATHFINDING] Graph generated with {len(graph)} nodes")
        path = graph2path(graph, endpoints, canvas_size=canvas_size)
        print(f"[PATHFINDING] Path generated with {len(path)} points")
        os.makedirs(os.path.dirname(out_gcode_path), exist_ok=True)
        path2gcode(path, filename=out_gcode_path)
        print(f"[PATHFINDING] GCODE GENERATED: {out_gcode_path}")
        if os.path.isfile(out_gcode_path):
            file_size = os.path.getsize(out_gcode_path)
            print(f"[PATHFINDING] G-code file size: {file_size} bytes")
            for i, line in enumerate(open(out_gcode_path, "r", encoding="utf-8")):
                if i < 10:  # Only print first 10 lines
                    print(f"GCODE: {line.strip()}")
        else:
            print(f"[PATHFINDING] ERROR: G-code file was not created!")
            raise RuntimeError(f"G-code file not created: {out_gcode_path}")

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
# CV worker (kept for metrics/overlay).
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
        # --- METRIC PRINTS -----------------------------------------------------
        if MET.count % 676767 == 0:
            median_lat_ms = MET.median_latency_ms()
            hz = MET.rate_hz()
            uptime = MET.uptime_pct

            st = cvp.get_state()
            conf = st.get("board_confidence", None)
            calibrated = st["calibrated"]
            if conf == 0:
                calibrated = False
            
            # Get path deviation metrics if path is active
            rms_dev = None
            try:
                if RUN.active:
                    rms_dev = cvp.correction.rms_deviation()
            except Exception:
                pass
            
            # Log metrics with optional RMS deviation
            # if rms_dev is not None:
            #     print(
            #         f"[METRICS] median_cam_pose_latency={median_lat_ms:.1f}ms, "
            #         f"update_rate={hz:.1f}Hz, uptime={uptime:.1f}%, "
            #         f"board_conf={conf if conf is not None else 'NA'}, calibrated={calibrated}, "
            #         f"path_rms_deviation={rms_dev:.2f}mm"
            #     )

            # Auto-recalibration check
            # if conf is not None and conf < 0.6:
            #     print("[METRICS] WARNING: marker confidence < 0.6 → reinitializing calibration")
            #     try:
            #         cvp.calibrate_board(frame)
            #     except Exception:
            #         print("[METRICS] ERROR: failed to reset calibration")
        time.sleep(0.001)

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
        # if st["calibrated"]:
        #     try:
        #         line1 += f" | reproj: {st['board_reproj_err_px']:.2f}"
        #     except Exception:
        #         pass
        # else:
        #     line1 += " | CALIBRATING..."
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
    """CV-based erase with automatic smudge detection and CV-corrected path following."""
    _ensure_cv_thread()
    STATE["mode"] = "erase"
    msg = None
    error = None

    if request.method == "POST":
        action = request.form.get("action", "start")
        if action == "stop":
            RUN.stop()
            return redirect(url_for("erase"))

        # Check if board is calibrated
        if cvp.cal.board_pose is None:
            error = "Board not calibrated yet. Please calibrate first."
            return render_template("index.html", page="erase", msg=msg, error=error)
        
        # Capture current frame for smudge detection
        cam = get_cam()
        if cam is None:
            error = "Camera not available"
            return render_template("index.html", page="erase", msg=msg, error=error)
        
        frame = cam.get_latest_good()
        if frame is None:
            error = "Failed to capture frame from camera"
            return render_template("index.html", page="erase", msg=msg, error=error)
        
        # Get current bot position
        bot_pose = cvp.get_pose_mm()
        if bot_pose:
            (bot_x, bot_y), _, _ = bot_pose
            current_position = (bot_x, bot_y)
            print(f"[ERASE] Bot position: ({bot_x:.1f}, {bot_y:.1f})mm")
        else:
            # Use center of board as fallback
            board_size = cvp.board_size_mm()
            if board_size:
                current_position = (board_size[0] / 2, board_size[1] / 2)
            else:
                current_position = (400, 200)
            print(f"[ERASE] No bot pose, using fallback: {current_position}")
        
        # Import smudge detection module
        from app.smudge_detection import generate_smudge_erase_gcode
        
        # Get detection parameters from form (with defaults)
        try:
            darkness_threshold = int(request.form.get("darkness_threshold", 200))
            min_smudge_area = float(request.form.get("min_smudge_area", 25.0))
            line_spacing = float(request.form.get("line_spacing", 5.0))
        except ValueError:
            darkness_threshold = 200
            min_smudge_area = 25.0
            line_spacing = 5.0
        
        # Detect smudges and generate G-code
        try:
            gcode_lines, smudge_contours = generate_smudge_erase_gcode(
                frame_bgr=frame,
                H_img2board=cvp.cal.board_pose.H_img2board,
                board_width_mm=800.0,
                board_height_mm=400.0,
                current_position=current_position,
                detector_params={
                    "darkness_threshold": darkness_threshold,
                    "min_smudge_area_mm2": min_smudge_area,
                },
                path_params={
                    "line_spacing_mm": line_spacing,
                },
            )
            
            if not gcode_lines:
                msg = "No smudges detected! Board appears clean."
                return render_template("index.html", page="erase", msg=msg, error=error)
            
            print(f"[ERASE] Detected {len(smudge_contours)} smudge(s)")
            print(f"[ERASE] Generated {len(gcode_lines)} G-code lines")
            
            # Convert G-code lines to waypoints for CV correction
            waypoints = []
            current_x, current_y = current_position
            pen_down = False
            
            for line in gcode_lines:
                line = line.strip()
                if not line or line.startswith(";"):
                    continue
                
                # Parse G1 commands
                if line.startswith("G1"):
                    x = current_x
                    y = current_y
                    
                    if "X" in line:
                        try:
                            x_str = line.split("X")[1].split()[0]
                            x = float(x_str)
                        except (IndexError, ValueError):
                            pass
                    
                    if "Y" in line:
                        try:
                            y_str = line.split("Y")[1].split()[0]
                            y = float(y_str)
                        except (IndexError, ValueError):
                            pass
                    
                    # Add waypoint if pen is down (erasing)
                    if pen_down:
                        waypoints.append(Waypoint(x_mm=x, y_mm=y))
                    
                    current_x = x
                    current_y = y
                
                # Track pen state (eraser down = 0, eraser up = 90)
                elif "M280 P0 S0" in line or "M280 P1 S0" in line:
                    pen_down = True
                elif "M280 P0 S90" in line or "M280 P1 S90" in line:
                    pen_down = False
            
            # Send G-code with CV correction enabled
            gcode_str = "\n".join(gcode_lines)
            
            if waypoints:
                print(f"[ERASE] Extracted {len(waypoints)} waypoints for CV correction")
                # Use CV correction for precise erasing (window_size=3 for rapid feedback)
                _send_with_cv_correction(gcode_str, waypoints, start_pos=current_position, window_size=3)
                msg = f"Detected {len(smudge_contours)} smudge(s). Generated {len(gcode_lines)} commands with {len(waypoints)} CV-corrected waypoints."
            else:
                # No waypoints (only pen lifts/drops), send directly without correction
                print("[ERASE] No waypoints extracted, sending commands directly")
                bt.send_gcode_windowed(gcode_str, window_size=3)
                msg = f"Detected {len(smudge_contours)} smudge(s). Sent {len(gcode_lines)} commands."
        
        except Exception as e:
            import traceback
            error = f"Failed to detect/erase smudges: {e}"
            print(f"[ERASE] ERROR: {traceback.format_exc()}")
            return render_template("index.html", page="erase", msg=msg, error=error)

        return render_template("index.html", page="erase", msg=msg, error=error)

    return render_template("index.html", page="erase", msg=msg, error=error)



@app.route("/erase_smudges", methods=["GET", "POST"])
def erase_smudges():
    """Legacy route - now redirects to /erase for CV-based smudge detection."""
    return redirect(url_for("erase"))



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
        norm_origin: Optional[Tuple[float, float]] = None
        gcode_path: Optional[str] = None

        if uploaded_file and uploaded_file.filename:
            fname = secure_filename(uploaded_file.filename)
            img_path = os.path.join(UPLOADS_DIR, fname)
            uploaded_file.save(img_path)
            STATE["target_image"] = fname
            try:
                base = os.path.splitext(fname)[0]
                gcode_path = os.path.join(UPLOADS_DIR, f"{base}.gcode")
                gcode_path = _convert_image_to_gcode(img_path, gcode_path, canvas_size=(900, 530))
                ops = load_gcode_file(gcode_path)
                raw_norm, norm_origin = _ops_to_normalized_wps(ops, return_origin=True)
            except Exception as e:
                raw_norm = []
                error = f"Image conversion failed: {e}"

        if not raw_norm:
            fallback_norm, fallback_origin = _load_named_gcode_normalized("draw", return_origin=False)
            if not fallback_norm and error is None:
                error = "No draw path found and no valid uploaded image."
            raw_norm = fallback_norm
            norm_origin = fallback_origin

        if raw_norm:
            fitted = cvp.fit_path_to_board(raw_norm, margin_frac=0.10)
            # print(f"FITTED: {len(fitted)} waypoints")
            # print(f"FIRST 5 WPS: {[f'({wp.x_mm:.1f},{wp.y_mm:.1f})' for wp in fitted[:5]]}")
            _assert_fitted_in_bounds(fitted)
            if not fitted:
                error = "Board not calibrated yet (no homography or scale)."
            else:
                # if not _home_bot():
                #     error = "Failed to home bot to starting position"
                # else:
                #     # Send raw G-code to MCU (use generated file if present, else named fallback)
                try:
                    src_for_gcode = gcode_path if gcode_path else "draw"
                    gcode_str = _read_gcode_text(src_for_gcode)
                    print(f"Sending G-code from {src_for_gcode}...")
                    
                    # Use actual bot position from CV as starting point (retry up to 10s)
                    bot_pose = _wait_for_pose(10.0)
                    if not bot_pose:
                        # Try to get cached pose if available
                        bot_pose = cvp.get_pose_mm()
                        if bot_pose:
                            print(f"[START] Using cached pose (no fresh detection)")
                    
                    # Get bounds for validation
                    bounds = cvp.get_board_bounds_mm()
                    board_size = cvp.board_size_mm()
                    
                    if board_size:
                        print(f"[BOARD] Physical size: {board_size[0]:.1f}mm × {board_size[1]:.1f}mm")
                    if bounds:
                        min_x, min_y, max_x, max_y = bounds
                        print(f"[BOARD] Safe drawing zone: [{min_x:.1f}-{max_x:.1f}, {min_y:.1f}-{max_y:.1f}]")
                    
                    if bot_pose:
                        (bot_x, bot_y), _, _ = bot_pose
                        print(f"[START] Bot detected at: ({bot_x:.1f}, {bot_y:.1f})mm")
                        
                        if bounds:
                            min_x, min_y, max_x, max_y = bounds
                            if not (min_x <= bot_x <= max_x and min_y <= bot_y <= max_y):
                                print(f"[START] WARNING: Bot position OUT OF BOUNDS [{min_x:.1f}-{max_x:.1f}, {min_y:.1f}-{max_y:.1f}]")
                                print(f"[START] Using bot position anyway - robot must start from where it is!")
                        
                        # Always use actual bot position, even if out of bounds
                        # The robot HAS to start from where it physically is
                        start_pos = (bot_x, bot_y)
                        fitted[0] = Waypoint(x_mm=bot_x, y_mm=bot_y)
                        print(f"[START] Anchor set to bot position: ({bot_x:.1f}, {bot_y:.1f})mm")
                    else:
                        print(f"[START] WARNING: No bot pose detected after 10s wait!")
                        print(f"[START] This may cause waypoint following issues - robot position unknown")
                        # Use first fitted waypoint but warn that this might be wrong
                        start_pos = _first_waypoint_pos(fitted)
                        if start_pos:
                            print(f"[START] Falling back to first fitted waypoint: {start_pos}")
                            print(f"[START] NOTE: Follower will try to skip waypoint 0 if robot is far away")
                        else:
                            error = "No waypoints available and no bot pose detected"
                    firmware_gcode = cvp.build_firmware_gcode(
                        gcode_str,
                        start_pos=start_pos,
                        fitted_wps=fitted,
                        norm_origin=norm_origin,
                    )
                    print("Built firmware G-code:")
                    print(firmware_gcode)
                    print(f"Built firmware G-code with {len(firmware_gcode.splitlines())} lines.")              
                    _send_with_cv_correction(firmware_gcode, fitted, start_pos=start_pos, window_size=3)
                except Exception as e:
                    error = f"Failed to send G-code: {e}"

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
    print("HOME: Requested homing sequence")
    _ensure_cv_thread()
    print("HOME: Ensuring board is calibrated...")
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
    
    ok = _home_bot()
    if ok:
        return {"success": True}, 200
    return {"success": False, "error": "homing failed"}, 500

@app.route("/test_send", methods=["POST"])
def test_send():
    """Send G-code directly without pathfinding pipeline (for testing)"""
    _ensure_cv_thread()
    try:
        gcode_file = request.form.get("gcode_file", os.path.join(UPLOADS_DIR, "test_simple_square.gcode"))
        
        if not os.path.isfile(gcode_file):
            return {"error": f"File not found: {gcode_file}"}, 404
        
        with open(gcode_file, 'r') as f:
            gcode_str = f.read()
        
        if not cvp.get_state()["calibrated"]:
            return {"error": "Board not calibrated - wait for calibration to lock"}, 400
        
        board_size = cvp.board_size_mm()
        if not board_size:
            return {"error": "No board scale available - robot markers not detected"}, 400
        
        bounds = cvp.get_board_bounds_mm()
        if bounds:
            min_x, min_y, max_x, max_y = bounds
            margin = max(CV_BOARD_MARGIN_MM, CV_BOARD_MARGIN_FRAC * min(max_x - min_x, max_y - min_y))
            print(f"TEST_SEND: Board {board_size[0]:.1f}mm × {board_size[1]:.1f}mm, margins={margin:.1f}mm")
            print(f"TEST_SEND: Safe zone ({min_x+margin:.1f},{min_y+margin:.1f}) to ({max_x-margin:.1f},{max_y-margin:.1f})")
        
        normalized = gcode_str
        
        print(f"Sending test G-code from {gcode_file}:")
        for i, line in enumerate(normalized.split('\n')[:10]):
            if line.strip():
                print(f"  {i}: {line}")
        
        bt.send_gcode_windowed(normalized, window_size=1)
        
        return {"success": True, "lines_sent": len([l for l in normalized.split('\n') if l.strip()])}
    except Exception as e:
        return {"error": str(e)}, 500

def _shutdown():
    try:
        RUN.stop()
        bt.close()
    except Exception:
        pass
    cam = get_cam()
    if cam:
        cam.close()
    os._exit(0)

if __name__ == "__main__":
    try: faulthandler.enable()
    except Exception: pass
    
    print("=" * 80)
    print("Starting doodle_bot server")
    print("=" * 80)
    print(f"CV_SOFTWARE_DIR: {CV_SOFTWARE_DIR}")
    print(f"UPLOADS_DIR: {UPLOADS_DIR}")
    print(f"PATHFINDING_DIR: {PATHFINDING_DIR}")
    print(f"CAMERA_SRC: {CAMERA_SRC}")
    print(f"USE_CV_CORRECTION: {USE_CV_CORRECTION}")
    print("=" * 80)
    
    _ensure_cv_thread()
    app.run(host="0.0.0.0", port=int(os.getenv("PORT", "5000")), threaded=True, debug=False)
