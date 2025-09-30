from __future__ import annotations
from dataclasses import dataclass
from typing import Optional, Iterable, List, Dict, Any, Union, Tuple
import math, time, threading, queue

from app.cv_core import CVPipeline

# ---------- instructions ----------
@dataclass(frozen=True)
class MoveMM:
    dx_mm: float
    dy_mm: float
    speed_mm_s: Optional[float] = None
    rotate_into_bot_frame: bool = True  # kept for compatibility; follower rotates anyway

@dataclass(frozen=True)
class PenCmd:
    down: bool

Instruction = Union[MoveMM, PenCmd]

class InstructionQueue:
    def __init__(self, maxsize: int = 10000):
        self._q: "queue.Queue[Instruction]" = queue.Queue(maxsize=maxsize)
    def put_many(self, items: Iterable[Instruction]) -> None:
        for it in items:
            self._q.put_nowait(it)
    def get_blocking(self, timeout: Optional[float] = None) -> Instruction:
        return self._q.get(timeout=timeout)
    def empty(self) -> bool:
        return self._q.empty()

# ---------- payload parsing ----------
def parse_moves_payload(payload: Dict[str, Any]) -> List[Instruction]:
    """
    Accepts any of:
      { "frame":"board", "moves":[{"dx":..,"dy":..,"speed":..,"rotate_into_bot_frame":..}, ...] }
      { "frame":"board", "waypoints":[[x,y], [x,y], ...] }
      [[dx,dy], ...]   # tolerant alternative used by sim client sometimes
      [{"dx":..,"dy":..}, ...]
    Returns: list[MoveMM]
    """
    # if the payload is already a list of moves
    if isinstance(payload, list):
        out: List[Instruction] = []
        for item in payload:
            if isinstance(item, dict):
                out.append(MoveMM(float(item["dx"]), float(item["dy"]),
                                  float(item["speed"]) if "speed" in item else None,
                                  bool(item.get("rotate_into_bot_frame", True))))
            else:
                dx, dy = item
                out.append(MoveMM(float(dx), float(dy)))
        return out

    frame = str(payload.get("frame", "board")).lower()
    if frame != "board":
        raise ValueError("Unsupported frame; expected 'board'")

    out: List[Instruction] = []
    if "moves" in payload:
        for m in payload["moves"]:
            out.append(MoveMM(
                dx_mm=float(m["dx"]),
                dy_mm=float(m["dy"]),
                speed_mm_s=float(m["speed"]) if "speed" in m else None,
                rotate_into_bot_frame=bool(m.get("rotate_into_bot_frame", True)),
            ))
        return out

    if "waypoints" in payload:
        wps: List[List[float]] = payload["waypoints"]
        for (x0, y0), (x1, y1) in zip(wps[:-1], wps[1:]):
            out.append(MoveMM(dx_mm=float(x1 - x0), dy_mm=float(y1 - y0)))
        return out

    raise ValueError("Payload must include 'moves' or 'waypoints'")

# ---------- utility: chunking (still useful for safety) ----------
def chunk_move(m: MoveMM, max_step_mm: float = 60.0) -> List[MoveMM]:
    """
    Even in closed loop we may want to cap very long legs to keep
    course-corrections frequent and safe.
    """
    d = math.hypot(m.dx_mm, m.dy_mm)
    if d <= max_step_mm or d == 0:
        return [m]
    n = max(1, math.ceil(d / max_step_mm))
    step_dx, step_dy = m.dx_mm / n, m.dy_mm / n
    return [MoveMM(step_dx, step_dy, m.speed_mm_s, m.rotate_into_bot_frame) for _ in range(n)]

# ---------- BT adapter (so follower can call one interface) ----------
class _BTAdapter:
    """
    Normalizes your BTLink surface. We prefer specific methods if present:
      - send_move(fwd_mm, left_mm)
      - send_rotate(rot_rad)  (optional)
      - send_pen(down: bool)
      - stop()
    Falls back to a generic send(dict) if needed.
    """
    def __init__(self, bt_link):
        self.bt = bt_link

    def stop(self):
        if hasattr(self.bt, "stop"):
            return self.bt.stop()
        if hasattr(self.bt, "send"):
            return self.bt.send({"cmd": "stop"})

    def move(self, fwd_mm: float, left_mm: float):
        if hasattr(self.bt, "send_move"):
            return self.bt.send_move(fwd_mm, left_mm)
        if hasattr(self.bt, "send"):
            return self.bt.send({"cmd": "move", "fwd_mm": fwd_mm, "left_mm": left_mm})

    def rotate(self, rot_rad: float):
        if abs(rot_rad) < 1e-6:
            return
        if hasattr(self.bt, "send_rotate"):
            return self.bt.send_rotate(rot_rad)
        if hasattr(self.bt, "send"):
            return self.bt.send({"cmd": "rotate", "rot_rad": rot_rad})

    def pen(self, down: bool):
        if hasattr(self.bt, "send_pen"):
            return self.bt.send_pen(down)
        if hasattr(self.bt, "send"):
            return self.bt.send({"cmd": "pen", "down": bool(down)})

# ---------- CV follower (closed loop) ----------
class CVFollower:
    def __init__(self, cvp: CVPipeline, bt_link,
                 pos_tol_mm: float = 4.0,
                 ang_tol_rad: float = math.radians(3.0),
                 max_step_fwd_mm: float = 30.0,
                 max_step_left_mm: float = 25.0,
                 max_step_rot_rad: float = math.radians(10),
                 kp_pos: float = 0.9,
                 kp_ang: float = 1.2,
                 stale_timeout_s: float = 0.6,
                 settle_pause_s: float = 0.05):
        self.cvp = cvp
        self.bt = _BTAdapter(bt_link)
        self.pos_tol_mm = pos_tol_mm
        self.ang_tol_rad = ang_tol_rad
        self.max_step_fwd_mm = max_step_fwd_mm
        self.max_step_left_mm = max_step_left_mm
        self.max_step_rot_rad = max_step_rot_rad
        self.kp_pos = kp_pos
        self.kp_ang = kp_ang
        self.stale_timeout_s = stale_timeout_s
        self.settle_pause_s = settle_pause_s
        self._last_seen_ts = 0.0

    # --- pose helpers ---
    def _get_pose_mm(self) -> Optional[Tuple[Tuple[float,float], float]]:
        st = self.cvp.get_state()
        if not st["calibrated"] or st["bot_pose"] is None or st["mm_per_px"] is None:
            return None
        self._last_seen_ts = time.time()
        px = st["bot_pose"]["center_board_px"]
        mm_per_px = st["mm_per_px"]
        heading = st["bot_pose"]["heading_rad"]
        return ((px[0]*mm_per_px, px[1]*mm_per_px), float(heading))

    def _stale(self) -> bool:
        return (time.time() - self._last_seen_ts) > self.stale_timeout_s

    # --- main loop: drive to an absolute board-frame waypoint ---
    def drive_to_waypoint_mm(self, goal_xy_mm: Tuple[float,float]) -> bool:
        while True:
            pose = self._get_pose_mm()
            if pose is None:
                # No pose right now: stop and bail if the reading has gone stale
                self.bt.stop()
                time.sleep(0.05)
                if self._stale():   # <-- add this
                    return False
                continue

            (cur_x_mm, cur_y_mm), heading = pose
            ex = goal_xy_mm[0] - cur_x_mm
            ey = goal_xy_mm[1] - cur_y_mm
            dist = math.hypot(ex, ey)

            if dist <= self.pos_tol_mm:
                # optionally: align heading to path tangent (not implemented here)
                self.bt.stop()
                time.sleep(self.settle_pause_s)
                return True

            # Convert board error to bot frame (fwd, left)
            fwd_mm, left_mm = self.cvp.corrected_delta_for_bt((ex, ey), rotate_into_bot_frame=True)

            # Heading correction from lateral error (simple heuristic)
            ang_correction = math.atan2(left_mm, max(1e-6, abs(fwd_mm)))
            rot_cmd = self.kp_ang * ang_correction
            rot_cmd = float(max(-self.max_step_rot_rad, min(self.max_step_rot_rad, rot_cmd)))

            # Proportional step on position
            fwd_cmd = float(self.kp_pos * fwd_mm)
            left_cmd = float(self.kp_pos * left_mm)

            # Saturate
            fwd_cmd = max(-self.max_step_fwd_mm,  min(self.max_step_fwd_mm,  fwd_cmd))
            left_cmd = max(-self.max_step_left_mm, min(self.max_step_left_mm, left_cmd))

            # Issue small step(s)
            # If your MCU supports rotate separately, send it; otherwise lateral bias handles yaw.
            self.bt.rotate(rot_cmd)
            self.bt.move(fwd_cmd, left_cmd)

            # Let it move; then re-measure on the next loop
            time.sleep(0.08)

            if self._stale():
                self.bt.stop()
                return False

# ---------- coordinator ----------
class Coordinator:
    """
    Pulls instructions and executes with CV-closed loop:
      - MoveMM: turned into ABSOLUTE board-frame goals and followed by CVFollower
      - PenCmd: passed through to BT
    """
    def __init__(self, queue: InstructionQueue, cvp: CVPipeline, bt_link):
        self.q = queue
        self.cvp = cvp
        self.bt = bt_link
        self.bt_norm = _BTAdapter(bt_link)
        self._stop = threading.Event()
        self._thr: Optional[threading.Thread] = None

        # Closed-loop follower
        self.follower = CVFollower(cvp, bt_link,
                                   pos_tol_mm=3.5,
                                   max_step_fwd_mm=28.0,
                                   max_step_left_mm=22.0,
                                   kp_pos=0.9, kp_ang=1.1)

        # For legacy open-loop fallbacks
        self.open_loop_max_step_mm = 15.0
        self.pkt_sleep_s = 0.01

    def start(self):
        try:
            if hasattr(self.bt, "connect"):
                self.bt.connect()
        except Exception as e:
            print(f"[BT] connect failed: {e}")
        self._thr = threading.Thread(target=self._run, daemon=True)
        self._thr.start()

    def stop(self):
        self._stop.set()
        if self._thr:
            self._thr.join(timeout=1.0)
        try:
            if hasattr(self.bt, "close"):
                self.bt.close()
        except:
            pass

    def _ready(self) -> bool:
        st = self.cvp.get_state()
        return bool(st["calibrated"] and st["mm_per_px"] is not None)

    def _run(self):
        while not self._stop.is_set():
            if not self._ready():
                time.sleep(0.05)
                continue
            try:
                inst = self.q.get_blocking(timeout=0.1)
            except Exception:
                time.sleep(0.01)
                continue
            self._handle(inst)

    def _cur_board_xy_mm(self) -> Optional[Tuple[float,float]]:
        st = self.cvp.get_state()
        if not st["calibrated"] or st["bot_pose"] is None or st["mm_per_px"] is None:
            return None
        px = st["bot_pose"]["center_board_px"]
        mpp = st["mm_per_px"]
        return (px[0]*mpp, px[1]*mpp)

    def _handle(self, inst: Instruction):
        if isinstance(inst, PenCmd):
            self.bt_norm.pen(inst.down)
            return

        if isinstance(inst, MoveMM):
            # Break long legs into sub-goals for safety/flexibility
            for step in chunk_move(inst, max_step_mm=60.0):
                # Compute ABSOLUTE goal in board frame (mm) from current pose + relative delta.
                cur_xy = self._cur_board_xy_mm()
                if cur_xy is None:
                    # Vision not ready momentarily; small wait and retry
                    time.sleep(0.05)
                    cur_xy = self._cur_board_xy_mm()
                    if cur_xy is None:
                        return  # abort this step; loop will retry on next _run cycle
                goal_xy = (cur_xy[0] + step.dx_mm, cur_xy[1] + step.dy_mm)

                # Closed-loop follower does the correction using live CV feedback.
                ok = self.follower.drive_to_waypoint_mm(goal_xy)
                if not ok:
                    # Vision went stale; stop motors and bail out of this instruction
                    self.bt_norm.stop()
                    return

            return
