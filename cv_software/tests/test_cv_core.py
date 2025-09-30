# tests/test_cv_core.py
from __future__ import annotations

import math
import types
from typing import Dict, Tuple, Optional

import numpy as np
import pytest

import app.cv_core as cvp
import cv2

# ------------------------------
# Utilities & synthetic geometry
# ------------------------------

def square_corners(cx: float, cy: float, size: float = 20.0) -> np.ndarray:
    """Return (4,2) corners TL,TR,BR,BL around a center (cx,cy)."""
    half = size / 2.0
    return np.array(
        [
            [cx - half, cy - half],  # TL
            [cx + half, cy - half],  # TR
            [cx + half, cy + half],  # BR
            [cx - half, cy + half],  # BL
        ],
        dtype=np.float32,
    )


def fake_board_idmap() -> Dict[int, np.ndarray]:
    """
    Build a consistent board with IDs:
      TL=0 at (100,100), TR=1 at (300,100), BR=2 at (300,220), BL=3 at (100,220).
    """
    return {
        cvp.BOARD_CORNERS["TL"]: square_corners(100, 100, size=20),
        cvp.BOARD_CORNERS["TR"]: square_corners(300, 100, size=20),
        cvp.BOARD_CORNERS["BR"]: square_corners(300, 220, size=20),
        cvp.BOARD_CORNERS["BL"]: square_corners(100, 220, size=20),
    }


def fake_bot_idmap(bot0_xy=(150, 160), bot1_xy=(200, 160)) -> Dict[int, np.ndarray]:
    """Two bot markers (IDs 10 and 11) left->right along +x axis."""
    idmap = {}
    idmap[cvp.BOT_MARKERS[0]] = square_corners(*bot0_xy, size=10)
    idmap[cvp.BOT_MARKERS[1]] = square_corners(*bot1_xy, size=10)
    return idmap


# ------------------------------
# Fixtures
# ------------------------------

@pytest.fixture
def blank_frame() -> np.ndarray:
    # Simple 480x640 black BGR frame
    return np.zeros((480, 640, 3), dtype=np.uint8)


@pytest.fixture
def calibrator(monkeypatch) -> cvp.BoardCalibrator:
    cal = cvp.BoardCalibrator(cvp.CameraModel())
    # Monkeypatch its detector to return our synthetic board
    class _Det:
        def detect(self, gray):
            id_map = fake_board_idmap()
            # corners & ids are ignored downstream; id_map is what matters.
            return None, None, id_map
    cal.det = _Det()
    return cal


@pytest.fixture
def tracker_with_calibrated_board(calibrator, blank_frame) -> Tuple[cvp.BotTracker, cvp.BoardCalibrator]:
    # Run calibration once so H is set
    ok = calibrator.calibrate(blank_frame)
    assert ok is not None, "Board calibration failed in fixture"
    trk = cvp.BotTracker(calibrator)

    # Patch tracker detector to emit both board and bot markers (board markers are ignored by track)
    class _Det:
        def detect(self, gray):
            id_map = fake_bot_idmap()
            return None, None, id_map
    trk.det = _Det()
    return trk, calibrator


@pytest.fixture
def pipeline(monkeypatch, blank_frame) -> cvp.CVPipeline:
    pipe = cvp.CVPipeline()

    # Patch internal detectors
    class _BoardDet:
        def detect(self, gray):
            return None, None, fake_board_idmap()

    class _BotDet:
        def detect(self, gray):
            return None, None, fake_bot_idmap()

    pipe.cal.det = _BoardDet()
    pipe.trk.det = _BotDet()

    # Calibrate immediately for tests that only need tracking
    assert pipe.calibrate_board(blank_frame) is True
    return pipe


# ------------------------------
# BoardCalibrator unit tests
# ------------------------------

def test_collect_corners(calibrator):
    id_map = fake_board_idmap()
    have = calibrator._collect_corners(id_map)
    assert set(have.keys()) == {"TL", "TR", "BR", "BL"}
    for v in have.values():
        assert v.shape == (2,)


def test_complete_quad_synthesizes_missing(calibrator):
    id_map = fake_board_idmap()
    have = calibrator._collect_corners(id_map)
    # Remove BR to test synthesis
    have.pop("BR")
    out = calibrator._complete_quad(have)
    assert "BR" in out
    # Geometry: BR ≈ TR + (BL - TL)
    np.testing.assert_allclose(
        out["BR"],
        have["TR"] + (have["BL"] - have["TL"]),
        rtol=1e-6, atol=1e-6
    )


def test_calibrate_success(calibrator, blank_frame):
    bp = calibrator.calibrate(blank_frame)
    assert isinstance(bp, cvp.BoardPose)
    assert bp.board_size_px[0] > 0 and bp.board_size_px[1] > 0
    assert 0.0 <= bp.confidence <= 1.0
    # Reprojection error should be finite and small-ish
    assert math.isfinite(bp.reproj_err_px)
    assert bp.H_img2board.shape == (3, 3)


def test_calibrate_tolerates_missing_corners(monkeypatch, blank_frame):
    cal = cvp.BoardCalibrator()

    class _Det:
        def detect(self, gray):
            idmap = fake_board_idmap()
            # Remove one more so only two corners remain → not enough to calibrate
            idmap.pop(cvp.BOARD_CORNERS["BR"])
            idmap.pop(cvp.BOARD_CORNERS["BL"])
            return None, None, idmap

    cal.det = _Det()
    # No prior board_pose; with <3 corners we keep previous (None)
    assert cal.calibrate(blank_frame) is None


# ------------------------------
# BotTracker unit tests
# ------------------------------

def test_track_success(tracker_with_calibrated_board, blank_frame):
    trk, cal = tracker_with_calibrated_board
    bp = trk.track(blank_frame)
    assert isinstance(bp, cvp.BotPose)
    # Bot markers were horizontal (y equal), so heading ~ 0
    assert abs(bp.heading_rad) < 1e-3
    assert bp.pixel_baseline > 0
    assert 0.0 <= bp.confidence <= 1.0


def test_track_missing_marker_returns_none(calibrator, blank_frame, monkeypatch):
    # Calibrate first
    assert calibrator.calibrate(blank_frame) is not None

    trk = cvp.BotTracker(calibrator)

    class _Det:
        def detect(self, gray):
            id_map = fake_bot_idmap()
            # Remove one marker to simulate partial detection
            id_map.pop(cvp.BOT_MARKERS[1])
            return None, None, id_map

    trk.det = _Det()
    assert trk.track(blank_frame) is None


# ------------------------------
# ScaleEstimator unit tests
# ------------------------------

def test_scale_from_bot_baseline_updates_cal(calibrator, blank_frame):
    assert calibrator.calibrate(blank_frame) is not None
    sc = cvp.ScaleEstimator(calibrator)
    # Create a fake bot pose with baseline 40 px ⇒ mm/px = 60/40 = 1.5
    bot = cvp.BotPose(center_board_px=(0, 0), heading_rad=0.0, pixel_baseline=40.0)
    mm_per_px = sc.from_bot_baseline(bot)
    assert mm_per_px == pytest.approx(cvp.BOT_BASELINE_MM / 40.0, rel=1e-6)
    assert calibrator.board_pose.mm_per_px == pytest.approx(mm_per_px, rel=1e-6)


def test_scale_from_board_size_uses_known_constants(monkeypatch, calibrator, blank_frame):
    assert calibrator.calibrate(blank_frame) is not None

    # Temporarily set known board sizes
    monkeypatch.setattr(cvp, "KNOWN_BOARD_WIDTH_MM", 200.0, raising=False)
    monkeypatch.setattr(cvp, "KNOWN_BOARD_HEIGHT_MM", 100.0, raising=False)

    sc = cvp.ScaleEstimator(calibrator)
    mm_per_px = sc.from_board_size()
    assert mm_per_px is not None
    # Average of sx & sy
    Wpx, Hpx = calibrator.board_pose.board_size_px
    expected = 0.5 * (200.0 / Wpx + 100.0 / Hpx)
    assert mm_per_px == pytest.approx(expected, rel=1e-6)


# ------------------------------
# CVPipeline (facade) unit tests
# ------------------------------

def test_pipeline_full_flow(pipeline, blank_frame):
    # Already calibrated in fixture
    assert pipeline.get_state()["calibrated"] is True

    # First update sets bot pose and mm/px (via bot baseline fallback)
    ok = pipeline.update_bot(blank_frame)
    assert ok is True

    st = pipeline.get_state()
    assert st["bot_pose"] is not None
    assert st["mm_per_px"] is not None
    assert 0.0 <= st["board_confidence"] <= 1.0

    # Check smoothing moves heading/center gradually (hard to assert numerically;
    # we just ensure a second call does not error and state remains consistent).
    ok2 = pipeline.update_bot(blank_frame)
    assert ok2 is True
    st2 = pipeline.get_state()
    assert st2["bot_pose"] is not None
    assert st2["mm_per_px"] is not None


def test_corrected_delta_rotation():
    # Move 100 mm forward (board +x) with bot heading 90° (pi/2):
    # Bot's "forward" aligns with board +y, so fwd ~ +0, left ~ +100.
    fwd, lef = cvp.correct_delta_mm((100.0, 0.0), math.pi / 2)
    assert abs(fwd) < 1e-6
    # left is positive but y-down flip makes left negative internally then flipped; net sign tested by magnitude
    assert abs(lef) == pytest.approx(100.0, rel=1e-6)


def test_warp_points_board2img_roundtrip(calibrator, blank_frame):
    bp = calibrator.calibrate(blank_frame)
    assert bp is not None
    # Take a point in board coords and warp back to image, then forward again
    pts_board = np.array([[10.0, 5.0]], dtype=np.float32)
    pts_img = cvp.warp_points_board2img(bp.H_img2board, pts_board)
    # Forward should get us (approximately) the same board point
    pts_board2 = cv2.perspectiveTransform(pts_img[None], bp.H_img2board)[0]
    np.testing.assert_allclose(pts_board2, pts_board, atol=1e-3, rtol=1e-4)


# ------------------------------
# Edge cases & robustness
# ------------------------------

def test_calibrate_handles_none_H(monkeypatch, blank_frame):
    cal = cvp.BoardCalibrator()

    class _Det:
        def detect(self, gray):
            return None, None, fake_board_idmap()

    cal.det = _Det()

    # Monkeypatch getPerspectiveTransform to return None
    import cv2
    orig = cv2.getPerspectiveTransform

    def _none_pt(src, dst):
        return None

    monkeypatch.setattr(cv2, "getPerspectiveTransform", _none_pt)
    try:
        # No previous pose; should remain None
        assert cal.calibrate(blank_frame) is None
    finally:
        # restore
        monkeypatch.setattr(cv2, "getPerspectiveTransform", orig)


def test_bot_tracker_no_board_pose_returns_none(blank_frame):
    cal = cvp.BoardCalibrator()
    trk = cvp.BotTracker(cal)

    class _Det:
        def detect(self, gray):
            return None, None, fake_bot_idmap()

    trk.det = _Det()
    assert trk.track(blank_frame) is None
# ---------- extra tests for cv_core ----------

def test_undistort_noop_and_with_intrinsics(blank_frame):
    cal = cvp.BoardCalibrator(cvp.CameraModel())
    # No intrinsics => returns same object (no-op)
    out0 = cal._undistort(blank_frame)
    assert out0 is blank_frame

    # With valid intrinsics: should return an array with same shape (content may equal for zero dist)
    h, w = blank_frame.shape[:2]
    K = np.array([[500.0, 0, w/2],
                  [0, 500.0, h/2],
                  [0,   0,     1]], dtype=np.float32)
    dist = np.zeros(5, dtype=np.float32)
    cal2 = cvp.BoardCalibrator(cvp.CameraModel(K=K, dist=dist))
    out1 = cal2._undistort(blank_frame)
    assert isinstance(out1, np.ndarray)
    assert out1.shape == blank_frame.shape


def test_calibrate_keeps_previous_when_corners_drop(monkeypatch, blank_frame):
    cal = cvp.BoardCalibrator()

    class _GoodDet:
        def detect(self, gray):
            return None, None, fake_board_idmap()
    cal.det = _GoodDet()

    prev = cal.calibrate(blank_frame)
    assert prev is not None

    # Now only 2 corners -> should keep the previous pose object
    class _TwoCorners:
        def detect(self, gray):
            idmap = fake_board_idmap()
            # drop TR and BR
            idmap.pop(cvp.BOARD_CORNERS["TR"])
            idmap.pop(cvp.BOARD_CORNERS["BR"])
            return None, None, idmap
    cal.det = _TwoCorners()
    after = cal.calibrate(blank_frame)
    assert after is prev  # same object retained


def test_calibrate_rejects_by_condition(monkeypatch, blank_frame):
    # Force detection of a valid board, but make cond(H) huge AND reprojection not very good
    cal = cvp.BoardCalibrator()

    class _Det:
        def detect(self, gray):
            return None, None, fake_board_idmap()
    cal.det = _Det()

    # Make reprojection "not good" (> _REPROJ_GOOD_PX) so condition matters
    monkeypatch.setattr(cvp.BoardCalibrator, "_reproj_error", lambda self, H, a, b: 3.0)
    # Force gigantic condition number
    monkeypatch.setattr(np.linalg, "cond", lambda H: cvp._COND_HARD_LIMIT * 10.0)

    # No previous pose → should remain None (rejected)
    assert cal.calibrate(blank_frame) is None


def test_calibrate_accepts_when_reproj_good_even_if_cond_bad(monkeypatch, blank_frame):
    cal = cvp.BoardCalibrator()

    class _Det:
        def detect(self, gray):
            return None, None, fake_board_idmap()
    cal.det = _Det()

    # Very small reprojection => should accept even if cond is huge
    monkeypatch.setattr(cvp.BoardCalibrator, "_reproj_error", lambda self, H, a, b: 0.25)
    monkeypatch.setattr(np.linalg, "cond", lambda H: cvp._COND_HARD_LIMIT * 10.0)

    bp = cal.calibrate(blank_frame)
    assert isinstance(bp, cvp.BoardPose)


def test_scale_from_bot_baseline_zero_rejected(calibrator, blank_frame):
    assert calibrator.calibrate(blank_frame) is not None
    sc = cvp.ScaleEstimator(calibrator)
    bot = cvp.BotPose(center_board_px=(0, 0), heading_rad=0.0, pixel_baseline=0.0)
    assert sc.from_bot_baseline(bot) is None
    # Should not set mm_per_px
    assert calibrator.board_pose.mm_per_px is None


def test_board_pixels_to_mm_simple():
    pts = np.array([[2.0, 3.0], [10.0, -4.0]], dtype=np.float32)
    out = cvp.board_pixels_to_mm(pts, 1.5)
    np.testing.assert_allclose(out, pts * 1.5)


@pytest.mark.parametrize(
    "dx,dy,hd,exp_fwd_sign,exp_left_mag",
    [
        (100.0, 0.0, 0.0, +1, 0.0),                    # heading 0 => forward is +x
        (100.0, 0.0, math.pi/2, 0, 100.0),             # heading 90deg => left-only
        (0.0, 50.0, -math.pi/2, -1, 0.0),              # move +y with -90deg -> forward negative, left ~ 0
        (-80.0, 0.0, math.pi, +1, 0.0),                # heading pi, dx<0 => forward positive (toward bot forward)
    ],
)
def test_correct_delta_mm_various(dx, dy, hd, exp_fwd_sign, exp_left_mag):
    f, l = cvp.correct_delta_mm((dx, dy), hd)
    if exp_fwd_sign == 0:
        assert abs(f) < 1e-6
    else:
        assert math.copysign(1.0, f) == exp_fwd_sign
    assert pytest.approx(abs(l), rel=1e-6) == exp_left_mag


def test_bot_tracker_confidence_caps_at_one(tracker_with_calibrated_board, blank_frame):
    trk, cal = tracker_with_calibrated_board

    # Make the markers far apart in pixels -> very large baseline -> confidence should clamp at 1
    class _Det:
        def detect(self, gray):
            id_map = fake_bot_idmap(bot0_xy=(100, 160), bot1_xy=(1000, 160))
            return None, None, id_map
    trk.det = _Det()

    bp = trk.track(blank_frame)
    assert isinstance(bp, cvp.BotPose)
    assert bp.confidence <= 1.0
    assert pytest.approx(bp.confidence, rel=1e-6) == 1.0


def test_bot_tracker_heading_wrap(tracker_with_calibrated_board, blank_frame):
    trk, cal = tracker_with_calibrated_board

    # Arrange markers vertically (same x, different y) -> heading around +pi/2
    class _Det:
        def detect(self, gray):
            id_map = {
                cvp.BOT_MARKERS[0]: square_corners(200, 100, size=10),
                cvp.BOT_MARKERS[1]: square_corners(200, 220, size=10),
            }
            return None, None, id_map
    trk.det = _Det()

    bp = trk.track(blank_frame)
    assert isinstance(bp, cvp.BotPose)
    assert -math.pi <= bp.heading_rad <= math.pi
    assert abs(abs(bp.heading_rad) - math.pi/2) < 0.05


def test_pipeline_update_bot_false_when_no_markers(pipeline, blank_frame, monkeypatch):
    # Patch tracker detector to output empty map
    class _EmptyDet:
        def detect(self, gray):
            return None, None, {}
    pipeline.trk.det = _EmptyDet()

    assert pipeline.update_bot(blank_frame) is False
    st = pipeline.get_state()
    # Previously calibrated; mm_per_px remains whatever it was (None or prior),
    # but bot_pose should still be None since update failed
    assert st["bot_pose"] is None


def test_pipeline_corrected_delta_for_bt_requires_cal_and_heading(pipeline):
    pipeline._pose_smooth = None
    pipeline._bot = None

    bp = pipeline.cal.board_pose
    assert bp is not None
    pipeline.cal.board_pose.mm_per_px = 1.0

    passthrough = pipeline.corrected_delta_for_bt((5.0, -3.0), rotate_into_bot_frame=False)
    assert passthrough == (5.0, -3.0)

    rotated = pipeline.corrected_delta_for_bt((5.0, -3.0), rotate_into_bot_frame=True)
    # With no pose available, function returns the input delta unchanged (pass-through)
    assert rotated == (5.0, -3.0)

def test_reproj_error_decreases_for_exact_transform():
    # Build a perfect square & identity warp -> reprojection error ~ 0
    H = np.eye(3, dtype=np.float32)
    img_pts = np.float32([[0,0],[10,0],[10,10],[0,10]])
    dst_pts = img_pts.copy()
    err = cvp.BoardCalibrator._reproj_error(H, img_pts, dst_pts)
    assert err == pytest.approx(0.0, abs=1e-6)