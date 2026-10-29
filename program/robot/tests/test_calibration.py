"""Tes kalibrasi: proyeksi piksel <-> tanah, round-trip, mount nyata.

Konvensi (mengikuti annotations.jsonl arena-3d): car frame X maju, Y kanan,
Z bawah; pinhole CV. ground_to_pixel(x_forward, y_right) — urutan argumen
ini pernah jadi sumber bug, jadi round-trip diuji eksplisit.
"""
import math

import numpy as np
import pytest

from mizuro.perception.calibration import (
    DEFAULT_HFOV_DEG,
    DEFAULT_HEIGHT_M,
    DEFAULT_PITCH_DEG,
    MAX_FORWARD_M,
    CameraCalibration,
)


@pytest.fixture
def calib():
    return CameraCalibration.from_hfov(640, 480, 0.16, 12.0)


def test_default_mount_matches_arena3d(calib):
    assert calib.height_m == DEFAULT_HEIGHT_M == 0.16
    assert math.degrees(calib.pitch_rad) == pytest.approx(DEFAULT_PITCH_DEG)
    # fx dari hFOV 62 derajat pada 640 px.
    assert calib.fx == pytest.approx(320.0 / math.tan(math.radians(31.0)))
    assert calib.cx == pytest.approx(319.5)
    assert calib.cy == pytest.approx(239.5)


def test_ground_enters_frame_close_to_mount(calib):
    # README arena-3d: dari mount 0.16 m / 12 deg, tanah masuk ~0.22 m.
    y_near, _ = calib.forward_range(0.35)
    assert y_near == pytest.approx(0.2185, abs=0.01)


def test_forward_range_clamped_and_roi_scoped(calib):
    # roi_top 0.35 -> batas jauh ~2.1 m (bukan 89 m dekat horizon).
    _, y_far = calib.forward_range(0.35)
    assert 2.0 < y_far < 2.2
    # Tanpa roi (default 0.35 juga dipakai sekarang), tetap ter-clamp.
    _, y_far0 = calib.forward_range(0.0)
    assert y_far0 <= MAX_FORWARD_M


def test_roundtrip_pixel_ground_pixel(calib):
    rng = np.random.default_rng(11)
    checked = 0
    for _ in range(50):
        x = float(rng.uniform(0.3, 2.0))       # forward
        y = float(rng.uniform(-0.35, 0.35))    # right
        u, v = calib.ground_to_pixel(x, y)
        # Titik sampel bisa jatuh di luar frame (lajur lebar di jarak
        # dekat) — itu wajar; yang diuji adalah round-trip-nya eksak.
        if 0 <= float(u) <= 640 and 0 <= float(v) <= 480:
            checked += 1
        x2, y2 = calib.pixel_to_ground(u, v)
        assert float(x2) == pytest.approx(x, abs=1e-6)
        assert float(y2) == pytest.approx(y, abs=1e-6)
    assert checked > 10   # sebagian besar titik harus masuk frame


def test_roundtrip_ground_pixel_ground(calib):
    rng = np.random.default_rng(12)
    for _ in range(50):
        u = float(rng.uniform(50, 590))
        v = float(rng.uniform(calib.horizon_row + 5, 470))
        x, y = calib.pixel_to_ground(u, v)
        assert math.isfinite(float(x)) and math.isfinite(float(y))
        u2, v2 = calib.ground_to_pixel(x, y)
        assert float(u2) == pytest.approx(u, abs=1e-6)
        assert float(v2) == pytest.approx(v, abs=1e-6)


def test_above_horizon_is_nan_not_garbage(calib):
    # Baris di atas horizon tidak menyentuh tanah -> NaN, bukan angka palsu.
    x, y = calib.pixel_to_ground(320.0, calib.horizon_row - 10.0)
    assert math.isnan(float(x)) and math.isnan(float(y))


def test_homography_matches_ray_projection(calib):
    rng = np.random.default_rng(13)
    uv = np.column_stack([rng.uniform(50, 590, 30),
                          rng.uniform(calib.horizon_row + 5, 470, 30)])
    batch = calib.ground_from_pixels(uv)
    for (u, v), (x, y) in zip(uv, batch):
        xr, yr = calib.pixel_to_ground(u, v)
        assert float(x) == pytest.approx(float(xr), abs=1e-9)
        assert float(y) == pytest.approx(float(yr), abs=1e-9)


def test_homography_horizon_rows_are_nan(calib):
    uv = np.array([[320.0, calib.horizon_row - 5.0],
                   [100.0, calib.horizon_row - 1.0]])
    out = calib.ground_from_pixels(uv)
    assert np.isnan(out).all()


def test_center_column_maps_to_straight_ahead(calib):
    # u = cx harus selalu y_right = 0 (tidak ada offset lateral).
    for v in (200, 250, 300, 400):
        _, y = calib.pixel_to_ground(calib.cx, v)
        assert float(y) == pytest.approx(0.0, abs=1e-9)


def test_right_is_positive_right(calib):
    # y_right positif harus muncul di u > cx.
    u, _ = calib.ground_to_pixel(1.0, 0.2)
    assert float(u) > calib.cx
    u, _ = calib.ground_to_pixel(1.0, -0.2)
    assert float(u) < calib.cx


def test_farther_is_higher_in_frame(calib):
    # Jarak maju bertambah -> v mengecil (mendekati horizon).
    v_prev = None
    for x in (0.4, 0.8, 1.2, 1.6, 2.0):
        _, v = calib.ground_to_pixel(x, 0.0)
        if v_prev is not None:
            assert float(v) < v_prev
        v_prev = float(v)


def test_rejects_non_positive_height():
    with pytest.raises(ValueError):
        CameraCalibration(width=640, height=480, height_m=0.0)


def test_explicit_intrinsics_override_hfov():
    calib = CameraCalibration(width=640, height=480, height_m=0.2,
                              fx=500.0, fy=500.0, cx=320.0, cy=240.0)
    assert calib.fx == 500.0 and calib.cy == 240.0
