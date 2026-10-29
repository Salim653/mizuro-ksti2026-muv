"""Tes synth: frame yang dirender harus konsisten dengan ground truth.

Ini yang membuat eval tanpa hardware sah: kalau render dan proyeksi tidak
konsisten, angka akurasi di eval berarti apa-apa. Jadi paint diperiksa
LANDING di posisi terproyeksi, bukan cuma "ada marka di frame".
"""
import cv2
import numpy as np
import pytest

from mizuro.perception.calibration import CameraCalibration
from mizuro.perception.synth import (
    DASH_ON_M,
    DASH_OFF_M,
    DASH_WIDTH_M,
    EDGE_WIDTH_M,
    FLOOR_DARK,
    FLOOR_LIGHT,
    PAINT_DARK,
    PAINT_LIGHT,
    LaneTruth,
    random_truth,
    render_lane_frame,
)


@pytest.fixture
def calib():
    return CameraCalibration.from_hfov(640, 480, 0.16, 12.0)


def test_truth_state_matches_analytic_formula():
    truth = LaneTruth(a=0.03, b=-0.04, c=0.07)
    lateral, heading, curvature = truth.state_at(1.0)
    slope = 2 * 0.03 * 1.0 - 0.04
    assert lateral == pytest.approx(0.03 - 0.04 + 0.07)
    assert heading == pytest.approx(np.arctan(slope))
    assert curvature == pytest.approx(2 * 0.03 / (1 + slope * slope) ** 1.5)


def test_as_dict_reports_station_state():
    truth = LaneTruth(0.02, 0.01, 0.05)
    d = truth.as_dict(station_m=0.6)
    assert d["station_m"] == 0.6
    assert d["lateral_m"] == pytest.approx(truth.x_at(0.6))
    assert d["a"] == 0.02 and d["width_m"] == 0.5


def test_paint_lands_at_projected_edge_positions(calib):
    # Frame tanpa noise/blur: piksel di posisi tepi terproyeksi HARUS
    # berwarna cat, dan lantai di antara dua tepi harus warna lantai.
    truth = LaneTruth(0.0195, 0.0587, 0.1189, polarity="dark_on_light")
    frame, _ = render_lane_frame(calib, truth, seed=1, noise_sigma=0.0,
                                 blur_sigma=0.0)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    checked = 0
    for y in (1.0, 1.4, 1.8):     # y=0.6: tepi kanan jatuh di luar frame
        for sign in (-1.0, 1.0):
            u, v = calib.ground_to_pixel(
                y, truth.x_at(y) + sign * truth.width_m / 2.0)
            u, v = int(round(float(u))), int(round(float(v)))
            if not (0 <= u < 640 and 0 <= v < 480):
                continue
            assert gray[v, u] == pytest.approx(PAINT_DARK, abs=30)
            checked += 1
        # Lantai di antara dua tepi.
        u, v = calib.ground_to_pixel(y, truth.x_at(y) + 0.12)
        u, v = int(round(float(u))), int(round(float(v)))
        if 0 <= u < 640 and 0 <= v < 480:
            assert gray[v, u] == pytest.approx(FLOOR_LIGHT, abs=30)
            checked += 1
    assert checked >= 6


def test_dash_alternates_on_off_along_forward(calib):
    truth = LaneTruth(0.0, 0.0, 0.0, polarity="light_on_dark")
    frame, _ = render_lane_frame(calib, truth, seed=1, noise_sigma=0.0,
                                 blur_sigma=0.0)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Sapu sepanjang pusat lajur dari dekat ke jauh, catat paint/floor.
    states = []
    for y in np.arange(0.45, 2.0, 0.01):
        u, v = calib.ground_to_pixel(y, truth.x_at(y))
        u, v = int(round(float(u))), int(round(float(v)))
        if 0 <= u < 640 and 0 <= v < 480:
            states.append(gray[v, u] > 150)
    states = np.array(states)
    # Harus ada perpindahan paint -> floor -> paint (dash nyata).
    transitions = int(np.sum(states[1:] != states[:-1]))
    assert transitions >= 3
    # Proporsi paint kira-kira DASH_ON / (DASH_ON + DASH_OFF) = 0.5.
    assert 0.3 < states.mean() < 0.7


def test_both_polarities_render_contrasting_scenes(calib):
    truth_l = LaneTruth(0, 0, 0, polarity="light_on_dark")
    truth_d = LaneTruth(0, 0, 0, polarity="dark_on_light")
    fl, _ = render_lane_frame(calib, truth_l, seed=1, noise_sigma=0.0,
                              blur_sigma=0.0)
    fd, _ = render_lane_frame(calib, truth_d, seed=1, noise_sigma=0.0,
                              blur_sigma=0.0)
    gl = cv2.cvtColor(fl, cv2.COLOR_BGR2GRAY)
    gd = cv2.cvtColor(fd, cv2.COLOR_BGR2GRAY)
    # light_on_dark: lantai gelap, marka terang.
    assert gl[300, 10] == pytest.approx(FLOOR_DARK, abs=5)
    u, v = calib.ground_to_pixel(1.0, 0.25)
    assert gl[int(v), int(u)] == pytest.approx(PAINT_LIGHT, abs=30)
    # dark_on_light: sebaliknya.
    assert gd[300, 10] == pytest.approx(FLOOR_LIGHT, abs=5)
    u, v = calib.ground_to_pixel(1.0, 0.25)
    assert gd[int(v), int(u)] == pytest.approx(PAINT_DARK, abs=30)


def test_noise_is_seeded_and_reproducible(calib):
    truth = LaneTruth(0, 0, 0)
    f1, _ = render_lane_frame(calib, truth, seed=7)
    f2, _ = render_lane_frame(calib, truth, seed=7)
    f3, _ = render_lane_frame(calib, truth, seed=8)
    assert np.array_equal(f1, f2)
    assert not np.array_equal(f1, f3)


def test_random_truth_stays_in_sane_range():
    rng = np.random.default_rng(5)
    for _ in range(200):
        t = random_truth(rng)
        assert -0.12 <= t.c <= 0.12
        assert -0.06 <= t.b <= 0.06
        assert abs(t.a) <= 0.25
        assert t.polarity in ("light_on_dark", "dark_on_light")
        assert t.width_m == 0.5


def test_render_returns_ground_truth_dict(calib):
    truth = LaneTruth(0.02, 0.01, 0.05)
    frame, gt = render_lane_frame(calib, truth, seed=1)
    assert frame.shape == (480, 640, 3)
    assert frame.dtype == np.uint8
    assert gt["lateral_m"] == pytest.approx(truth.x_at(0.6))
    assert gt["heading_rad"] == pytest.approx(
        np.arctan(2 * 0.02 * 0.6 + 0.01))


def test_dash_constants_match_guidebook_minimums():
    # Guidebook: marka >= 1 cm lebar; pola dash bebas.
    assert EDGE_WIDTH_M >= 0.01
    assert DASH_WIDTH_M >= 0.01
    assert DASH_ON_M > 0 and DASH_OFF_M > 0
