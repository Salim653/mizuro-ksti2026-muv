"""Tes LaneTracker: akurasi pada frame sintetis, penolakan frame buruk,
dan logika stasiun (unambiguous-first, interpretasi run)."""
import cv2
import numpy as np
import pytest

from mizuro.perception.calibration import CameraCalibration
from mizuro.perception.classical import LANE_HALF_WIDTH_M, LaneTracker
from mizuro.perception.synth import (
    FLOOR_DARK,
    FLOOR_LIGHT,
    PAINT_DARK,
    PAINT_LIGHT,
    LaneTruth,
    render_lane_frame,
)


@pytest.fixture
def calib():
    return CameraCalibration.from_hfov(640, 480, 0.16, 12.0)


@pytest.fixture
def tracker():
    return LaneTracker()


def _binary_for(tracker, frame, calib, polarity):
    """Binary mask persis seperti yang dipakai estimate()."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    roi_row = int(tracker.roi_top * calib.height)
    start = max(0, int(calib.horizon_row + 2.0) - roi_row)
    body = gray[roi_row + start:, :]
    return tracker._binarize(body, polarity), roi_row, start


# --- akurasi pada frame sintetis ---

@pytest.mark.parametrize("polarity", ["light_on_dark", "dark_on_light"])
def test_straight_lane_recovered_to_mm(tracker, calib, polarity):
    truth = LaneTruth(a=0.0, b=0.0, c=0.0, polarity=polarity)
    frame, gt = render_lane_frame(calib, truth, seed=1)
    obs = tracker.estimate(frame, calib, 0.0)
    assert obs.confidence > 0.5
    assert obs.lane_error_m == pytest.approx(gt["lateral_m"], abs=0.01)
    assert obs.heading_error_rad == pytest.approx(gt["heading_rad"], abs=0.02)


@pytest.mark.parametrize("truth", [
    LaneTruth(a=0.02, b=0.01, c=0.05),      # lengkung halus + offset
    LaneTruth(a=-0.05, b=-0.03, c=-0.08),   # lengkung berlawanan
    LaneTruth(a=0.15, b=0.0, c=0.10),       # tikungan tajam (R ~3.3 m)
    LaneTruth(a=0.0, b=0.0587, c=0.1189),   # miring kuat (kasus bias lama)
    LaneTruth(a=0.0195, b=0.0587, c=0.1189),
])
def test_curved_and_offset_lanes_recovered(tracker, calib, truth):
    frame, gt = render_lane_frame(calib, truth, seed=3)
    obs = tracker.estimate(frame, calib, 0.0)
    assert obs.confidence > 0.5
    # Dulu kasus b besar ini bias tepat +/- setengah lebar lajur (25 cm).
    assert obs.lane_error_m == pytest.approx(gt["lateral_m"], abs=0.02)
    assert obs.heading_error_rad == pytest.approx(gt["heading_rad"], abs=0.05)


def test_many_random_scenarios_within_target(tracker, calib):
    from mizuro.perception.synth import random_truth
    rng = np.random.default_rng(99)
    lat_err, head_err, no_lane = [], [], 0
    for i in range(40):
        truth = random_truth(rng)
        frame, gt = render_lane_frame(calib, truth, seed=i)
        obs = tracker.estimate(frame, calib, float(i))
        if obs.confidence == 0:
            no_lane += 1
            continue
        lat_err.append(abs(obs.lane_error_m - gt["lateral_m"]))
        head_err.append(abs(obs.heading_error_rad - gt["heading_rad"]))
    # Target Fase 1: median < 3 cm lateral, < 5 deg heading.
    assert np.median(lat_err) < 0.03
    assert np.median(head_err) < np.radians(5.0)
    assert no_lane <= 2   # hanya skenario lajur-keluar-frame yang mustahil


# --- penolakan frame buruk ---

def test_blank_frame_no_lane(tracker, calib):
    blank = np.full((480, 640, 3), FLOOR_DARK, dtype=np.uint8)
    obs = tracker.estimate(blank, calib, 0.0)
    assert obs.confidence == 0.0
    assert obs.detections[0]["kind"] == "lane"


def test_none_frame_no_lane(tracker, calib):
    obs = tracker.estimate(None, calib, 0.0)
    assert obs.confidence == 0.0


def test_wrong_size_frame_no_lane(tracker, calib):
    frame, _ = render_lane_frame(calib, LaneTruth(0, 0, 0), seed=1)
    obs = tracker.estimate(frame[:240], calib, 0.0)
    assert obs.confidence == 0.0
    assert "ukuran frame" in obs.detections[0]["reason"]


def test_explicit_polarity_is_honored(tracker, calib):
    # Override eksplisit dipakai (bukan auto), dan frame tetap terbaca.
    truth = LaneTruth(0, 0, 0, polarity="light_on_dark")
    frame, gt = render_lane_frame(calib, truth, seed=1)
    tracker.polarity = "light_on_dark"
    obs = tracker.estimate(frame, calib, 0.0)
    assert obs.confidence > 0.5
    assert tracker.last_debug["polarity"] == "light_on_dark"
    assert obs.lane_error_m == pytest.approx(gt["lateral_m"], abs=0.01)


def test_inverted_polarity_still_recovers_two_tone_scene(tracker, calib):
    # Catatan perilaku (bukan jaminan): pada scene DUA-TONE seperti synth,
    # polaritas terbalik membuat mask = lantai, dan "run lantai di antara
    # dua tepi" pusatnya tetap tepat di pusat lajur — jadi tracker masih
    # recover. Di lantai nyata (tekstur, glare) ini TIDAK dijamin; heuristik
    # auto dari far-field median tetap jalur utama.
    truth = LaneTruth(0, 0, 0, polarity="light_on_dark")
    frame, gt = render_lane_frame(calib, truth, seed=1)
    tracker.polarity = "dark_on_light"      # salah dengan sengaja
    obs = tracker.estimate(frame, calib, 0.0)
    assert tracker.last_debug["polarity"] == "dark_on_light"
    assert obs.confidence > 0.5
    assert obs.lane_error_m == pytest.approx(gt["lateral_m"], abs=0.02)


def test_estimate_is_deterministic(tracker, calib):
    truth = LaneTruth(0.03, -0.02, 0.07)
    frame, gt = render_lane_frame(calib, truth, seed=5)
    a = tracker.estimate(frame, calib, 0.0)
    b = tracker.estimate(frame, calib, 0.0)
    assert a.lane_error_m == b.lane_error_m
    assert a.heading_error_rad == b.heading_error_rad


def test_debug_dict_populated(tracker, calib):
    frame, _ = render_lane_frame(calib, LaneTruth(0, 0, 0), seed=1)
    tracker.estimate(frame, calib, 0.0)
    dbg = tracker.last_debug
    assert dbg["stations_total"] == tracker.n_stations
    assert 0.0 <= dbg["coverage"] <= 1.0
    assert dbg["polarity"] in ("light_on_dark", "dark_on_light")
    assert dbg["rms_m"] is not None and dbg["rms_m"] < 0.02


# --- logika stasiun ---

def _row_runs(tracker, binary, calib, y, x_pred, roi_row, start):
    """Run centers (piksel) di baris stasiun y, window sekitar x_pred."""
    u_pred, v = calib.ground_to_pixel(y, x_pred)
    u_lo, _ = calib.ground_to_pixel(y, x_pred - 0.4)
    u_hi, _ = calib.ground_to_pixel(y, x_pred + 0.4)
    lo = max(0, int(np.floor(min(u_lo, u_hi))))
    hi = min(639, int(np.ceil(max(u_lo, u_hi))))
    row_idx = int(round(float(v))) - roi_row - start
    row = binary[row_idx, lo:hi + 1] > 0
    d = np.diff(np.concatenate(([0], row.view(np.int8), [0])))
    s = np.flatnonzero(d == 1)
    e = np.flatnonzero(d == -1)
    return (s + e - 1) / 2.0 + lo


def test_three_runs_resolve_to_dash_center(tracker, calib):
    # Stasiun dengan tepi+dash+tepi: run tengah = dash = pusat lajur.
    truth = LaneTruth(0.0, 0.0, 0.0, polarity="light_on_dark")
    frame, _ = render_lane_frame(calib, truth, seed=2, noise_sigma=0.0,
                                 blur_sigma=0.0)
    binary, roi_row, start = _binary_for(tracker, frame, calib,
                                         "light_on_dark")
    # Cari stasiun dengan tepat 3 run.
    for y in np.linspace(0.5, 2.0, 16):
        runs = _row_runs(tracker, binary, calib, y, 0.0, roi_row, start)
        if len(runs) == 3:
            hit = tracker._station_center(binary, calib, float(y), 0.0,
                                          roi_row, start, 640, 480)
            assert hit is not None
            assert hit[0] == pytest.approx(float(np.median(runs)))
            assert hit[2] == 3
            return
    pytest.skip("tidak ada stasiun 3-run di frame ini")


def test_two_edges_resolve_to_midpoint(tracker, calib):
    truth = LaneTruth(0.0, 0.0, 0.0, polarity="light_on_dark")
    frame, _ = render_lane_frame(calib, truth, seed=2, noise_sigma=0.0,
                                 blur_sigma=0.0)
    binary, roi_row, start = _binary_for(tracker, frame, calib,
                                         "light_on_dark")
    for y in np.linspace(0.5, 2.0, 16):
        runs = _row_runs(tracker, binary, calib, y, 0.0, roi_row, start)
        if len(runs) == 2:
            gap_ground = abs(float(calib.pixel_to_ground(
                runs[1], float(calib.ground_to_pixel(y, 0.0)[1]))[1])
                - float(calib.pixel_to_ground(
                    runs[0], float(calib.ground_to_pixel(y, 0.0)[1]))[1]))
            if gap_ground > 0.4:      # dua tepi (0.5 m), bukan tepi+dash
                hit = tracker._station_center(binary, calib, float(y), 0.0,
                                              roi_row, start, 640, 480)
                assert hit[0] == pytest.approx(float((runs[0] + runs[1]) / 2))
                return
    pytest.skip("tidak ada stasiun 2-tepi di frame ini")


def test_single_run_skipped_in_pass1_but_used_in_pass2(tracker, calib):
    # Tikungan tajam dengan lajur keluar frame: stasiun dekat hanya
    # memungut satu tepi (tepi lain di luar frame) -> 1-run = ambigu.
    truth = LaneTruth(a=-0.2066, b=-0.0038, c=-0.0762,
                      polarity="light_on_dark")
    frame, _ = render_lane_frame(calib, truth, seed=7)
    binary, roi_row, start = _binary_for(tracker, frame, calib,
                                         "light_on_dark")
    found_single = False
    for y in np.linspace(0.5, 2.0, 16):
        runs = _row_runs(tracker, binary, calib, y, 0.0, roi_row, start)
        if len(runs) == 1:
            found_single = True
            # Pass 1 (allow_ambiguous=False): dilewati, tidak ditebak.
            assert tracker._station_center(binary, calib, float(y), 0.0,
                                           roi_row, start, 640, 480,
                                           allow_ambiguous=False) is None
            # Pass 2 dengan prediksi benar: resolve ke pusat lajur.
            hit = tracker._station_center(binary, calib, float(y),
                                          truth.x_at(y), roi_row, start,
                                          640, 480, allow_ambiguous=True)
            assert hit is not None
            _, x_got = calib.pixel_to_ground(hit[0], hit[1])  # (fwd, right)
            assert float(x_got) == pytest.approx(truth.x_at(y), abs=0.02)
    assert found_single, "frame uji tidak memuat stasiun 1-run"


def test_half_width_constant_matches_guidebook():
    # Lebar lajur 0.50 m -> setengah 0.25 m.
    assert LANE_HALF_WIDTH_M == 0.25


def test_tracker_survives_rolled_camera(tracker, calib):
    # Kamera roll 3 derajat: frame synth konsisten (dinding miring,
    # lajur diproyeksikan lewat transformasi roll) dan tracker harus
    # tetap recover — ROI crop pakai horizon_row_worst.
    rolled = CameraCalibration.from_hfov(640, 480, 0.16, 12.0,
                                         mount_roll_deg=3.0)
    truth = LaneTruth(0.02, 0.01, 0.05, polarity="light_on_dark")
    frame, gt = render_lane_frame(rolled, truth, seed=4)
    obs = tracker.estimate(frame, rolled, 0.0)
    assert obs.confidence > 0.5
    assert obs.lane_error_m == pytest.approx(gt["lateral_m"], abs=0.02)
    assert obs.heading_error_rad == pytest.approx(gt["heading_rad"], abs=0.05)


def test_tracker_with_offset_mount_stays_accurate(tracker, calib):
    # Kamera maju 10 cm dari poras: output tracker adalah state lajur
    # di frame ROBOT pada stasiun lookahead — ground truth synth juga
    # robot-frame, jadi akurasi harus tetap mm-level.
    moved = CameraCalibration.from_hfov(640, 480, 0.16, 12.0,
                                        mount_x_m=0.10, mount_y_m=0.02,
                                        mount_yaw_deg=2.0)
    truth = LaneTruth(-0.03, 0.02, -0.06, polarity="dark_on_light")
    frame, gt = render_lane_frame(moved, truth, seed=6)
    obs = tracker.estimate(frame, moved, 0.0)
    assert obs.confidence > 0.5
    assert obs.lane_error_m == pytest.approx(gt["lateral_m"], abs=0.02)
    assert obs.heading_error_rad == pytest.approx(gt["heading_rad"], abs=0.05)
