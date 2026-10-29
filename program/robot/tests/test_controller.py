"""Tes controller Fase 3: Stanley steering + speed profile + smoothing.

Konvensi tanda yang diuji (terverifikasi dari geometry.py + firmware):
- lane_error > 0     = lajur di KANAN robot  -> steer positif (kanan)
- heading_error > 0  = lajur mengarah KANAN   -> steer positif (kanan)
- curvature_inv > 0  = belok kanan
"""
import math

import pytest

from mizuro.controller import StanleyController
from mizuro.types import Command, Observation


def _obs(lane=0.0, heading=0.0, curvature=None, conf=0.9, t=0.0):
    return Observation(timestamp=t, lane_error_m=lane,
                       heading_error_rad=heading,
                       curvature_inv_m=curvature, confidence=conf)


def _ctrl(**kw):
    defaults = dict(lookahead_m=0.6, heading_gain=0.8, lateral_gain=1.2,
                    max_lateral_accel=3.0, max_speed_mps=0.8,
                    steering_limit_rad=0.55, min_speed_mps=0.15,
                    max_accel_mps2=0.5, max_decel_mps2=1.0,
                    stanley_softening_mps=0.1, min_curve_radius_m=1.5)
    defaults.update(kw)
    return StanleyController(**defaults)


# --- steering -----------------------------------------------------------

def test_steer_zero_when_lane_centered_and_aligned():
    c = _ctrl()
    assert c.steer(_obs(lane=0.0, heading=0.0), speed_mps=0.5) == pytest.approx(0.0)


def test_steer_right_when_lane_right():
    """Lajur di kanan robot -> steer positif (kanan)."""
    c = _ctrl()
    s = c.steer(_obs(lane=0.1, heading=0.0), speed_mps=0.5)
    assert s > 0.0


def test_steer_left_when_lane_left():
    """Lajur di kiri robot -> steer negatif (kiri)."""
    c = _ctrl()
    s = c.steer(_obs(lane=-0.1, heading=0.0), speed_mps=0.5)
    assert s < 0.0


def test_steer_corrects_heading_right():
    """Heading error positif (lajur mengarah kanan) -> steer positif."""
    c = _ctrl()
    s = c.steer(_obs(lane=0.0, heading=0.2), speed_mps=0.5)
    assert s > 0.0
    assert s == pytest.approx(0.8 * 0.2, abs=1e-9)   # murni suku heading


def test_steer_corrects_heading_left():
    c = _ctrl()
    s = c.steer(_obs(lane=0.0, heading=-0.2), speed_mps=0.5)
    assert s < 0.0


def test_steer_clamped_to_limit():
    """Error besar -> steer tidak pernah melebihi steering_limit."""
    c = _ctrl()
    s = c.steer(_obs(lane=0.5, heading=1.0), speed_mps=0.1)
    assert s <= c.steering_limit_rad + 1e-12
    s2 = c.steer(_obs(lane=-0.5, heading=-1.0), speed_mps=0.1)
    assert s2 >= -c.steering_limit_rad - 1e-12


def test_steer_softens_with_speed():
    """Cross-track gain melunak seiring kecepatan: error sama, steer
    lebih kecil di cepat. Mencegah overshoot."""
    c = _ctrl()
    slow = c.steer(_obs(lane=0.1), speed_mps=0.1)
    fast = c.steer(_obs(lane=0.1), speed_mps=0.7)
    assert abs(fast) < abs(slow)


def test_steer_no_div_by_zero_at_zero_speed():
    """v=0 tidak boleh crash — softening mencegah div/0."""
    c = _ctrl()
    s = c.steer(_obs(lane=0.1, heading=0.0), speed_mps=0.0)
    assert math.isfinite(s)


# --- speed profile ------------------------------------------------------

def test_speed_max_on_straight():
    c = _ctrl()
    assert c.target_speed(_obs(curvature=None)) == pytest.approx(0.8)
    assert c.target_speed(_obs(curvature=0.0)) == pytest.approx(0.8)


def test_speed_reduced_in_curve():
    """Belok kanan (curvature positif) -> lebih lambat dari lurus.
    a_lat kecil + floor R rendah supaya profil aktif di rentang robot."""
    c = _ctrl(max_lateral_accel=0.5, min_curve_radius_m=0.1)
    v = c.target_speed(_obs(curvature=1.0 / 1.0))   # R=1m
    assert v < 0.8
    assert v == pytest.approx(math.sqrt(0.5 * 1.0), rel=1e-6)


def test_speed_symmetric_for_left_right():
    """Arah belok tidak mempengaruhi kecepatan — hanya |kurvatur|."""
    c = _ctrl(max_lateral_accel=0.5, min_curve_radius_m=0.1)
    right = c.target_speed(_obs(curvature=1.0 / 1.0))
    left = c.target_speed(_obs(curvature=-1.0 / 1.0))
    assert right == pytest.approx(left)


def test_speed_clamped_to_min_on_tight_curve():
    """R sangat kecil -> speed tidak bisa di bawah min_speed."""
    c = _ctrl(max_lateral_accel=0.2, min_curve_radius_m=0.01)
    v = c.target_speed(_obs(curvature=1.0 / 0.05))   # R=0.05m
    assert v == pytest.approx(c.min_speed_mps)


def test_speed_clamped_to_max_on_gentle_curve():
    c = _ctrl(max_lateral_accel=0.5, min_curve_radius_m=0.1)
    v = c.target_speed(_obs(curvature=1.0 / 100.0))  # R=100m
    assert v == pytest.approx(c.max_speed_mps)


def test_speed_respects_min_curve_radius_floor():
    """R di bawah min_curve_radius_m dipakai batas bawah itu, bukan R asli."""
    c = _ctrl(max_lateral_accel=0.4, min_curve_radius_m=1.5)
    v = c.target_speed(_obs(curvature=1.0 / 0.5))    # R=0.5m < 1.5m
    assert v == pytest.approx(math.sqrt(0.4 * 1.5), rel=1e-6)


# --- speed smoothing -----------------------------------------------------

def test_smooth_first_frame_uses_target():
    c = _ctrl()
    v = c._smooth_speed(0.5, now=0.0)
    assert v == pytest.approx(0.5)


def test_smooth_limits_acceleration():
    """Dari 0 ke 0.8 butuh waktu; tidak bisa instan."""
    c = _ctrl(max_accel_mps2=0.5)
    c._smooth_speed(0.0, now=0.0)
    v = c._smooth_speed(0.8, now=1.0)   # dt=1s, max dv=0.5
    assert v == pytest.approx(0.5)


def test_smooth_limits_braking():
    """Rem bisa lebih agresif dari akselerasi."""
    c = _ctrl(max_decel_mps2=1.0)
    c._smooth_speed(0.8, now=0.0)
    v = c._smooth_speed(0.0, now=0.5)   # dt=0.5s, max dv=0.5
    assert v == pytest.approx(0.3)


def test_smooth_holds_on_duplicate_timestamp():
    """dt <= 0 (frame duplikat) -> tahan kecepatan, tidak crash."""
    c = _ctrl()
    c._smooth_speed(0.5, now=1.0)
    v = c._smooth_speed(0.8, now=1.0)
    assert v == pytest.approx(0.5)


def test_smooth_never_negative():
    c = _ctrl()
    c._smooth_speed(0.2, now=0.0)
    v = c._smooth_speed(0.0, now=10.0)
    assert v >= 0.0


# --- command integration --------------------------------------------------

def test_command_stops_when_no_lane():
    """Confidence 0 -> perintah NOL. Kontrak keselamatan."""
    c = _ctrl()
    cmd = c.command(_obs(conf=0.0))
    assert cmd.speed_mps == 0.0
    assert cmd.steering_rad == 0.0
    assert cmd.reason == "tidak ada estimasi lajur"


def test_command_stops_when_lane_error_none():
    c = _ctrl()
    cmd = c.command(_obs(lane=None, conf=0.9))
    assert cmd.speed_mps == 0.0
    assert cmd.steering_rad == 0.0


def test_command_returns_valid_command():
    """Output harus selalu dalam batas protokol."""
    c = _ctrl()
    cmd = c.command(_obs(lane=0.05, heading=0.1, curvature=1.0 / 5.0, t=0.0))
    assert isinstance(cmd, Command)
    assert 0.0 <= cmd.speed_mps <= 0.8
    assert -0.55 <= cmd.steering_rad <= 0.55
    assert cmd.reason == "stanley"


def test_command_resets_smoothing_on_lane_loss():
    """Setelah lane hilang, state smoothing dibersihkan supaya start
    berikutnya tidak mewarisi kecepatan lama."""
    c = _ctrl()
    c.command(_obs(lane=0.0, heading=0.0, curvature=None, t=0.0))
    c.command(_obs(lane=0.1, heading=0.0, curvature=None, t=0.1))
    c.command(_obs(conf=0.0, t=0.2))          # lane hilang
    assert c._prev_speed is None
    assert c._prev_time is None


def test_command_slows_entering_curve():
    """Sequensi: lurus lalu belok -> speed turun secara bertahap (smooth).
    Kurva tajam + rem limited supaya slowdown butuh >1 frame."""
    c = _ctrl(max_lateral_accel=0.5, min_curve_radius_m=0.1,
              max_decel_mps2=0.5)
    v1 = c.command(_obs(curvature=None, t=0.0)).speed_mps
    v2 = c.command(_obs(curvature=1.0 / 0.3, t=0.5)).speed_mps
    v3 = c.command(_obs(curvature=1.0 / 0.3, t=1.0)).speed_mps
    assert v1 > v2 > v3                        # turun bertahap, bukan instan
    assert v3 == pytest.approx(math.sqrt(0.5 * 0.3), rel=0.5)


def test_reset_clears_state():
    c = _ctrl()
    c.command(_obs(t=0.0))
    c.reset()
    assert c._prev_speed is None
    assert c._prev_time is None


# --- sign convention end-to-end -------------------------------------------

def test_sign_consistency_lane_right_steers_right():
    """End-to-end: lajur di kanan -> steer positif (satu arah dengan
    konvensi firmware)."""
    c = _ctrl()
    cmd = c.command(_obs(lane=0.15, heading=0.0, curvature=None, t=0.0))
    assert cmd.steering_rad > 0.0


def test_sign_consistency_lane_left_steers_left():
    c = _ctrl()
    cmd = c.command(_obs(lane=-0.15, heading=0.0, curvature=None, t=0.0))
    assert cmd.steering_rad < 0.0


def test_sign_consistency_heading_right_steers_right():
    c = _ctrl()
    cmd = c.command(_obs(lane=0.0, heading=0.15, curvature=None, t=0.0))
    assert cmd.steering_rad > 0.0
