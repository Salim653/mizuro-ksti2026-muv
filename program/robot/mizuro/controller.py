"""Controller Fase 3: Stanley + speed profile berbasis kurvatur.

Stanley (Hoffmann et al. 2007) menggabungkan dua suku:
    steer = heading_gain * heading_error
          + atan(lateral_gain * lane_error / (v + v_soft))

Suku pertama membetulkan heading robot relatif lajur; suku kedua
mengoreksi cross-track error dengan gain yang melunak seiring
kecepatan (mencegah overshoot di cepat, tetap responsif di lambat).

Speed profile: kecepatan target dari kurvatur lajur —
    v = sqrt(a_lat_max * R)
sehingga akselerasi lateral TIDAK PERNAH melebihi max_lateral_accel,
berapa pun tajam belokannya. Output dibatasi [min_speed, max_speed]
dan di-smooth dengan clamp akselerasi/rem supaya tidak jerked.

Konvensi tanda (terverifikasi dari geometry.py + firmware):
- lane_error_m > 0     = lajur di KANAN robot
- heading_error_rad > 0 = arah lajur mengarah ke KANAN heading robot
- curvature_inv_m > 0  = lajur belok kanan
- steering_rad > 0     = belok kanan (firmware: pulse > 1500 us)

Confidence 0 atau lane_error None = "tidak ada estimasi lajur" →
perintah NOL (speed 0, steer 0). Ini bukan fallback, ini kontrak:
controller tidak boleh menebak arah ketika persepsi buta.
"""
from __future__ import annotations

import math

from .types import Command, Observation


class StanleyController:
    """Stanley steering + curvature-based speed, satu frame -> satu Command.

    State antar-frame: kecepatan terakhir (buat smoothing akselerasi)
    dan timestamp terakhir (buat dt). Tidak menyimpan histori lajur —
    setiap frame berdiri sendiri, sama seperti tracker.
    """

    def __init__(self, *, lookahead_m: float = 0.6,
                 heading_gain: float = 0.8,
                 lateral_gain: float = 1.2,
                 max_lateral_accel: float = 3.0,
                 max_speed_mps: float = 0.8,
                 steering_limit_rad: float = 0.55,
                 min_speed_mps: float = 0.15,
                 max_accel_mps2: float = 0.5,
                 max_decel_mps2: float = 1.0,
                 stanley_softening_mps: float = 0.1,
                 min_curve_radius_m: float = 1.5) -> None:
        self.lookahead_m = lookahead_m
        self.heading_gain = heading_gain
        self.lateral_gain = lateral_gain
        self.max_lateral_accel = max_lateral_accel
        self.max_speed_mps = max_speed_mps
        self.steering_limit_rad = steering_limit_rad
        self.min_speed_mps = min_speed_mps
        self.max_accel_mps2 = max_accel_mps2
        self.max_decel_mps2 = max_decel_mps2
        self.stanley_softening_mps = stanley_softening_mps
        self.min_curve_radius_m = min_curve_radius_m
        self._prev_speed: float | None = None
        self._prev_time: float | None = None

    # -- steering ------------------------------------------------------

    def steer(self, obs: Observation, speed_mps: float) -> float:
        """Stanley steering (rad), sudah di-clamp ke steering_limit."""
        heading = obs.heading_error_rad or 0.0
        lateral = obs.lane_error_m or 0.0
        # atan melunak di kecepatan rendah: v + v_soft mencegah div/0
        # dan menjaga responsif saat robot hampir berhenti.
        cross = math.atan(
            self.lateral_gain * lateral / (speed_mps + self.stanley_softening_mps))
        raw = self.heading_gain * heading + cross
        return max(-self.steering_limit_rad,
                   min(self.steering_limit_rad, raw))

    # -- speed ----------------------------------------------------------

    def target_speed(self, obs: Observation) -> float:
        """Kecepatan target dari kurvatur lajur (m/s), belum di-smooth.

        Lurus (kurvatur ~0 atau None) -> max_speed.
        Belok -> sqrt(a_lat * R), dibatasi R >= min_curve_radius_m
        supaya kecepatan tidak jadi mustahil diukur pada R ekstrem.
        """
        kappa = obs.curvature_inv_m
        if kappa is None or abs(kappa) < 1e-6:
            return self.max_speed_mps
        radius = 1.0 / abs(kappa)
        radius = max(radius, self.min_curve_radius_m)
        v = math.sqrt(self.max_lateral_accel * radius)
        return max(self.min_speed_mps, min(self.max_speed_mps, v))

    def _smooth_speed(self, target: float, now: float) -> float:
        """Batasi perubahan kecepatan dengan akselerasi/rem maksimum.

        Frame pertama (belum ada _prev_speed): mulai dari target
        langsung — robot belum bergerak, tidak ada inersia yang dilanggar.
        """
        if self._prev_speed is None or self._prev_time is None:
            self._prev_speed = target
            self._prev_time = now
            return target
        dt = now - self._prev_time
        if dt <= 0.0:
            # Timestamp tidak maju (frame duplikat): tahan kecepatan.
            return self._prev_speed
        dv = target - self._prev_speed
        if dv >= 0.0:
            max_dv = self.max_accel_mps2 * dt
        else:
            max_dv = self.max_decel_mps2 * dt
        speed = self._prev_speed + max(-max_dv, min(max_dv, dv))
        speed = max(0.0, min(self.max_speed_mps, speed))
        self._prev_speed = speed
        self._prev_time = now
        return speed

    # -- antarmuka utama -------------------------------------------------

    def command(self, obs: Observation) -> Command:
        """Satu Observation -> satu Command. Tidak pernah melempar.

        Confidence 0 / lane_error None -> Command nol (robot berhenti).
        """
        if obs.confidence <= 0.0 or obs.lane_error_m is None:
            self._prev_speed = None
            self._prev_time = None
            return Command(speed_mps=0.0, steering_rad=0.0,
                           reason="tidak ada estimasi lajur")

        target = self.target_speed(obs)
        speed = self._smooth_speed(target, obs.timestamp)
        steer = self.steer(obs, speed)
        return Command(speed_mps=speed, steering_rad=steer,
                       reason="stanley")

    def reset(self) -> None:
        """Bersihkan state smoothing (dipanggil saat lane hilang / restart)."""
        self._prev_speed = None
        self._prev_time = None

    @classmethod
    def from_config(cls, config) -> StanleyController:
        """Bangun dari Config — semua gain dari satu tempat."""
        return cls(
            lookahead_m=config.lookahead_m,
            heading_gain=config.heading_gain,
            lateral_gain=config.lateral_gain,
            max_lateral_accel=config.max_lateral_accel,
            max_speed_mps=config.max_speed_mps,
            steering_limit_rad=config.steering_limit_rad,
            min_speed_mps=config.min_speed_mps,
            max_accel_mps2=config.max_accel_mps2,
            max_decel_mps2=config.max_decel_mps2,
            stanley_softening_mps=config.stanley_softening_mps,
            min_curve_radius_m=config.min_curve_radius_m)
