"""Tipe data bersama antarmodul.

Command adalah muatan protokol serial (lihat protocol.py). Observation
adalah hasil persepsi yang dikonsumsi controller: confidence 0 berarti
"tidak ada estimasi lajur" dan WAJIB menghasilkan perintah nol.
"""
from __future__ import annotations

from typing import Any, NamedTuple


class Command(NamedTuple):
    """Satu setpoint ke MCU: maju + kemudi, plus alasan untuk log."""

    speed_mps: float = 0.0
    steering_rad: float = 0.0
    reason: str = "safe default"


class Observation(NamedTuple):
    """Estimasi lajur pada satu timestamp.

    lane_error_m positif = lajur di kanan robot (perlu belok kanan).
    heading_error_rad positif = heading robot di kanan arah lajur.
    curvature_inv_m = 1/R lajur pada titik acuan (positif = belok kiri).
    """

    timestamp: float
    lane_error_m: float | None = None
    heading_error_rad: float | None = None
    curvature_inv_m: float | None = None
    confidence: float = 0.0
    detections: tuple[dict[str, Any], ...] = ()


class McuStatus(NamedTuple):
    """Status MCU yang dikirimkan ke Jetson (10 Hz, non-blocking).

    Dibuat oleh parse_status_line() di protocol.py dari baris
    `#v1,<mode>,<estop>,<stale>,<speed_out>,<steer_out>,<rx_ok>,<uptime_ms>`.
    """

    mode: str            # "esc1" | "hb1" | "skid4"
    estop: bool          # True = tombol E-stop tertekan, penggerak dipotong
    stale: bool          # True = watchdog MCU trip / belum ada setpoint segar
    speed_out_mps: float # kecepatan yang benar-benar diterapkan (0 = dipotong)
    steer_out_rad: float # sudut kemudi yang diterapkan (0 = dipotong)
    rx_ok: int           # jumlah setpoint valid yang diterima MCU
    uptime_ms: int       # millis() MCU saat baris dibuat
