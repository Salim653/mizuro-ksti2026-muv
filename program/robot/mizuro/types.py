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
