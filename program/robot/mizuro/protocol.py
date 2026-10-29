"""Protokol serial Jetson <-> MCU bodi.

Setpoint (Jetson -> MCU), satu baris ASCII per paket:

    speed_mps,steering_rad\\n

dengan tepat empat angka desimal. Batasnya juga ditegakkan firmware MCU;
mengubah batas berarti mengubah DUA sisi sekaligus.

MCU memutus penggerak jika tidak ada setpoint segar dalam WATCHDOG_MS,
sehingga loop runtime HARUS mengirim setiap iterasi — bahkan nol saat
berhenti. Itu yang membuat send() berfungsi sebagai heartbeat.
"""
from __future__ import annotations

from .types import Command

DEFAULT_BAUD = 115200
MAX_SPEED_MPS = 0.8
MAX_STEER_RAD = 0.55
WATCHDOG_MS = 100
DECIMALS = 4


def _fixed(value: float) -> str:
    """Format dengan DECIMALS desimal; "-0.0000" dinormalkan jadi "0.0000"
    supaya wire dan log tidak pernah memuat negatif-nol."""
    text = f"{value:.{DECIMALS}f}"
    if text.startswith("-") and float(text) == 0.0:
        text = text[1:]
    return text


def format_command(command: Command) -> bytes:
    """Encode Command menjadi baris protokol.

    Di luar batas (termasuk NaN/inf) melempar ValueError — jangan pernah
    mengirim nilai yang tidak bisa ditegakkan MCU.
    """
    if not -MAX_SPEED_MPS <= command.speed_mps <= MAX_SPEED_MPS:
        raise ValueError(
            f"speed_mps {command.speed_mps} di luar batas ±{MAX_SPEED_MPS}")
    if not -MAX_STEER_RAD <= command.steering_rad <= MAX_STEER_RAD:
        raise ValueError(
            f"steering_rad {command.steering_rad} di luar batas ±{MAX_STEER_RAD}")
    line = f"{_fixed(command.speed_mps)},{_fixed(command.steering_rad)}\n"
    return line.encode("ascii")
