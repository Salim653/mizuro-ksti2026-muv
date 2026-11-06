"""Protokol serial Jetson <-> MCU bodi.

Setpoint (Jetson -> MCU), satu baris ASCII per paket:

    speed_mps,steering_rad\\n

dengan tepat empat angka desimal. Batasnya juga ditegakkan firmware MCU;
mengubah batas berarti mengubah DUA sisi sekaligus.

Pemilihan mode output (Jetson -> MCU), satu baris:

    #M,<mode>\\n

dengan mode: `esc1` (ESC brushless + servo kemudi, target kompetisi),
`hb1` (satu motor PWM/DIR + servo kemudi, bench), atau `skid4`
(prototype: 4x L298N, skid steer, mixing di MCU). Dikirim sekali saat
McuLink dibuka supaya firmware tahu tahap output mana yang dipakai —
robot.json adalah sumber kebenaran, bukan firmware yang di-flash.

Status (MCU -> Jetson), satu baris 10 Hz:

    #v1,<mode>,<estop>,<stale>,<speed_out>,<steer_out>,<rx_ok>,<uptime_ms>\\n

Di-parse oleh parse_status_line(). Hanya field whitelist yang dibaca;
baris apa pun yang tidak persis bentuk ini diabaikan.

MCU memutus penggerak jika tidak ada setpoint segar dalam WATCHDOG_MS,
sehingga loop runtime HARUS mengirim setiap iterasi — bahkan nol saat
berhenti. Itu yang membuat send() berfungsi sebagai heartbeat.
"""
from __future__ import annotations

import math

from .types import Command, McuStatus

DEFAULT_BAUD = 115200
MAX_SPEED_MPS = 0.8
MAX_STEER_RAD = 0.55
WATCHDOG_MS = 100
DECIMALS = 4

# motor_driver di config (nama hardware) -> kode mode firmware MCU.
# SUMBER KEBENARAN TUNGGAL untuk kedua sisi; menambah hardware baru
# berarti menambah entri di sini DAN cabang mode di body_controller.ino.
FIRMWARE_MODES = {
    "external_esc": "esc1",    # 1x ESC brushless + servo kemudi (kompetisi)
    "mcu_hbridge": "hb1",      # 1x motor PWM/DIR + servo kemudi (bench)
    "skid4_l298n": "skid4",    # prototype: 4x L298N, skid steer
}
FIRMWARE_MODE_NAMES = frozenset(FIRMWARE_MODES.values())

# Awalan baris status MCU -> Jetson (v1 = format pertama).
STATUS_PREFIX = "#v1,"
STATUS_FIELDS = 8   # tag + mode + estop + stale + speed + steer + rx_ok + uptime


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


def format_mode(driver: str) -> bytes:
    """Encode pilihan driver jadi baris perintah mode MCU.

    Dibuat satu baris #M,<mode>\\n; MCU mengganti tahap output tanpa
    perlu di-flash ulang. ValueError kalau driver tidak dikenal — config
    yang salah harus gagal di sini, bukan jadi output salah di roda.
    """
    try:
        mode = FIRMWARE_MODES[driver]
    except KeyError:
        raise ValueError(
            f"motor_driver tidak dikenal: {driver!r} "
            f"(pilihan: {sorted(FIRMWARE_MODES)})") from None
    return f"#M,{mode}\n".encode("ascii")


def parse_status_line(line: str) -> McuStatus | None:
    """Parse baris status MCU. None kalau baris tidak sesuai format atau
    data di luar batas protokol — bukan error fatal, hanya diabaikan.
    """
    text = line.strip()
    if not text.startswith(STATUS_PREFIX):
        return None
    parts = text.split(",")
    if len(parts) != STATUS_FIELDS:
        return None
    _, mode, estop, stale, speed, steer, rx_ok, uptime = parts
    if mode not in FIRMWARE_MODE_NAMES:
        return None
    if estop not in ("0", "1") or stale not in ("0", "1"):
        return None
    try:
        speed_v = float(speed)
        steer_v = float(steer)
        rx = int(rx_ok)
        up = int(uptime)
    except ValueError:
        return None
    if not (math.isfinite(speed_v) and math.isfinite(steer_v)):
        return None
    if abs(speed_v) > MAX_SPEED_MPS + 1e-6 or abs(steer_v) > MAX_STEER_RAD + 1e-6:
        return None
    if rx < 0 or up < 0:
        return None
    return McuStatus(mode, estop == "1", stale == "1", speed_v, steer_v, rx, up)
