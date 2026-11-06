#!/usr/bin/env python3
"""Virtual MCU — cermin pemetaan output firmware body_controller.ino.

Bench tanpa hardware: jalankan pemetaan output firmware (speed+steer ->
PWM per roda / pulse ESC / pulse servo) DI PYTHON, rekam apa yang AKAN
dikirim ke roda. TIDAK PERNAH menyentuh serial atau hardware.

    from tools.virtual_mcu import VirtualMcu
    mcu = VirtualMcu("skid4_l298n")
    mcu.send(Command(0.4, -0.1))
    mcu.last_output["wheels"]   # {'FL': (1, 134), 'RL': (1, 134), ...}

Cermin, BUKAN sumber kebenaran: authority tetap
`program/body-mcu/body_controller.ino` + tes host `test/run_tests.sh`.
Konstanta di bawah sengaja ditulis ulang dari firmware (tidak di-import —
firmware C++ tidak bisa di-import); kalau angka di sini beda dari tes
host, firmware yang benar dan cermin ini yang harus diperbaiki.
`tests/test_virtual_mcu.py` mengunci nilai-nilai yang sudah diverifikasi
di tes host (134/121, 193/255, 35, 1770, 1569, 1573).
"""
from __future__ import annotations

import sys
import time
from collections import deque
from pathlib import Path
from typing import NamedTuple

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mizuro.protocol import FIRMWARE_MODES, parse_status_line  # noqa: E402
from mizuro.types import Command, McuStatus  # noqa: E402

# ---------------------------------------------------------------------------
# Cermin konstanta firmware (body_controller.ino) — ubah DUA SISI bersamaan
# ---------------------------------------------------------------------------
MAX_SPEED_MPS = 0.8
MAX_STEER_RAD = 0.55
MOTOR_PWM_MAX = 255
MOTOR_DIR_FORWARD = 1          # HIGH di firmware
WATCHDOG_S = 0.100             # WATCHDOG_MS
STATUS_PERIOD_S = 0.100        # STATUS_PERIOD_MS (10 Hz)

# mode skid4 (prototype) — placeholder kalibrasi, sama seperti firmware
SKID_TRACK_M = 0.20
SKID_MAX_YAW_RATE = 2.0

# mode esc1/hb1
STEER_CENTER_US = 1500
STEER_MIN_US = 1100
STEER_MAX_US = 1900
ESC_NEUTRAL_US = 1500
ESC_MIN_US = 1000
ESC_MAX_US = 2000
ESC_DEADBAND_US = 40
MIN_COMMAND_MPS = 0.02
ESC_ARM_S = 2.0                # ESC_ARM_MS


def _constrain(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


# ---------------------------------------------------------------------------
# Pemetaan output — cermin fungsi firmware satu per satu
# ---------------------------------------------------------------------------
class SkidWheels(NamedTuple):
    """Hasil mixing skid4: kecepatan roda kiri/kanan + flag clamp."""

    left: float
    right: float
    clamped: bool     # True kalau puncak roda > batas dan dua sisi diskalakan


def skid_wheels(speed_mps: float, steer_rad: float) -> SkidWheels:
    """Cermin applySkid4(): steering_rad = yaw-rate ternormalisasi.

    Beda kecepatan kiri/kanan = yaw * track/2. Kalau puncak roda
    melebihi batas, dua sisi diskalakan BERSAMA supaya rasio belok
    tetap (tidak pernah memotong satu sisi saja).
    """
    yaw = steer_rad * SKID_MAX_YAW_RATE
    delta = yaw * (SKID_TRACK_M * 0.5)
    left = speed_mps - delta
    right = speed_mps + delta
    peak = max(abs(left), abs(right))
    clamped = peak > MAX_SPEED_MPS
    if clamped:
        scale = MAX_SPEED_MPS / peak
        left *= scale
        right *= scale
    return SkidWheels(left, right, clamped)


def hbridge(speed_mps: float) -> tuple[int, int]:
    """Cermin setHBridge(): (level pin DIR, PWM 0..255).

    DIR = MOTOR_DIR_FORWARD untuk maju (termasuk nol), kebalikan untuk
    mundur — persis seperti firmware.
    """
    clamped = _constrain(speed_mps, -MAX_SPEED_MPS, MAX_SPEED_MPS)
    dir_pin = (MOTOR_DIR_FORWARD if clamped >= 0.0
               else 1 - MOTOR_DIR_FORWARD)
    pwm = int(abs(clamped) / MAX_SPEED_MPS * MOTOR_PWM_MAX + 0.5)
    return dir_pin, pwm


def esc_pulse(speed_mps: float) -> int:
    """Cermin escPulse(): pulse ESC 1000..2000 us dengan deadband netral.

    Deadband 40 us: pada gir 1:10, 1 m/s = pulse ~1515-1560 us yang di
    DALAM deadband ESC biasa; window maju dijepit mulai netral+deadband
    supaya perintah kecil benar-benar menggerakkan roda.
    """
    if abs(speed_mps) < MIN_COMMAND_MPS:
        return ESC_NEUTRAL_US
    magnitude = abs(speed_mps) / MAX_SPEED_MPS
    span = (ESC_MAX_US - ESC_NEUTRAL_US) - ESC_DEADBAND_US
    offset = ESC_DEADBAND_US + int(magnitude * span + 0.5)
    return (ESC_NEUTRAL_US + offset if speed_mps > 0.0
            else ESC_NEUTRAL_US - offset)


def steer_pulse(steer_rad: float) -> int:
    """Cermin steerPulse(): pulse servo kemudi 1100..1900 us."""
    ratio = _constrain(steer_rad, -MAX_STEER_RAD, MAX_STEER_RAD) / MAX_STEER_RAD
    exact = STEER_CENTER_US + ratio * (STEER_MAX_US - STEER_CENTER_US)
    pulse = int(exact + 0.5) if exact >= 0.0 else int(exact - 0.5)
    return int(_constrain(pulse, STEER_MIN_US, STEER_MAX_US))


def _fixed(value: float, decimals: int = 3) -> str:
    """Cermin appendFixed(): tanpa printf %f, bulat setengah ke atas.

    Dipakai hanya untuk menyusun baris status #v1,... sintetis.
    """
    negative = value < 0.0
    magnitude = -value if negative else value
    scale = 10 ** decimals
    rounded = int(magnitude * scale + 0.5)
    sign = "-" if negative and rounded != 0 else ""
    whole, fraction = divmod(rounded, scale)
    if decimals <= 0:
        return f"{sign}{whole}"
    return f"{sign}{whole}.{fraction:0{decimals}d}"


# ---------------------------------------------------------------------------
# Virtual MCU
# ---------------------------------------------------------------------------
class VirtualMcu:
    """Pengganti McuLink untuk bench: menjalankan pemetaan output firmware
    di Python dan merekam hasilnya. Tidak ada serial, tidak ada hardware.

    `dry_run` di sini murni penanda untuk runner (mirror McuLink):
    False = jalur status balik di runner ikut diuji lewat baris #v1,...
    sintetis. Link sungguhan TIDAK pernah dibuka dalam kelas ini.

    Cermin perilaku firmware yang relevan buat bench:
    - mode skid4: mixing 4 roda + clamp bersama (lihat skid_wheels)
    - mode esc1: arming hold 2 s (netral) seperti firmware
    - status 10 Hz + stale kalau tidak ada setpoint segar > 100 ms
      (watchdog) — di bench ini berarti "persepsi terlalu lambat"
    """

    def __init__(self, driver: str = "skid4_l298n", *, dry_run: bool = False,
                 estop: bool = False, arm_delay_s: float = ESC_ARM_S) -> None:
        if driver not in FIRMWARE_MODES:
            raise ValueError(
                f"driver tidak dikenal: {driver} "
                f"(pilih {sorted(FIRMWARE_MODES)})")
        self.driver = driver
        self.mode = FIRMWARE_MODES[driver]
        self.dry_run = dry_run
        self.alive = True
        self.serial = None          # selalu None: tidak ada hardware
        self.estop = estop
        self._arm_delay_s = arm_delay_s
        self._boot_at = time.monotonic()
        self._last_command_at: float | None = None
        self._last_status_at: float | None = None
        self._rx_ok = 0
        self._applied_speed = 0.0
        self._applied_steer = 0.0
        # statistik bench
        self.commands: list[Command] = []
        self.last_outputs: deque = deque(maxlen=20)
        self.zero_commands = 0
        self.neutral_holds = 0
        self.clamp_events = 0
        self.max_abs_speed = 0.0
        self.max_abs_steer = 0.0
        self.max_pwm = 0
        self.dir_changes = 0
        self.esc_min_us: int | None = None
        self.esc_max_us: int | None = None
        self.steer_min_us: int | None = None
        self.steer_max_us: int | None = None
        self.stale_status_lines = 0
        self._prev_dir: dict[str, int] = {}

    # -- interface yang dipakai runner (mirror McuLink) ----------------

    def send(self, command: Command) -> None:
        """Terima setpoint, jalankan pemetaan output firmware, rekam."""
        self.commands.append(command)
        self._rx_ok += 1
        self._last_command_at = time.monotonic()
        if command.speed_mps == 0.0 and command.steering_rad == 0.0:
            self.zero_commands += 1
        self.max_abs_speed = max(self.max_abs_speed, abs(command.speed_mps))
        self.max_abs_steer = max(self.max_abs_steer, abs(command.steering_rad))

        speed = _constrain(command.speed_mps, -MAX_SPEED_MPS, MAX_SPEED_MPS)
        steer = _constrain(command.steering_rad, -MAX_STEER_RAD, MAX_STEER_RAD)
        now = time.monotonic()
        arming = (self.mode == "esc1"
                  and now - self._boot_at < self._arm_delay_s)

        output: dict = {"t": now, "speed": speed, "steer": steer}
        if self.estop:
            # E-stop aktif: firmware memotong semua output.
            output["estop"] = True
            output["wheels"] = {w: hbridge(0.0) for w in
                                ("FL", "FR", "RL", "RR")}
            output["esc_us"] = ESC_NEUTRAL_US
            output["steer_us"] = STEER_CENTER_US
        elif arming:
            # applyNeutral(): ESC butuh netral beberapa detik setelah boot.
            self.neutral_holds += 1
            output["arming"] = True
            output["esc_us"] = ESC_NEUTRAL_US
            output["steer_us"] = STEER_CENTER_US
        elif self.mode == "skid4":
            wheels = skid_wheels(speed, steer)
            if wheels.clamped:
                self.clamp_events += 1
            output["clamped"] = wheels.clamped
            output["wheels"] = {
                "FL": hbridge(wheels.left), "RL": hbridge(wheels.left),
                "FR": hbridge(wheels.right), "RR": hbridge(wheels.right),
            }
            output["wheel_speed_mps"] = {"left": wheels.left,
                                         "right": wheels.right}
            self._applied_speed = speed
            self._applied_steer = steer
        elif self.mode == "esc1":
            output["esc_us"] = esc_pulse(speed)
            output["steer_us"] = steer_pulse(steer)
            self._applied_speed = speed
            self._applied_steer = steer
        else:  # hb1
            output["motor"] = hbridge(speed)
            output["steer_us"] = steer_pulse(steer)
            self._applied_speed = speed
            self._applied_steer = steer

        self._track_output(output)
        self.last_outputs.append(output)

    def _track_output(self, output: dict) -> None:
        """Update statistik dari satu output (PWM maks, ganti arah, rentang
        pulse). Dipakai summary bench."""
        for side, (dir_pin, pwm) in output.get("wheels", {}).items():
            self.max_pwm = max(self.max_pwm, pwm)
            previous = self._prev_dir.get(side)
            if previous is not None and previous != dir_pin:
                self.dir_changes += 1
            self._prev_dir[side] = dir_pin
        if "motor" in output:
            dir_pin, pwm = output["motor"]
            self.max_pwm = max(self.max_pwm, pwm)
            previous = self._prev_dir.get("motor")
            if previous is not None and previous != dir_pin:
                self.dir_changes += 1
            self._prev_dir["motor"] = dir_pin
        if "esc_us" in output and not output.get("arming"):
            pulse = output["esc_us"]
            self.esc_min_us = pulse if self.esc_min_us is None \
                else min(self.esc_min_us, pulse)
            self.esc_max_us = pulse if self.esc_max_us is None \
                else max(self.esc_max_us, pulse)
        if "steer_us" in output and not output.get("arming"):
            pulse = output["steer_us"]
            self.steer_min_us = pulse if self.steer_min_us is None \
                else min(self.steer_min_us, pulse)
            self.steer_max_us = pulse if self.steer_max_us is None \
                else max(self.steer_max_us, pulse)

    def read_status(self) -> McuStatus | None:
        """Baris status sintetis 10 Hz (mirror McuLink.read_status).

        Baris #v1,... disusun dengan format yang SAMA seperti firmware
        lalu dilewatkan ke parse_status_line() sungguhan — jadi jalur
        parser + event runner ikut teruji, bukan di-bypass.
        """
        if self.dry_run:
            return None
        now = time.monotonic()
        if (self._last_status_at is not None
                and now - self._last_status_at < STATUS_PERIOD_S):
            return None
        self._last_status_at = now
        stale = (self._last_command_at is None
                 or now - self._last_command_at > WATCHDOG_S)
        if stale:
            self.stale_status_lines += 1
        uptime_ms = int((now - self._boot_at) * 1000.0)
        line = (f"#v1,{self.mode},{1 if self.estop else 0},"
                f"{1 if stale else 0},{_fixed(self._applied_speed)},"
                f"{_fixed(self._applied_steer)},{self._rx_ok},{uptime_ms}")
        return parse_status_line(line)

    def close(self) -> None:
        """Mirror McuLink.close(): perintah nol terakhir, lalu mati."""
        if self.alive:
            self.send(Command(reason="shutdown"))
        self.alive = False

    def __enter__(self) -> "VirtualMcu":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # -- ringkasan buat laporan bench ---------------------------------

    @property
    def last_output(self) -> dict:
        return self.last_outputs[-1] if self.last_outputs else {}

    def summary(self) -> dict:
        """Statistik buat bench_summary.json + cetakan akhir."""
        out: dict = {
            "driver": self.driver,
            "mode": self.mode,
            "commands": len(self.commands),
            "zero_commands": self.zero_commands,
            "zero_fraction": (self.zero_commands / len(self.commands)
                              if self.commands else 0.0),
            "max_abs_speed_mps": self.max_abs_speed,
            "max_abs_steer_rad": self.max_abs_steer,
            "max_pwm": self.max_pwm,
            "dir_changes": self.dir_changes,
            "neutral_holds": self.neutral_holds,
            "stale_status_lines": self.stale_status_lines,
        }
        if self.mode == "skid4":
            out["clamp_events"] = self.clamp_events
        if self.esc_min_us is not None:
            out["esc_us_range"] = [self.esc_min_us, self.esc_max_us]
        if self.steer_min_us is not None:
            out["steer_us_range"] = [self.steer_min_us, self.steer_max_us]
        return out
