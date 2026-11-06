"""Tes VirtualMcu: cermin pemetaan output firmware body_controller.ino.

Nilai-nilai di sini dikunci ke angka yang SUDAH diverifikasi di tes host
firmware (program/body-mcu/test/run_tests.sh). Kalau firmware berubah,
tes host yang benar — cermin Python ini (dan tes ini) yang harus ikut
diperbarui.
"""
import importlib.util
from pathlib import Path

import pytest

_spec = importlib.util.spec_from_file_location(
    "virtual_mcu",
    Path(__file__).resolve().parent.parent / "tools" / "virtual_mcu.py")
vm = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(vm)

from mizuro.types import Command, McuStatus  # noqa: E402


def _cmd(speed=0.0, steer=0.0, reason=""):
    return Command(speed, steer, reason)


# --- cermin mixing skid4 (angka dari tes host firmware) ----------------------

def test_skid_wheels_matches_firmware_host_test():
    left, right, clamped = vm.skid_wheels(0.4, -0.1)
    assert vm.hbridge(left)[1] == 134
    assert vm.hbridge(right)[1] == 121
    assert clamped is False


def test_skid_clamp_scales_both_sides():
    """Steer penuh maju: puncak > batas -> dua sisi diskalakan BERSAMA."""
    left, right, clamped = vm.skid_wheels(0.8, 0.55)
    assert clamped is True
    assert vm.hbridge(left)[1] == 193
    assert vm.hbridge(right)[1] == 255


def test_skid_pivot_in_place():
    left, right, _ = vm.skid_wheels(0.0, 0.55)
    assert vm.hbridge(left) == (0, 35)      # kiri mundur (DIR kebalikan)
    assert vm.hbridge(right) == (1, 35)     # kanan maju


# --- cermin pemetaan ESC / servo ----------------------------------------------

def test_esc_pulse_matches_firmware_host_test():
    assert vm.esc_pulse(0.4) == 1770
    assert vm.esc_pulse(0.05) == 1569       # gir rendah: di atas deadband
    assert vm.esc_pulse(0.01) == 1500       # di bawah MIN_COMMAND_MPS
    assert vm.esc_pulse(-0.4) == 1230


def test_steer_pulse_matches_firmware_host_test():
    assert vm.steer_pulse(0.1) == 1573
    assert vm.steer_pulse(0.55) == 1900
    assert vm.steer_pulse(-0.55) == 1100
    assert vm.steer_pulse(0.0) == 1500


def test_fixed_mirrors_appendfixed():
    assert vm._fixed(0.0) == "0.000"
    assert vm._fixed(-0.0004) == "0.000"    # tidak pernah cetak "-0.000"
    assert vm._fixed(-0.1234) == "-0.123"
    assert vm._fixed(1.5) == "1.500"


# --- VirtualMcu: interface + rekaman ------------------------------------------

def test_virtual_mcu_records_wheel_pwm():
    mcu = vm.VirtualMcu("skid4_l298n")
    mcu.send(_cmd(0.4, -0.1))
    wheels = mcu.last_output["wheels"]
    assert wheels["FL"] == (1, 134)
    assert wheels["FR"] == (1, 121)
    assert wheels["RL"] == (1, 134)
    assert wheels["RR"] == (1, 121)


def test_virtual_mcu_rejects_unknown_driver():
    with pytest.raises(ValueError, match="driver tidak dikenal"):
        vm.VirtualMcu("sihir")


def test_virtual_mcu_summary_stats():
    mcu = vm.VirtualMcu("skid4_l298n")
    for _ in range(5):
        mcu.send(_cmd(0.3, 0.0))
    mcu.send(_cmd(0.0, 0.0))
    mcu.send(_cmd(0.8, 0.55))               # steer penuh -> clamp
    s = mcu.summary()
    assert s["mode"] == "skid4"
    assert s["commands"] == 7
    assert s["zero_commands"] == 1
    assert s["clamp_events"] == 1
    assert s["max_abs_speed_mps"] == 0.8
    assert s["max_pwm"] == 255


def test_virtual_mcu_dir_changes_counted():
    mcu = vm.VirtualMcu("skid4_l298n")
    mcu.send(_cmd(0.3, 0.0))                 # semua maju
    mcu.send(_cmd(-0.3, 0.0))                # semua mundur: 4 sisi ganti
    assert mcu.summary()["dir_changes"] == 4


def test_virtual_mcu_estop_cuts_output():
    mcu = vm.VirtualMcu("skid4_l298n", estop=True)
    mcu.send(_cmd(0.5, 0.2))
    out = mcu.last_output
    assert out["estop"] is True
    assert all(pwm == 0 for _, pwm in out["wheels"].values())


def test_virtual_mcu_close_sends_zero():
    mcu = vm.VirtualMcu("skid4_l298n")
    mcu.send(_cmd(0.5, 0.0))
    mcu.close()
    assert mcu.alive is False
    assert mcu.commands[-1].reason == "shutdown"
    assert mcu.commands[-1].speed_mps == 0.0


# --- VirtualMcu: status balik (lewat parser sungguhan) ------------------------

def test_virtual_mcu_status_round_trip():
    """Baris #v1 sintetis harus lolos parse_status_line sungguhan."""
    mcu = vm.VirtualMcu("skid4_l298n", dry_run=False)
    mcu.send(_cmd(0.25, -0.1))
    status = mcu.read_status()
    assert isinstance(status, McuStatus)
    assert status.mode == "skid4"
    assert status.rx_ok == 1
    assert status.speed_out_mps == pytest.approx(0.25, abs=1e-3)
    assert status.steer_out_rad == pytest.approx(-0.1, abs=1e-3)


def test_virtual_mcu_status_rate_limited_10hz():
    mcu = vm.VirtualMcu("skid4_l298n", dry_run=False)
    mcu.send(_cmd(0.1, 0.0))
    assert mcu.read_status() is not None
    assert mcu.read_status() is None         # < 100 ms: belum waktunya


def test_virtual_mcu_dry_run_no_status():
    mcu = vm.VirtualMcu("skid4_l298n", dry_run=True)
    mcu.send(_cmd(0.1, 0.0))
    assert mcu.read_status() is None


def test_virtual_mcu_stale_when_no_fresh_command():
    """Mirror watchdog: tidak ada setpoint segar > 100 ms -> stale."""
    mcu = vm.VirtualMcu("skid4_l298n", dry_run=False)
    mcu.send(_cmd(0.2, 0.0))
    assert mcu.read_status().stale is False
    # pura-pakan loop mandek 200 ms (di hardware: persepsi terlalu lambat)
    mcu._last_status_at = None               # bypass rate-limit 10 Hz
    mcu._last_command_at -= 0.2
    assert mcu.read_status().stale is True


# --- mode esc1 / hb1 -----------------------------------------------------------

def test_virtual_mcu_esc1_arming_hold():
    mcu = vm.VirtualMcu("external_esc", dry_run=False, arm_delay_s=60.0)
    mcu.send(_cmd(0.5, 0.2))
    out = mcu.last_output
    assert out["arming"] is True
    assert out["esc_us"] == 1500             # netral selama arming
    assert out["steer_us"] == 1500
    assert mcu.summary()["neutral_holds"] == 1


def test_virtual_mcu_esc1_after_arming():
    mcu = vm.VirtualMcu("external_esc", dry_run=False, arm_delay_s=0.0)
    mcu.send(_cmd(0.4, 0.1))
    out = mcu.last_output
    assert "arming" not in out
    assert out["esc_us"] == 1770
    assert out["steer_us"] == 1573


def test_virtual_mcu_hb1_mode():
    mcu = vm.VirtualMcu("mcu_hbridge", dry_run=False, arm_delay_s=0.0)
    mcu.send(_cmd(0.4, 0.1))
    out = mcu.last_output
    assert out["motor"] == (1, 128)          # 0.4/0.8*255+0.5
    assert out["steer_us"] == 1573
