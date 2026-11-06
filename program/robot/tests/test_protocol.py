"""Tes protokol serial: format persis dan batas yang ditegakkan."""
import pytest

from mizuro.protocol import (
    FIRMWARE_MODES,
    FIRMWARE_MODE_NAMES,
    MAX_SPEED_MPS,
    MAX_STEER_RAD,
    format_command,
    format_mode,
    parse_status_line,
)
from mizuro.types import Command


def test_format_zero():
    assert format_command(Command()) == b"0.0000,0.0000\n"


def test_format_at_limits():
    assert format_command(Command(MAX_SPEED_MPS, -MAX_STEER_RAD)) == b"0.8000,-0.5500\n"


def test_format_negative_and_four_decimals():
    assert format_command(Command(-0.123456, 0.05)) == b"-0.1235,0.0500\n"


def test_negative_zero_normalized_on_wire():
    # -0.00004 dibulatkan jadi "-0.0000" oleh format biasa; protokol
    # menormalkannya supaya wire dan log tidak pernah memuat negatif-nol.
    assert format_command(Command(-0.00004, 0.0)) == b"0.0000,0.0000\n"
    assert format_command(Command(0.0, -0.00004)) == b"0.0000,0.0000\n"


@pytest.mark.parametrize("cmd", [
    Command(0.81, 0.0),
    Command(-0.81, 0.0),
    Command(0.0, 0.56),
    Command(0.0, -0.56),
])
def test_out_of_range_raises(cmd):
    with pytest.raises(ValueError):
        format_command(cmd)


@pytest.mark.parametrize("cmd", [
    Command(float("nan"), 0.0),
    Command(float("inf"), 0.0),
    Command(0.0, float("nan")),
    Command(0.0, float("-inf")),
])
def test_nan_and_inf_raise(cmd):
    with pytest.raises(ValueError):
        format_command(cmd)


# --- perintah mode (Jetson -> MCU) -----------------------------------------

def test_firmware_modes_cover_all_drivers():
    """Tiga motor_driver di config punya pasangan mode firmware."""
    assert set(FIRMWARE_MODES) == {"external_esc", "mcu_hbridge", "skid4_l298n"}
    assert FIRMWARE_MODES["external_esc"] == "esc1"
    assert FIRMWARE_MODES["mcu_hbridge"] == "hb1"
    assert FIRMWARE_MODES["skid4_l298n"] == "skid4"
    assert FIRMWARE_MODE_NAMES == {"esc1", "hb1", "skid4"}


@pytest.mark.parametrize("driver,mode", [
    ("external_esc", "esc1"),
    ("mcu_hbridge", "hb1"),
    ("skid4_l298n", "skid4"),
])
def test_format_mode(driver, mode):
    assert format_mode(driver) == f"#M,{mode}\n".encode("ascii")


def test_format_mode_rejects_unknown_driver():
    # Driver tidak dikenal harus gagal DI SINI, bukan jadi mode salah
    # yang dikirim ke roda.
    with pytest.raises(ValueError, match="motor_driver"):
        format_mode("sihir")
    with pytest.raises(ValueError, match="motor_driver"):
        format_mode("")


# --- baris status (MCU -> Jetson) -------------------------------------------

def test_parse_status_line_valid():
    status = parse_status_line("#v1,skid4,0,1,0.400,-0.100,7,12345\n")
    assert status is not None
    assert status.mode == "skid4"
    assert status.estop is False
    assert status.stale is True
    assert status.speed_out_mps == 0.4
    assert status.steer_out_rad == -0.1
    assert status.rx_ok == 7
    assert status.uptime_ms == 12345


def test_parse_status_line_flags_true():
    status = parse_status_line("#v1,esc1,1,0,0.000,0.000,0,0")
    assert status.estop is True
    assert status.stale is False


@pytest.mark.parametrize("line", [
    "",                                   # kosong
    "0.4000,-0.1000\n",                   # setpoint yang memantul
    "#v1,skid4,0,0,0.400,-0.100,7",       # field kurang
    "#v1,skid4,0,0,0.400,-0.100,7,1,2",   # field lebih
    "#v1,nope,0,0,0.000,0.000,0,0",       # mode tidak dikenal
    "#v1,skid4,2,0,0.000,0.000,0,0",      # flag bukan 0/1
    "#v1,skid4,0,0,0.900,0.000,0,0",      # speed di luar batas
    "#v1,skid4,0,0,0.000,0.600,0,0",      # steer di luar batas
    "#v1,skid4,0,0,abc,0.000,0,0",        # speed bukan angka
    "#v1,skid4,0,0,0.000,0.000,-1,0",     # rx_ok negatif
    "#v1,skid4,0,0,0.000,0.000,0,-5",     # uptime negatif
    "#v2,skid4,0,0,0.000,0.000,0,0",      # versi format lain
    "sampah#v1,skid4,0,0,0.000,0.000,0,0",  # awalan salah
])
def test_parse_status_line_rejects(line):
    """Baris rusak/sampah diabaikan (None), bukan melempar exception."""
    assert parse_status_line(line) is None


def test_parse_status_line_tolerates_whitespace():
    status = parse_status_line("  #v1,hb1,0,0,0.100,0.050,3,99  \n")
    assert status is not None
    assert status.mode == "hb1"
    assert status.rx_ok == 3
