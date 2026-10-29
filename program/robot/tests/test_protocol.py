"""Tes protokol serial: format persis dan batas yang ditegakkan."""
import pytest

from mizuro.protocol import MAX_SPEED_MPS, MAX_STEER_RAD, format_command
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
