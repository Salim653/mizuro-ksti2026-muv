"""Tes tautan serial: dry-run no-op, format kiriman, gagal tulis, close."""
import pytest

from mizuro.mcu import McuLink
from mizuro.types import Command


class FakeSerial:
    """Mencatat semua yang ditulis; bisa diset gagal lewat subclass."""

    def __init__(self):
        self.writes = []
        self.closed = False

    def write(self, data):
        self.writes.append(data)
        return len(data)

    def close(self):
        self.closed = True


class BrokenSerial(FakeSerial):
    def write(self, data):
        raise OSError("serial putus")


def test_dry_run_never_touches_serial():
    mcu = McuLink("/dev/ttyTIDAKADA", dry_run=True)
    mcu.send(Command(0.5, 0.1))
    assert mcu.alive is True
    assert mcu.serial is None
    mcu.close()  # tidak melempar walau port tidak pernah dibuka


def test_send_writes_protocol_line():
    fake = FakeSerial()
    mcu = McuLink("/dev/fake", dry_run=False, opener=lambda dev, baud: fake)
    mcu.send(Command(0.4, -0.1))
    assert fake.writes == [b"0.4000,-0.1000\n"]
    assert mcu.alive is True


def test_write_failure_marks_link_dead_and_stops_writing():
    fake = BrokenSerial()
    mcu = McuLink("/dev/fake", dry_run=False, opener=lambda dev, baud: fake)
    mcu.send(Command(0.4, 0.0))  # tidak melempar ke pemanggil
    assert mcu.alive is False
    mcu.send(Command(0.4, 0.0))  # setelah mati: no-op
    assert fake.writes == []


def test_close_sends_zero_then_closes():
    fake = FakeSerial()
    mcu = McuLink("/dev/fake", dry_run=False, opener=lambda dev, baud: fake)
    mcu.close()
    assert fake.writes == [b"0.0000,0.0000\n"]
    assert fake.closed is True
    assert mcu.serial is None
    mcu.close()  # idempoten


def test_close_after_failure_still_closes_port():
    fake = BrokenSerial()
    mcu = McuLink("/dev/fake", dry_run=False, opener=lambda dev, baud: fake)
    mcu.send(Command())  # OSError -> alive False
    mcu.close()  # tetap menutup tanpa melempar
    assert fake.closed is True
