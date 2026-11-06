"""Tes tautan serial: dry-run no-op, format kiriman, gagal tulis, close."""
import pytest

from mizuro.mcu import McuLink
from mizuro.types import Command


class FakeSerial:
    """Mencatat semua yang ditulis; bisa diset gagal lewat subclass.

    `rx` meniru buffer masuk MCU -> Jetson (baris status #v1,...);
    `read(n)` mengonsumsi dari sana seperti pyserial.
    """

    def __init__(self):
        self.writes = []
        self.rx = b""
        self.closed = False

    def write(self, data):
        self.writes.append(data)
        return len(data)

    @property
    def in_waiting(self):
        return len(self.rx)

    def read(self, size):
        chunk, self.rx = self.rx[:size], self.rx[size:]
        return chunk

    def close(self):
        self.closed = True


class BrokenSerial(FakeSerial):
    def write(self, data):
        raise OSError("serial putus")


class BrokenReadSerial(FakeSerial):
    @property
    def in_waiting(self):
        raise OSError("port lepas saat baca")


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
    # Baris mode lebih dulu (dikirim saat port dibuka), lalu setpoint.
    assert fake.writes == [b"#M,esc1\n", b"0.4000,-0.1000\n"]
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
    assert fake.writes == [b"#M,esc1\n", b"0.0000,0.0000\n"]
    assert fake.closed is True
    assert mcu.serial is None
    mcu.close()  # idempoten


def test_close_after_failure_still_closes_port():
    fake = BrokenSerial()
    mcu = McuLink("/dev/fake", dry_run=False, opener=lambda dev, baud: fake)
    mcu.send(Command())  # OSError -> alive False
    mcu.close()  # tetap menutup tanpa melempar
    assert fake.closed is True


# --- perintah mode (Jetson -> MCU) -------------------------------------------

def test_mode_line_sent_when_port_opens():
    """Saat port dibuka, baris #M,<mode> dikirim SEBELUM setpoint apa pun —
    firmware perlu tahu tahap output mana yang dipakai."""
    fake = FakeSerial()
    McuLink("/dev/fake", dry_run=False, driver="skid4_l298n",
            opener=lambda dev, baud: fake)
    assert fake.writes == [b"#M,skid4\n"]


def test_mode_line_matches_driver():
    fake = FakeSerial()
    McuLink("/dev/fake", dry_run=False, driver="external_esc",
            opener=lambda dev, baud: fake)
    assert fake.writes == [b"#M,esc1\n"]


def test_invalid_driver_rejected_even_in_dry_run():
    # Config salah harus gagal di konstruktor, bukan jadi output salah
    # di roda — dry-run pun divalidasi.
    with pytest.raises(ValueError, match="motor_driver"):
        McuLink("/dev/ttyTIDAKADA", dry_run=True, driver="sihir")


def test_dry_run_sends_nothing():
    fake = FakeSerial()
    mcu = McuLink("/dev/fake", dry_run=True, driver="skid4_l298n",
                  opener=lambda dev, baud: fake)
    assert fake.writes == []
    assert mcu.serial is None


# --- pembacaan status (MCU -> Jetson) -----------------------------------------

def test_read_status_dry_run_returns_none():
    mcu = McuLink("/dev/fake", dry_run=True, driver="skid4_l298n")
    assert mcu.read_status() is None


def test_read_status_parses_line():
    fake = FakeSerial()
    mcu = McuLink("/dev/fake", dry_run=False, driver="skid4_l298n",
                  opener=lambda dev, baud: fake)
    fake.rx = b"#v1,skid4,0,0,0.400,-0.100,7,1234\n"
    status = mcu.read_status()
    assert status is not None
    assert status.mode == "skid4"
    assert status.speed_out_mps == 0.4
    assert status.rx_ok == 7
    assert status.uptime_ms == 1234
    assert fake.rx == b""          # buffer masuk dikonsumsi


def test_read_status_returns_none_without_newline():
    fake = FakeSerial()
    mcu = McuLink("/dev/fake", dry_run=False, driver="skid4_l298n",
                  opener=lambda dev, baud: fake)
    fake.rx = b"#v1,skid4,0,0,0.400"   # baris belum lengkap
    assert mcu.read_status() is None
    # Ditahan di buffer internal McuLink (bukan dibuang, bukan error).
    assert mcu._rx_buffer == b"#v1,skid4,0,0,0.400"
    fake.rx += b",-0.100,7,1234\n"
    status = mcu.read_status()
    assert status is not None
    assert status.steer_out_rad == -0.1


def test_read_status_ignores_junk_keeps_valid():
    fake = FakeSerial()
    mcu = McuLink("/dev/fake", dry_run=False, driver="skid4_l298n",
                  opener=lambda dev, baud: fake)
    fake.rx = b"sampah\n#v1,skid4,0,0,0.100,0.000,1,50\n#v1,nope,0,0,0,0,0,0\n"
    status = mcu.read_status()
    assert status is not None
    assert status.speed_out_mps == 0.1
    assert status.uptime_ms == 50


def test_read_status_returns_latest_of_multiple_lines():
    fake = FakeSerial()
    mcu = McuLink("/dev/fake", dry_run=False, driver="skid4_l298n",
                  opener=lambda dev, baud: fake)
    fake.rx = (b"#v1,skid4,0,0,0.100,0.000,1,50\n"
               b"#v1,skid4,0,0,0.200,0.000,2,100\n")
    status = mcu.read_status()
    assert status.speed_out_mps == 0.2
    assert status.rx_ok == 2


def test_read_status_limits_junk_buffer():
    """Sampah tanpa newline tidak boleh menumpuk tanpa batas di RAM."""
    fake = FakeSerial()
    mcu = McuLink("/dev/fake", dry_run=False, driver="skid4_l298n",
                  opener=lambda dev, baud: fake)
    fake.rx = b"X" * 5000
    mcu.read_status()
    assert len(mcu._rx_buffer) <= 256


def test_read_status_read_error_marks_link_dead():
    fake = BrokenReadSerial()
    mcu = McuLink("/dev/fake", dry_run=False, driver="skid4_l298n",
                  opener=lambda dev, baud: fake)
    assert mcu.read_status() is None
    assert mcu.alive is False
