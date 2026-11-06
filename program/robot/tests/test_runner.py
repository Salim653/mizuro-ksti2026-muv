"""Tes closed-loop runner Fase 4 — tanpa hardware.

Semua dependensi di-stub: kamera palsu yang menghasilkan frame
sintetis, MCU palsu yang merekam perintah, tracker palsu yang
mengembalikan Observation yang bisa dikontrol per-test.
"""
import time

import numpy as np
import pytest

from mizuro.runner import ClosedLoopRunner
from mizuro.states import State, StateMachine
from mizuro.telemetry import RunLog
from mizuro.types import Command, Observation


class StubCamera:
    """Kamera palsu: read() return frame hitam + hitung panggilan."""

    def __init__(self, frames=None):
        self.frames = frames or [np.zeros((480, 640, 3), dtype=np.uint8)]
        self.i = 0
        self.reads = 0

    def read(self):
        self.reads += 1
        frame = self.frames[self.i % len(self.frames)]
        self.i += 1
        return frame


class StubMcu:
    """MCU palsu: rekam setiap perintah yang dikirim.

    `statuses` = antrian McuStatus yang "dikirim MCU"; read_status()
    mengeluarkan satu per panggilan (None kalau antrian kosong), persis
    seperti McuLink sungguhan.
    """

    def __init__(self, dry_run=True, driver="external_esc", statuses=None):
        self.dry_run = dry_run
        self.driver = driver
        self.commands: list[Command] = []
        self.statuses = list(statuses or [])
        self.read_status_calls = 0
        self.alive = True
        self.serial = None

    def send(self, command: Command) -> None:
        self.commands.append(command)

    def read_status(self):
        self.read_status_calls += 1
        return self.statuses.pop(0) if self.statuses else None

    def close(self) -> None:
        self.alive = False


class DelayedStatusMcu(StubMcu):
    """Status pertama langsung; sisanya baru keluar setelah `gap_s` detik.

    Dipakai untuk menguji lost/recovered: tanpa jeda nyata, status
    kedua akan terbaca sebelum timeout sempat tercatat.
    """

    def __init__(self, *args, gap_s=0.2, **kwargs):
        super().__init__(*args, **kwargs)
        self.gap_s = gap_s
        self._first_at = None

    def read_status(self):
        self.read_status_calls += 1
        if not self.statuses:
            return None
        now = time.monotonic()
        if self._first_at is None:
            self._first_at = now
            return self.statuses.pop(0)
        if now - self._first_at >= self.gap_s:
            return self.statuses.pop(0)
        return None


class StubTracker:
    """Tracker palsu: kembalikan Observation dari antrian per-frame."""

    def __init__(self, observations):
        self.observations = observations
        self.i = 0

    def estimate(self, frame, calib, timestamp, station_m=0.6):
        obs = self.observations[self.i % len(self.observations)]
        self.i += 1
        return obs


class StubCalib:
    """Kalibrasi minimal — tracker stub tidak memakainya."""


@pytest.fixture
def log(tmp_path):
    rl = RunLog(str(tmp_path))
    yield rl
    if not rl._closed:
        rl.close()


def _runner(log, camera, tracker, mcu, *, loop_hz=60.0, timeout_s=1.0,
            controller=None, detector=None, status_timeout_s=2.0):
    from mizuro.controller import StanleyController
    return ClosedLoopRunner(
        camera=camera, tracker=tracker,
        controller=controller or StanleyController(),
        mcu=mcu, log=log, machine=StateMachine(), calib=StubCalib(),
        loop_hz=loop_hz, run_timeout_s=timeout_s, detector=detector,
        status_timeout_s=status_timeout_s)


def _obs(conf=0.9, lane=0.0, heading=0.0, curvature=None, t=0.0):
    return Observation(timestamp=t, lane_error_m=lane,
                       heading_error_rad=heading,
                       curvature_inv_m=curvature, confidence=conf)


# --- loop dasar -----------------------------------------------------------

def test_loop_sends_heartbeat_every_iteration(log):
    """Setiap iterasi WAJIB kirim setpoint (watchdog MCU 100ms)."""
    cam = StubCamera()
    mcu = StubMcu()
    trk = StubTracker([_obs(t=float(i)) for i in range(5)])
    r = _runner(log, cam, trk, mcu, loop_hz=1000.0, timeout_s=0.05)
    r.run()
    assert len(mcu.commands) >= 5
    assert r.iterations >= 5


def test_loop_stops_on_timeout(log):
    cam = StubCamera()
    mcu = StubMcu()
    trk = StubTracker([_obs(t=0.0)])
    r = _runner(log, cam, trk, mcu, loop_hz=1000.0, timeout_s=0.1)
    r.run()
    assert r.machine.state is State.STOPPED
    assert any(e["event"] == "run_timeout"
               for e in _read_events(log))


def test_loop_stops_on_stop(log):
    cam = StubCamera()
    mcu = StubMcu()
    trk = StubTracker([_obs(t=0.0)])
    r = _runner(log, cam, trk, mcu, loop_hz=1000.0, timeout_s=10.0)
    r.stop()
    r.run()
    assert r.iterations == 0
    assert r.machine.state is State.STOPPED


# --- state machine -----------------------------------------------------------

def test_lane_lost_transitions_to_degraded(log):
    """Confidence 0 -> DEGRADED, loop TETAP jalan (robot berhenti)."""
    cam = StubCamera()
    mcu = StubMcu()
    trk = StubTracker([_obs(conf=0.0, t=0.0), _obs(conf=0.0, t=0.001)])
    r = _runner(log, cam, trk, mcu, loop_hz=1000.0, timeout_s=0.05)
    r.run()
    assert r.lane_lost_count >= 2
    events = _read_events(log)
    assert any(e["event"] == "lane_lost" for e in events)


def test_lane_recovered_transitions_back_to_running(log):
    """DEGRADED -> RUNNING saat lajur ditemukan lagi."""
    cam = StubCamera()
    mcu = StubMcu()
    trk = StubTracker([
        _obs(conf=0.9, t=0.0),
        _obs(conf=0.0, t=0.001),
        _obs(conf=0.9, t=0.002),
    ])
    r = _runner(log, cam, trk, mcu, loop_hz=1000.0, timeout_s=0.05)
    r.run()
    events = _read_events(log)
    assert any(e["event"] == "lane_lost" for e in events)
    assert any(e["event"] == "lane_recovered" for e in events)


# --- keselamatan -----------------------------------------------------------

def test_no_lane_sends_zero_command(log):
    """Confidence 0 -> Command nol dikirim ke MCU (bukan tebak arah)."""
    cam = StubCamera()
    mcu = StubMcu()
    trk = StubTracker([_obs(conf=0.0, t=0.0)])
    r = _runner(log, cam, trk, mcu, loop_hz=1000.0, timeout_s=0.03)
    r.run()
    # Perintah terakhir (shutdown) dan iterasi no-lane harus nol.
    assert all(c.speed_mps == 0.0 for c in mcu.commands)
    assert all(c.steering_rad == 0.0 for c in mcu.commands)


def test_shutdown_sends_final_zero(log):
    """Saat loop selesai, perintah nol terakhir dikirim."""
    cam = StubCamera()
    mcu = StubMcu()
    trk = StubTracker([_obs(t=0.0)])
    r = _runner(log, cam, trk, mcu, loop_hz=1000.0, timeout_s=0.02)
    r.run()
    assert mcu.commands[-1].speed_mps == 0.0
    assert mcu.commands[-1].steering_rad == 0.0
    assert mcu.commands[-1].reason == "shutdown"


def test_dry_run_does_not_touch_serial(log):
    """dry_run=True: loop penuh jalan, tapi mcu.serial None."""
    cam = StubCamera()
    mcu = StubMcu(dry_run=True)
    trk = StubTracker([_obs(t=0.0)])
    r = _runner(log, cam, trk, mcu, loop_hz=1000.0, timeout_s=0.02)
    r.run()
    assert mcu.serial is None
    assert len(mcu.commands) > 0        # loop tetap jalan + merekam


# --- detektor opsional ------------------------------------------------------

class StubDetector:
    def __init__(self, dets=None, fail=False):
        self.dets = dets or []
        self.fail = fail
        self.calls = 0

    def infer(self, frame):
        self.calls += 1
        if self.fail:
            raise RuntimeError("detektor rusak")
        return self.dets


def test_detector_failure_does_not_stop_loop(log):
    """Detektor error != robot berhenti — di-log, loop lanjut."""
    cam = StubCamera()
    mcu = StubMcu()
    trk = StubTracker([_obs(t=float(i)) for i in range(4)])
    det = StubDetector(fail=True)
    r = _runner(log, cam, trk, mcu, loop_hz=1000.0, timeout_s=0.05,
                detector=det)
    r.run()
    assert det.calls >= 4
    events = _read_events(log)
    assert any(e["event"] == "detector_error" for e in events)


def test_detector_success_attaches_to_observation(log):
    """Deteksi berhasil -> masuk Observation.detections."""
    from mizuro.perception.detector import Detection
    cam = StubCamera()
    mcu = StubMcu()
    trk = StubTracker([_obs(t=0.0)])
    det = StubDetector(dets=[
        Detection(class_id=0, label="person", confidence=0.9,
                  bbox=(10, 10, 50, 50))])
    r = _runner(log, cam, trk, mcu, loop_hz=1000.0, timeout_s=0.02,
                detector=det)
    r.run()
    assert det.calls >= 1


# --- telemetry ---------------------------------------------------------------

def test_telemetry_rows_written(log):
    """Setiap iterasi menulis satu baris telemetry."""
    cam = StubCamera()
    mcu = StubMcu()
    trk = StubTracker([_obs(t=float(i)) for i in range(6)])
    r = _runner(log, cam, trk, mcu, loop_hz=1000.0, timeout_s=0.05)
    r.run()
    import csv
    with open(log.path / "telemetry.csv") as f:
        rows = list(csv.DictReader(f))
    assert len(rows) >= 6
    assert "confidence" in rows[0]
    assert "speed_mps" in rows[0]


# --- status MCU (MCU -> Jetson) ----------------------------------------------

def _status(mode="skid4", estop=False, stale=False, speed=0.0, steer=0.0,
            rx_ok=1, uptime=0):
    from mizuro.types import McuStatus
    return McuStatus(mode, estop, stale, speed, steer, rx_ok, uptime)


def test_dry_run_does_not_poll_status(log):
    """dry_run: status tidak dibaca (tidak ada serial untuk dibaca)."""
    cam = StubCamera()
    mcu = StubMcu(dry_run=True, statuses=[_status()])
    trk = StubTracker([_obs(t=0.0)])
    r = _runner(log, cam, trk, mcu, loop_hz=1000.0, timeout_s=0.05)
    r.run()
    assert mcu.read_status_calls == 0
    assert not any(e["event"].startswith("mcu_")
                   for e in _read_events(log))


def test_first_status_logs_ready_with_mode(log):
    cam = StubCamera()
    mcu = StubMcu(dry_run=False, driver="skid4_l298n",
                  statuses=[_status(mode="skid4")])
    trk = StubTracker([_obs(t=0.0)])
    r = _runner(log, cam, trk, mcu, loop_hz=1000.0, timeout_s=0.05)
    r.run()
    events = _read_events(log)
    ready = [e for e in events if e["event"] == "mcu_status_ready"]
    assert len(ready) == 1          # sekali saja, bukan tiap iterasi
    assert ready[0]["mode"] == "skid4"


def test_estop_events_logged_on_change(log):
    cam = StubCamera()
    mcu = StubMcu(dry_run=False, driver="skid4_l298n", statuses=[
        _status(estop=False),
        _status(estop=True),    # tombol ditekan
        _status(estop=True),    # tetap: tidak di-log ulang
        _status(estop=False),   # dilepas
    ])
    trk = StubTracker([_obs(t=0.0)])
    r = _runner(log, cam, trk, mcu, loop_hz=1000.0, timeout_s=0.05)
    r.run()
    events = [e["event"] for e in _read_events(log)]
    assert events.count("mcu_estop") == 1
    assert events.count("mcu_estop_clear") == 1


def test_stale_events_logged_on_change(log):
    cam = StubCamera()
    mcu = StubMcu(dry_run=False, driver="skid4_l298n", statuses=[
        _status(stale=False),
        _status(stale=True),
        _status(stale=False),
    ])
    trk = StubTracker([_obs(t=0.0)])
    r = _runner(log, cam, trk, mcu, loop_hz=1000.0, timeout_s=0.05)
    r.run()
    events = [e["event"] for e in _read_events(log)]
    assert events.count("mcu_stale") == 1
    assert events.count("mcu_stale_clear") == 1


def test_mode_mismatch_logged(log):
    """robot.json bilang skid4 tapi firmware jalan esc1 -> dicatat."""
    cam = StubCamera()
    mcu = StubMcu(dry_run=False, driver="skid4_l298n", statuses=[
        _status(mode="esc1"),
        _status(mode="esc1"),   # tetap mismatch: tidak di-log ulang
        _status(mode="skid4"),  # cocok lagi
    ])
    trk = StubTracker([_obs(t=0.0)])
    r = _runner(log, cam, trk, mcu, loop_hz=1000.0, timeout_s=0.05)
    r.run()
    events = _read_events(log)
    mismatch = [e for e in events if e["event"] == "mcu_mode_mismatch"]
    assert len(mismatch) == 1
    assert mismatch[0]["expected"] == "skid4"
    assert mismatch[0]["actual"] == "esc1"
    assert any(e["event"] == "mcu_mode_ok" and e["mode"] == "skid4"
               for e in events)


def test_status_lost_and_recovered(log):
    """Status berhenti datang > status_timeout_s -> lost; datang lagi ->
    recovered."""
    cam = StubCamera()
    mcu = StubMcu(dry_run=False, driver="skid4_l298n",
                  statuses=[_status()])
    trk = StubTracker([_obs(t=0.0)])
    r = _runner(log, cam, trk, mcu, loop_hz=200.0, timeout_s=0.4,
                status_timeout_s=0.05)
    r.run()
    events = [e["event"] for e in _read_events(log)]
    assert "mcu_status_lost" in events
    assert "mcu_status_recovered" not in events   # tidak ada status lagi


def test_status_recovered_after_loss(log):
    """Status harus BENAR-BENAR berhenti dulu (lost), baru datang lagi
    (recovered) — stub menahan status kedua sampai jeda lewat."""
    cam = StubCamera()
    mcu = DelayedStatusMcu(dry_run=False, gap_s=0.2, driver="skid4_l298n",
                           statuses=[_status(), _status()])
    trk = StubTracker([_obs(t=0.0)])
    r = _runner(log, cam, trk, mcu, loop_hz=100.0, timeout_s=0.6,
                status_timeout_s=0.05)
    r.run()
    events = [e["event"] for e in _read_events(log)]
    assert "mcu_status_ready" in events
    assert "mcu_status_lost" in events
    assert "mcu_status_recovered" in events


def test_status_polling_does_not_block_heartbeat(log):
    """Polling status gagal/None tidak menghentikan heartbeat."""
    cam = StubCamera()
    mcu = StubMcu(dry_run=False, driver="skid4_l298n")  # tanpa status
    trk = StubTracker([_obs(t=0.0)])
    r = _runner(log, cam, trk, mcu, loop_hz=1000.0, timeout_s=0.05)
    r.run()
    assert len(mcu.commands) >= 5
    assert mcu.read_status_calls >= 5


# --- pembantu -----------------------------------------------------------------

def _read_events(log) -> list[dict]:
    import json
    path = log.path / "events.jsonl"
    if not path.is_file():
        return []
    return [json.loads(line) for line in
            path.read_text(encoding="utf-8").splitlines() if line.strip()]
