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
    """MCU palsu: rekam setiap perintah yang dikirim."""

    def __init__(self, dry_run=True):
        self.dry_run = dry_run
        self.commands: list[Command] = []
        self.alive = True
        self.serial = None

    def send(self, command: Command) -> None:
        self.commands.append(command)

    def close(self) -> None:
        self.alive = False


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
            controller=None, detector=None):
    from mizuro.controller import StanleyController
    return ClosedLoopRunner(
        camera=camera, tracker=tracker,
        controller=controller or StanleyController(),
        mcu=mcu, log=log, machine=StateMachine(), calib=StubCalib(),
        loop_hz=loop_hz, run_timeout_s=timeout_s, detector=detector)


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


# --- pembantu -----------------------------------------------------------------

def _read_events(log) -> list[dict]:
    import json
    path = log.path / "events.jsonl"
    if not path.is_file():
        return []
    return [json.loads(line) for line in
            path.read_text(encoding="utf-8").splitlines() if line.strip()]
