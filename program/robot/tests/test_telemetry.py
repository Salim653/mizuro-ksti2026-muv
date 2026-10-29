"""Tes log run: struktur direktori, isi CSV/JSONL, dan siklus close."""
import csv
import json

import pytest

from mizuro.states import State, StateMachine
from mizuro.telemetry import RunLog
from mizuro.types import Command, Observation


def _ready_machine() -> StateMachine:
    sm = StateMachine()
    sm.transition(State.READY, "checks")
    return sm


def test_runlog_writes_csv_and_jsonl(tmp_path):
    with RunLog(str(tmp_path)) as log:
        assert log.path.is_dir()
        assert (log.path / "telemetry.csv").exists()
        assert (log.path / "events.jsonl").exists()
        log.event("ready", note="uji")
        machine = _ready_machine()
        log.row(machine, Observation(timestamp=1.0, lane_error_m=0.02,
                                     confidence=0.9),
                Command(0.3, 0.05, reason="track"))
        log.row(machine, None, Command())

    rows = list(csv.DictReader((log.path / "telemetry.csv").open(encoding="utf-8")))
    assert len(rows) == 2
    assert rows[0]["state"] == "READY"
    assert rows[0]["lane_error_m"] == "0.02"
    assert rows[0]["confidence"] == "0.9"
    assert rows[0]["command_reason"] == "track"
    assert rows[1]["confidence"] == "0.0"  # obs None -> confidence 0
    assert rows[1]["lane_error_m"] == ""

    events = [json.loads(line)
              for line in (log.path / "events.jsonl").read_text(encoding="utf-8").splitlines()]
    assert events[0]["event"] == "ready"
    assert events[0]["note"] == "uji"


def test_close_is_idempotent(tmp_path):
    log = RunLog(str(tmp_path))
    log.close()
    log.close()  # tidak melempar


def test_write_after_close_raises(tmp_path):
    log = RunLog(str(tmp_path))
    log.close()
    with pytest.raises(RuntimeError, match="ditutup"):
        log.row(_ready_machine(), None, Command())
    with pytest.raises(RuntimeError, match="ditutup"):
        log.event("terlambat")
