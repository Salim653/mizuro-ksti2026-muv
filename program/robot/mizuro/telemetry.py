"""Log run lokal: satu direktori bertimestamp per percobaan.

telemetry.csv = satu baris per iterasi loop (untuk analisis setelah run).
events.jsonl = peristiwa diskrit (ready, fault, shutdown, overrun, ...).
Semua log lokal: aturan lomba melarang komunikasi selama percobaan.
"""
from __future__ import annotations

import csv
import json
import time
from pathlib import Path
from typing import Any

from .states import StateMachine
from .types import Command, Observation

FIELDS = (
    "t_monotonic", "state", "reason", "lane_error_m", "heading_error_rad",
    "curvature_inv_m", "confidence", "speed_mps", "steering_rad", "command_reason",
)


class RunLog:
    """Satu run = satu direktori `run-YYYYmmdd-HHMMSS` di bawah output_dir."""

    def __init__(self, output_dir: str) -> None:
        self.path = Path(output_dir) / time.strftime("run-%Y%m%d-%H%M%S")
        self.path.mkdir(parents=True, exist_ok=False)
        self._telemetry = (self.path / "telemetry.csv").open(
            "w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._telemetry, fieldnames=list(FIELDS))
        self._writer.writeheader()
        self._events = (self.path / "events.jsonl").open("w", encoding="utf-8")
        self._closed = False

    def event(self, name: str, **details: Any) -> None:
        if self._closed:
            raise RuntimeError("RunLog sudah ditutup")
        record = {"t_monotonic": time.monotonic(), "event": name, **details}
        self._events.write(json.dumps(record) + "\n")
        self._events.flush()

    def row(self, machine: StateMachine, obs: Observation | None,
            cmd: Command) -> None:
        if self._closed:
            raise RuntimeError("RunLog sudah ditutup")
        self._writer.writerow({
            "t_monotonic": time.monotonic(),
            "state": machine.state.name,
            "reason": machine.reason,
            "lane_error_m": None if obs is None else obs.lane_error_m,
            "heading_error_rad": None if obs is None else obs.heading_error_rad,
            "curvature_inv_m": None if obs is None else obs.curvature_inv_m,
            "confidence": 0.0 if obs is None else obs.confidence,
            "speed_mps": cmd.speed_mps,
            "steering_rad": cmd.steering_rad,
            "command_reason": cmd.reason,
        })
        self._telemetry.flush()

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._telemetry.close()
        self._events.close()

    def __enter__(self) -> "RunLog":
        return self

    def __exit__(self, *exc: Any) -> None:
        self.close()
