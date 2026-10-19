import csv
import json
import time
from pathlib import Path
from typing import Any

from core import Command, Observation, StateMachine


class RunLog:
    def __init__(self, output_dir: str) -> None:
        self.path = Path(output_dir) / time.strftime("run-%Y%m%d-%H%M%S")
        self.path.mkdir(parents=True, exist_ok=False)
        self.telemetry = (self.path / "telemetry.csv").open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.telemetry, fieldnames=[
            "t_monotonic", "state", "reason", "lane_error_m", "heading_error_rad",
            "curvature_inv_m", "confidence", "speed_mps", "steering_rad", "command_reason",
        ])
        self.writer.writeheader()
        self.events = (self.path / "events.jsonl").open("w", encoding="utf-8")

    def event(self, name: str, **details: Any) -> None:
        self.events.write(json.dumps({"t_monotonic": time.monotonic(), "event": name, **details}) + "\n")
        self.events.flush()

    def row(self, state: StateMachine, obs: Observation | None, cmd: Command) -> None:
        self.writer.writerow({
            "t_monotonic": time.monotonic(), "state": state.state.name, "reason": state.reason,
            "lane_error_m": None if obs is None else obs.lane_error_m,
            "heading_error_rad": None if obs is None else obs.heading_error_rad,
            "curvature_inv_m": None if obs is None else obs.curvature_inv_m,
            "confidence": 0.0 if obs is None else obs.confidence,
            "speed_mps": cmd.speed_mps, "steering_rad": cmd.steering_rad,
            "command_reason": cmd.reason,
        })
        self.telemetry.flush()

    def close(self) -> None:
        self.telemetry.close()
        self.events.close()
