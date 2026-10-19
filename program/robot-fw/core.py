from __future__ import annotations

import json
import time
from dataclasses import dataclass
from enum import Enum, auto
from pathlib import Path
from typing import Any


class State(Enum):
    BOOT = auto()
    READY = auto()
    RUNNING = auto()
    DEGRADED = auto()
    STOPPED = auto()
    FAULT = auto()


@dataclass
class Config:
    camera_pipeline: str = ""
    model_path: str = ""
    serial_device: str = ""
    serial_baud: int = 115200
    output_dir: str = "runs"
    confidence_threshold: float = 0.5
    max_inference_ms: float = 150.0
    image_width: int = 1280
    image_height: int = 720
    max_speed_mps: float = 0.8
    steering_limit_rad: float = 0.55
    lane_lost_frames: int = 3
    loop_hz: float = 60.0
    dry_run: bool = True
    save_frames: bool = False
    motor_driver: str = "external_esc"

    @classmethod
    def load(cls, path: Path) -> "Config":
        data = json.loads(path.read_text(encoding="utf-8"))
        extra = set(data) - set(cls.__dataclass_fields__)
        if extra:
            raise ValueError(f"unknown config keys: {sorted(extra)}")
        cfg = cls(**data)
        if not 0.0 <= cfg.confidence_threshold <= 1.0:
            raise ValueError("confidence_threshold must be in [0, 1]")
        if cfg.max_inference_ms <= 0 or cfg.image_width <= 0 or cfg.image_height <= 0:
            raise ValueError("inference limit and image dimensions must be positive")
        if not 0 < cfg.max_speed_mps <= 0.8 or not 0 < cfg.steering_limit_rad <= 0.55:
            raise ValueError("max_speed_mps must be <= 0.8 and steering_limit_rad <= 0.55")
        if cfg.lane_lost_frames < 1 or cfg.loop_hz <= 0:
            raise ValueError("lane_lost_frames and loop_hz must be positive")
        if cfg.motor_driver not in {"external_esc", "mcu_hbridge"}:
            raise ValueError("motor_driver must be external_esc or mcu_hbridge")
        if not cfg.dry_run and not all((cfg.camera_pipeline, cfg.model_path, cfg.serial_device)):
            raise ValueError("live mode requires camera_pipeline, model_path, serial_device")
        if cfg.save_frames and cfg.dry_run:
            raise ValueError("save_frames requires live capture")
        return cfg


@dataclass
class Observation:
    timestamp: float
    lane_error_m: float | None = None
    heading_error_rad: float | None = None
    curvature_inv_m: float | None = None
    confidence: float = 0.0
    detections: list[dict[str, Any]] | None = None


@dataclass
class Command:
    speed_mps: float = 0.0
    steering_rad: float = 0.0
    reason: str = "safe default"


class StateMachine:
    def __init__(self) -> None:
        self.state = State.BOOT
        self.reason = "startup"
        self.started_at: float | None = None

    def transition(self, new: State, reason: str) -> None:
        allowed = {
            State.BOOT: {State.READY, State.FAULT, State.STOPPED},
            State.READY: {State.RUNNING, State.FAULT, State.STOPPED},
            State.RUNNING: {State.DEGRADED, State.STOPPED, State.FAULT},
            State.DEGRADED: {State.RUNNING, State.STOPPED, State.FAULT},
            State.STOPPED: {State.READY, State.FAULT},
            State.FAULT: set(),
        }
        if new not in allowed[self.state]:
            raise RuntimeError(f"invalid transition: {self.state.name} -> {new.name}")
        self.state, self.reason = new, reason
        if new is State.RUNNING and self.started_at is None:
            self.started_at = time.monotonic()
