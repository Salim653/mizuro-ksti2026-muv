from __future__ import annotations

import argparse
import logging
import signal
import time
from dataclasses import asdict
from pathlib import Path

from camera import Camera
from controller import Controller
from core import Command, Config, Observation, State, StateMachine
from mcu import McuLink, format_command
from telemetry import RunLog
from vision import LaneEstimator, OnnxDetector


class StartGate:
    def wait(self, stopping) -> bool:
        print("READY, press Enter to arm; Ctrl-C to leave stopped")
        try:
            return input() == "" and not stopping()
        except EOFError:
            return False


def self_check() -> None:
    sm = StateMachine()
    sm.transition(State.READY, "checks passed")
    sm.transition(State.RUNNING, "start requested")
    try:
        Config.load(Path(__file__).with_name("robot.example.json"))
    except Exception as exc:
        raise AssertionError(f"example config invalid: {exc}") from exc
    try:
        sm.transition(State.BOOT, "invalid")
    except RuntimeError:
        pass
    else:
        raise AssertionError("invalid transition accepted")
    safe = Controller(0.8, 0.55).command(Observation(timestamp=0))
    assert safe.speed_mps == 0.0
    assert safe.steering_rad == 0.0
    assert format_command(Command(0.8, -0.55)) == b"0.8000,-0.5500\n"
    try:
        format_command(Command(0.81, 0.0))
    except ValueError:
        pass
    else:
        raise AssertionError("out-of-range serial command accepted")
    assert Config().loop_hz == 60.0
    assert Config().dry_run
    print("self-check passed")


def run(config: Config) -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    log = RunLog(config.output_dir)
    machine = StateMachine()
    camera = detector = mcu = None
    stopping = False
    lane_lost = 0
    cmd = Command()
    obs = None

    def request_stop(_signum, _frame) -> None:
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        mcu = McuLink(config.serial_device, config.serial_baud, config.dry_run)
        if not config.dry_run:
            camera = Camera(config.camera_pipeline, config.image_width, config.image_height)
            detector = OnnxDetector(config.model_path, config.confidence_threshold)
        machine.transition(State.READY, "startup checks passed")
        log.event("ready", config=asdict(config))
        log.row(machine, obs, cmd)
        if config.dry_run:
            log.event("dry_run", message="hardware disabled; no movement commands sent")
            print(f"Dry run initialized. Logs: {log.path}")
            return
        if not StartGate().wait(lambda: stopping):
            machine.transition(State.STOPPED, "start cancelled")
            return
        machine.transition(State.RUNNING, "start confirmed")
        controller = Controller(config.max_speed_mps, config.steering_limit_rad)
        estimator = LaneEstimator()
        period_s = 1.0 / config.loop_hz
        next_tick = time.monotonic()
        while not stopping:
            loop_start = time.monotonic()
            frame = camera.read()
            infer_start = time.monotonic()
            detections = detector.infer(frame)
            infer_ms = (time.monotonic() - infer_start) * 1000
            obs = estimator.estimate(frame, detections, time.monotonic())
            if infer_ms > config.max_inference_ms or obs.confidence <= 0:
                lane_lost += 1
                cmd = Command(reason="inference timeout" if infer_ms > config.max_inference_ms else "lane unavailable")
                if lane_lost >= config.lane_lost_frames and machine.state is State.RUNNING:
                    machine.transition(State.DEGRADED, cmd.reason)
            else:
                lane_lost = 0
                cmd = controller.command(obs)
                if machine.state is State.DEGRADED:
                    machine.transition(State.RUNNING, "lane estimate recovered")
            mcu.send(cmd)
            log.row(machine, obs, cmd)
            if config.save_frames and not camera.cv2.imwrite(
                    str(log.path / f"frame-{int(loop_start * 1000)}.jpg"), frame):
                log.event("frame_save_failed")
            next_tick += period_s
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                log.event("loop_overrun", overrun_ms=round(-delay * 1000, 2))
                next_tick = time.monotonic()
        if machine.state in (State.RUNNING, State.DEGRADED):
            machine.transition(State.STOPPED, "stop requested")
    except Exception as exc:
        if machine.state is not State.FAULT:
            machine.transition(State.FAULT, str(exc))
        logging.exception("robot runtime fault")
        log.event("fault", error=str(exc))
    finally:
        try:
            if mcu:
                mcu.send(Command(reason="shutdown"))
        finally:
            try:
                if camera:
                    camera.close()
            finally:
                try:
                    if mcu:
                        mcu.close()
                finally:
                    log.row(machine, obs, cmd)
                    log.event("shutdown", state=machine.state.name, reason=machine.reason)
                    log.close()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=Path("robot.json"))
    parser.add_argument("--self-check", action="store_true")
    args = parser.parse_args()
    if args.self_check:
        self_check()
        return
    if not args.config.is_file():
        raise SystemExit(f"missing config: {args.config}; copy robot.example.json and edit it")
    run(Config.load(args.config))


if __name__ == "__main__":
    main()
