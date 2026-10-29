#!/usr/bin/env python3
"""Runtime robot MIZURO — Fase 0: fondasi.

Yang sudah ada: config tervalidasi, state machine, protokol serial, log run,
dan self-check. Loop persepsi (kamera -> lajur -> kontrol -> MCU) datang di
Fase 1-3; sampai itu ada, mode live sengaja DITOLAK alih-alih setengah jalan.
"""
from __future__ import annotations

import argparse
import tempfile
from dataclasses import asdict
from pathlib import Path

from mizuro.config import Config
from mizuro.mcu import McuLink
from mizuro.protocol import MAX_SPEED_MPS, MAX_STEER_RAD, format_command
from mizuro.states import State, StateMachine
from mizuro.telemetry import RunLog
from mizuro.types import Command


def self_check() -> None:
    """Verifikasi fondasi tanpa hardware. Gagal = assertion, bukan warning."""
    # --- protokol: format persis + batas ditegakkan ---
    assert format_command(Command(0.8, -0.55)) == b"0.8000,-0.5500\n"
    assert format_command(Command()) == b"0.0000,0.0000\n"
    for bad in (Command(0.81, 0.0), Command(-0.81, 0.0), Command(0.0, 0.56),
                Command(0.0, -0.56), Command(float("nan"), 0.0),
                Command(float("inf"), 0.0)):
        try:
            format_command(bad)
        except ValueError:
            pass
        else:
            raise AssertionError(f"perintah di luar batas diterima: {bad}")

    # --- config: default valid, nilai invalid ditolak, contoh config valid ---
    cfg = Config()
    assert cfg.dry_run is True
    assert cfg.loop_hz == 60.0 and cfg.run_timeout_s == 300.0
    assert cfg.max_speed_mps == MAX_SPEED_MPS
    assert cfg.steering_limit_rad == MAX_STEER_RAD
    for patch in ({"loop_hz": 0}, {"max_speed_mps": 0.9},
                  {"steering_limit_rad": 0.6}, {"stream_port": 70000},
                  {"lane_polarity": "ungu"}, {"motor_driver": "sihir"},
                  {"save_frames": True}, {"dry_run": False}):
        try:
            Config(**{**asdict(cfg), **patch}).validate()
        except ValueError:
            pass
        else:
            raise AssertionError(f"config invalid diterima: {patch}")
    Config.load(Path(__file__).with_name("robot.example.json"))

    # --- state machine: alur benar + transisi tidak valid ditolak ---
    sm = StateMachine()
    sm.transition(State.READY, "checks")
    sm.transition(State.RUNNING, "start")
    first_started = sm.started_at
    assert first_started is not None
    sm.transition(State.DEGRADED, "lane lost")
    sm.transition(State.RUNNING, "recovered")
    assert sm.started_at == first_started  # timer run tidak reset oleh DEGRADED
    sm.transition(State.STOPPED, "done")
    try:
        sm.transition(State.RUNNING, "invalid")
    except RuntimeError:
        pass
    else:
        raise AssertionError("transisi STOPPED -> RUNNING diterima")

    # --- telemetry: direktori run + dua file log benar-benar dibuat ---
    with tempfile.TemporaryDirectory() as tmp:
        with RunLog(tmp) as log:
            log.event("selftest")
            log.row(sm, None, Command())
            assert (log.path / "telemetry.csv").stat().st_size > 0
            assert (log.path / "events.jsonl").stat().st_size > 0

    # --- mcu: dry-run tidak menyentuh serial ---
    mcu = McuLink("/dev/ttyTIDAKADA", dry_run=True)
    mcu.send(Command(0.5, 0.1))
    assert mcu.alive and mcu.serial is None
    mcu.close()

    print("self-check passed")


def boot_dry(config: Config) -> None:
    """Fase 0: BOOT -> READY -> STOPPED tanpa hardware.

    Membuktikan config, state machine, telemetry, dan protokol menyambung
    dengan benar sebelum loop persepsi ada.
    """
    with RunLog(config.output_dir) as log:
        machine = StateMachine()
        mcu = McuLink(config.serial_device, config.serial_baud, dry_run=True)
        try:
            machine.transition(State.READY, "startup checks passed")
            log.event("ready", config=asdict(config))
            log.row(machine, None, Command())
            mcu.send(Command())  # heartbeat nol; dry-run tidak menulis apa pun
            log.event("dry_run",
                      message="Fase 0: loop persepsi belum ada; roda tidak bergerak")
            print(f"Dry run selesai. Log: {log.path}")
        finally:
            machine.transition(State.STOPPED, "dry run complete")
            log.row(machine, None, Command(reason="shutdown"))
            mcu.close()
            log.event("shutdown", state=machine.state.name, reason=machine.reason)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Runtime robot MIZURO (Fase 0: fondasi)")
    parser.add_argument("--config", type=Path, default=Path("robot.json"),
                        help="file config JSON (default: robot.json)")
    parser.add_argument("--self-check", action="store_true",
                        help="verifikasi fondasi tanpa hardware")
    args = parser.parse_args()

    if args.self_check:
        self_check()
        return
    if not args.config.is_file():
        raise SystemExit(
            f"config tidak ditemukan: {args.config}; "
            f"salin dari robot.example.json dan sesuaikan")
    config = Config.load(args.config)
    if not config.dry_run:
        raise SystemExit(
            "mode live belum diimplementasikan (Fase 3); "
            "set dry_run=true untuk uji fondasi")
    boot_dry(config)


if __name__ == "__main__":
    main()
