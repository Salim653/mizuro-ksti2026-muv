#!/usr/bin/env python3
"""Runtime robot MIZURO.

Yang sudah ada: config tervalidasi, state machine, protokol serial, log run,
self-check, dan mode bench `--stream-only` (kamera + MJPEG + tracker lajur,
TANPA motor). Loop kontrol (lajur -> kontrol -> MCU) datang di Fase 3;
sampai itu ada, mode live sengaja DITOLAK alih-alih setengah jalan.
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


def require(condition: object, message: str) -> None:
    """Assert yang TIDAK mati di bawah `python3 -O`.

    `assert` biasa dihapus interpreter saat optimisasi; self-check harus
    tetap gagal keras kalau fondasi rusak, jadi raise eksplisit.
    """
    if not condition:
        raise AssertionError(message)


def self_check() -> None:
    """Verifikasi fondasi tanpa hardware. Gagal = AssertionError, bukan warning."""
    # --- protokol: format persis + batas ditegakkan ---
    require(format_command(Command(0.8, -0.55)) == b"0.8000,-0.5500\n",
            "format perintah pada batas salah")
    require(format_command(Command()) == b"0.0000,0.0000\n",
            "format perintah nol salah")
    require(format_command(Command(-0.00004, 0.0)) == b"0.0000,0.0000\n",
            "negatif-nol tidak dinormalkan pada wire")
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
    require(cfg.dry_run is True, "default dry_run harus True")
    require(cfg.loop_hz == 60.0 and cfg.run_timeout_s == 300.0,
            "default loop_hz/run_timeout_s salah")
    require(cfg.max_speed_mps == MAX_SPEED_MPS, "default max_speed_mps salah")
    require(cfg.steering_limit_rad == MAX_STEER_RAD,
            "default steering_limit_rad salah")
    for patch in ({"loop_hz": 0}, {"max_speed_mps": 0.9},
                  {"steering_limit_rad": 0.6}, {"stream_port": 70000},
                  {"lane_polarity": "ungu"}, {"motor_driver": "sihir"},
                  {"save_frames": True}, {"dry_run": False},
                  {"dry_run": "ya"}, {"loop_hz": "60"}, {"stream_port": True}):
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
    require(first_started is not None, "started_at harus terisi saat RUNNING")
    sm.transition(State.DEGRADED, "lane lost")
    sm.transition(State.RUNNING, "recovered")
    require(sm.started_at == first_started,
            "timer run tidak boleh reset oleh DEGRADED")
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
            require((log.path / "telemetry.csv").stat().st_size > 0,
                    "telemetry.csv kosong")
            require((log.path / "events.jsonl").stat().st_size > 0,
                    "events.jsonl kosong")

    # --- mcu: dry-run tidak menyentuh serial ---
    mcu = McuLink("/dev/ttyTIDAKADA", dry_run=True)
    mcu.send(Command(0.5, 0.1))
    require(mcu.alive and mcu.serial is None,
            "dry-run menyentuh serial")
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


def stream_only(config: Config) -> None:
    """Bench: kamera CSI + MJPEG + tracker lajur. TANPA motor, TANPA serial.

    Mode ini tidak menyentuh serial dan tidak menggerakkan apa pun:
    kamera dibuka, tiap frame dipublish ke stream (buat mata) dan
    dilewatkan ke LaneTracker (buat angka). Ini validasi Fase 1 dengan
    kamera asli — kalau tracker gagal di lantai nyata, ketahuan di sini,
    bukan di lintasan.

    Frame sampel ditulis ke runs/bench-frames/ buat diagnosa offline
    (lantai nyata tidak bisa direproduksi synth).

    WAJIB /usr/bin/python3 di Jetson (cv2 butuh backend GStreamer).
    Ctrl+C untuk berhenti.
    """
    import math
    from pathlib import Path

    import cv2

    from mizuro.camera import Camera
    from mizuro.perception.calibration import CameraCalibration
    from mizuro.perception.classical import LaneTracker
    from mizuro.stream import FrameHub, MjpegServer

    calib = CameraCalibration.from_config(config)
    tracker = LaneTracker(polarity=config.lane_polarity,
                          roi_top=config.lane_roi_top,
                          min_confidence=config.lane_min_confidence)
    hub = FrameHub(quality=config.stream_quality, fps=config.stream_fps,
                   width=config.stream_width, height=config.stream_height)
    dump_dir = Path(config.output_dir) / "bench-frames"
    dump_dir.mkdir(parents=True, exist_ok=True)

    with Camera.from_config(config) as cam, \
            MjpegServer.from_config(config, hub) as server:
        print(f"stream : {server.url}")
        print(f"kamera : {config.image_width}x{config.image_height} "
              f"@{config.camera_fps}fps  mount z={config.mount_z_m} m "
              f"pitch={config.mount_pitch_deg} deg x={config.mount_x_m} "
              f"y={config.mount_y_m} yaw={config.mount_yaw_deg} "
              f"roll={config.mount_roll_deg}")
        print(f"dump   : {dump_dir} (setiap frame ke-30)")
        print("Ctrl+C untuk berhenti")
        n = 0
        try:
            while True:
                frame = cam.read()
                hub.publish(frame)
                obs = tracker.estimate(frame, calib, float(n))
                n += 1
                if n % 30 == 0:
                    cv2.imwrite(str(dump_dir / f"frame-{n:06d}.jpg"), frame)
                if obs.confidence == 0:
                    dbg = tracker.last_debug
                    print(f"[{n:6d}] NO-LANE "
                          f"{obs.detections[0].get('reason')} "
                          f"(stasiun {dbg.get('stations_found')}/"
                          f"{dbg.get('stations_total')}, "
                          f"polarity {dbg.get('polarity')})")
                elif n % 15 == 0:
                    print(f"[{n:6d}] lat {obs.lane_error_m * 1000:+8.1f} mm  "
                          f"head {math.degrees(obs.heading_error_rad):+6.2f}"
                          f" deg  conf {obs.confidence:.2f}  "
                          f"stasiun {tracker.last_debug.get('stations_found')}")
        except KeyboardInterrupt:
            print(f"\nstop setelah {n} frame")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Runtime robot MIZURO (fondasi + bench persepsi)")
    parser.add_argument("--config", type=Path, default=Path("robot.json"),
                        help="file config JSON (default: robot.json)")
    parser.add_argument("--self-check", action="store_true",
                        help="verifikasi fondasi tanpa hardware")
    parser.add_argument("--stream-only", action="store_true",
                        help="bench: kamera + MJPEG + tracker lajur, tanpa "
                             "motor (butuh /usr/bin/python3 di Jetson)")
    args = parser.parse_args()

    if args.self_check:
        self_check()
        return
    if args.config.is_file():
        config = Config.load(args.config)
    elif args.stream_only:
        config = Config()   # default; mount disesuaikan di robot.json
    else:
        raise SystemExit(
            f"config tidak ditemukan: {args.config}; "
            f"salin dari robot.example.json dan sesuaikan")
    if args.stream_only:
        stream_only(config)
        return
    if not config.dry_run:
        raise SystemExit(
            "mode live belum diimplementasikan (Fase 3); "
            "set dry_run=true untuk uji fondasi")
    boot_dry(config)


if __name__ == "__main__":
    main()
