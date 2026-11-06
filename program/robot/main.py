#!/usr/bin/env python3
"""Runtime robot MIZURO.

Yang sudah ada: config tervalidasi, state machine, protokol serial, log run,
self-check, mode bench `--stream-only` (kamera + MJPEG + tracker lajur,
TANPA motor), dan mode closed-loop `--closed-loop` (persepsi -> Stanley ->
MCU, dry_run default). Mode live dengan motor butuh dry_run=false +
roda terangkat + E-stop fisik.

Flag `--dashboard` (bench/pit) menyalakan server dashboard run di thread
terpisah selama mode berjalan — membaca log yang SEDANG ditulis proses
ini, jadi telemetry terlihat hidup di browser. Default OFF: saat lomba
semua HTTP wajib mati.
"""
from __future__ import annotations

import argparse
import math
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


def build_detector(config: Config):
    """Pilih backend detektor checkpoint dari `checkpoint_mode`.

    none = mati (default; lane keeping tetap jalan penuh)
    cv   = classical CV warna/bentuk (nol training)
    model = YOLO ONNX (butuh model terlatih kelas arena)

    Detektor TIDAK mengontrol robot — hasilnya cuma masuk log.
    """
    mode = config.checkpoint_mode
    if mode == "none":
        return None
    if mode == "cv":
        from mizuro.perception.checkpoint_cv import CheckpointDetectorCV
        det = CheckpointDetectorCV.from_config(config)
        print(f"detektor: CV klasik (warna {config.checkpoint_color}, "
              f"area {config.checkpoint_min_area_px}"
              f"-{config.checkpoint_max_area_px} px, "
              f"solidity >= {config.checkpoint_min_solidity})")
        return det
    if mode == "model":
        from mizuro.perception.detector import SemanticDetector
        det = SemanticDetector(
            config.model_path, config.confidence_threshold)
        print(f"detektor: {config.model_path} "
              f"(threshold {config.confidence_threshold})")
        return det
    raise ValueError(f"checkpoint_mode tidak dikenal: {mode}")


def serve_dashboard(config: Config, port: int, feed_url: str = ""):
    """Bench/pit: serve dashboard run di thread daemon.

    Dashboard membaca log run yang SEDANG ditulis proses ini
    (`latest_status` memilih run terbaru dari disk), jadi telemetry
    terlihat hidup di browser tanpa pipa antar-proses.

    `feed_url` (opsional) = URL MJPEG mode ini; di-inject ke halaman
    sehingga panel kamera langsung terisi.

    WAJIB mati saat percobaan lomba — aturan melarang komunikasi apa
    pun dengan robot selama run. Bind mengikuti `stream_host`
    (default 127.0.0.1; 0.0.0.0 hanya jaringan pit tepercaya).
    """
    import threading

    from dashboard import make_server

    host = config.stream_host or "127.0.0.1"
    server = make_server(host, port, Path(config.output_dir),
                         feed_url=feed_url)
    threading.Thread(target=server.serve_forever, name="dashboard",
                     daemon=True).start()
    print(f"dashboard : http://{host}:{server.server_port}/ "
          f"(log: {config.output_dir})")
    print("PERINGATAN: matikan dashboard + stream sebelum percobaan lomba")
    return server


def stream_only(config: Config, dashboard_port: int | None = None) -> None:
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

    # Kalibrasi: homography_path (kalau di-set) = intrinsik TERUKUR dari
    # tools/calibrate.py, termasuk distorsi. Tanpa itu, dari Config
    # (hFOV default). Frame di-undistort sekali di Camera kalau ada dist.
    if config.homography_path:
        calib = CameraCalibration.from_json(config.homography_path)
        undistort = ((calib.camera_matrix(), calib.dist)
                     if calib.dist else None)
        print(f"kalibrasi: {config.homography_path} "
              f"(fx={calib.fx:.1f} hFOV={calib.hfov_deg():.1f} deg"
              f"{', undistort ON' if undistort else ''})")
    else:
        calib = CameraCalibration.from_config(config)
        undistort = None
        print("kalibrasi: default (hFOV 62 deg) — set homography_path "
              "di robot.json buat intrinsik terukur")
    tracker = LaneTracker(polarity=config.lane_polarity,
                          roi_top=config.lane_roi_top,
                          min_confidence=config.lane_min_confidence)
    # Detektor checkpoint: opsional, dikendalikan checkpoint_mode.
    # Tidak memblokir loop; hasilnya cuma masuk log.
    detector = build_detector(config)
    hub = FrameHub(quality=config.stream_quality, fps=config.stream_fps,
                   width=config.stream_width, height=config.stream_height)
    dump_dir = Path(config.output_dir) / "bench-frames"
    dump_dir.mkdir(parents=True, exist_ok=True)

    with Camera.from_config(config, undistort=undistort) as cam, \
            MjpegServer.from_config(config, hub) as server:
        print(f"stream : {server.url}")
        print(f"kamera : {config.image_width}x{config.image_height} "
              f"@{config.camera_fps}fps  mount z={calib.height_m} m "
              f"pitch={math.degrees(calib.pitch_rad)} deg x={calib.mount_x_m} "
              f"y={calib.mount_y_m} yaw={math.degrees(calib.mount_yaw_rad)} "
              f"roll={math.degrees(calib.mount_roll_rad)}")
        print(f"dump   : {dump_dir} (setiap frame ke-30)")
        print("Ctrl+C untuk berhenti")
        n = 0
        dash = (serve_dashboard(config, dashboard_port, feed_url=server.url)
                if dashboard_port is not None else None)
        try:
            while True:
                frame = cam.read()
                hub.publish(frame)
                obs = tracker.estimate(frame, calib, float(n))
                n += 1
                if n % 30 == 0:
                    cv2.imwrite(str(dump_dir / f"frame-{n:06d}.jpg"), frame)
                if detector is not None and n % 15 == 0:
                    dets = detector.infer(frame)
                    if dets:
                        summary = ", ".join(
                            f"{d.label}:{d.confidence:.2f}" for d in dets)
                        print(f"[{n:6d}] DET   {summary} "
                              f"({detector.last_inference_ms:.0f} ms)")
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
        finally:
            if dash is not None:
                dash.shutdown()
                dash.server_close()


def run_closed(config: Config, dashboard_port: int | None = None) -> None:
    """Fase 4: loop kontrol tertutup — persepsi -> Stanley -> MCU.

    dry_run default true: loop penuh berjalan (kamera, tracker,
    controller, telemetry), MCU tidak menulis serial. Output command
    di-log untuk analisis sebelum motor pernah menyala.

    Motor hanya menyala kalau dry_run=false DAN roda terangkat DAN
    E-stop fisik terpasang. Jangan skip salah satu.
    """
    import cv2

    from mizuro.camera import Camera
    from mizuro.controller import StanleyController
    from mizuro.perception.calibration import CameraCalibration
    from mizuro.perception.classical import LaneTracker
    from mizuro.runner import ClosedLoopRunner

    if config.homography_path:
        calib = CameraCalibration.from_json(config.homography_path)
        undistort = ((calib.camera_matrix(), calib.dist)
                     if calib.dist else None)
        print(f"kalibrasi: {config.homography_path} "
              f"(fx={calib.fx:.1f} hFOV={calib.hfov_deg():.1f} deg"
              f"{', undistort ON' if undistort else ''})")
    else:
        calib = CameraCalibration.from_config(config)
        undistort = None
        print("kalibrasi: default (hFOV 62 deg) — set homography_path "
              "di robot.json buat intrinsik terukur")

    tracker = LaneTracker(polarity=config.lane_polarity,
                          roi_top=config.lane_roi_top,
                          min_confidence=config.lane_min_confidence)
    controller = StanleyController.from_config(config)

    detector = build_detector(config)

    with RunLog(config.output_dir) as log, \
            Camera.from_config(config, undistort=undistort) as cam, \
            McuLink(config.serial_device, config.serial_baud,
                    dry_run=config.dry_run,
                    driver=config.motor_driver) as mcu:
        machine = StateMachine()
        runner = ClosedLoopRunner(
            camera=cam, tracker=tracker, controller=controller, mcu=mcu,
            log=log, machine=machine, calib=calib,
            loop_hz=config.loop_hz, run_timeout_s=config.run_timeout_s,
            max_inference_ms=config.max_inference_ms, detector=detector)
        mode = "DRY-RUN (tanpa serial)" if config.dry_run \
            else "LIVE (motor aktif!)"
        print(f"closed-loop: {mode}")
        print(f"loop       : {config.loop_hz} Hz, timeout "
              f"{config.run_timeout_s:.0f}s")
        print(f"log        : {log.path}")
        print("Ctrl+C untuk berhenti")
        dash = (serve_dashboard(config, dashboard_port)
                if dashboard_port is not None else None)
        try:
            runner.run()
            print(f"\nSelesai: {runner.iterations} iterasi, "
                  f"{runner.overruns} overrun, "
                  f"{runner.lane_lost_count} frame lajur hilang")
            print(f"Log: {log.path}")
        finally:
            if dash is not None:
                dash.shutdown()
                dash.server_close()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Runtime robot MIZURO (fondasi + bench persepsi + "
                    "closed-loop)")
    parser.add_argument("--config", type=Path, default=Path("robot.json"),
                        help="file config JSON (default: robot.json)")
    parser.add_argument("--self-check", action="store_true",
                        help="verifikasi fondasi tanpa hardware")
    parser.add_argument("--stream-only", action="store_true",
                        help="bench: kamera + MJPEG + tracker lajur, tanpa "
                             "motor (butuh /usr/bin/python3 di Jetson)")
    parser.add_argument("--closed-loop", action="store_true",
                        help="Fase 4: persepsi -> Stanley -> MCU. dry_run "
                             "default true (tanpa serial). Motor hanya "
                             "nyala kalau dry_run=false + roda terangkat")
    parser.add_argument("--dashboard", action="store_true",
                        help="bench/pit: serve dashboard run selama mode "
                             "berjalan (butuh --stream-only atau "
                             "--closed-loop). MATIKAN saat lomba")
    parser.add_argument("--dashboard-port", type=int, default=18765,
                        help="port dashboard (default: 18765)")
    args = parser.parse_args()

    if args.dashboard and not (args.stream_only or args.closed_loop):
        raise SystemExit(
            "--dashboard butuh --stream-only atau --closed-loop; "
            "dashboard membaca log run yang sedang ditulis mode tersebut")
    if not 1 <= args.dashboard_port <= 65535:
        raise SystemExit(
            f"--dashboard-port harus dalam [1, 65535]: {args.dashboard_port}")

    if args.self_check:
        self_check()
        return
    if args.config.is_file():
        config = Config.load(args.config)
    elif args.stream_only or args.closed_loop:
        config = Config()   # default; mount disesuaikan di robot.json
    else:
        raise SystemExit(
            f"config tidak ditemukan: {args.config}; "
            f"salin dari robot.example.json dan sesuaikan")
    if args.stream_only:
        stream_only(config,
                    args.dashboard_port if args.dashboard else None)
        return
    if args.closed_loop:
        run_closed(config,
                   args.dashboard_port if args.dashboard else None)
        return
    if not config.dry_run:
        raise SystemExit(
            "mode live butuh --closed-loop; "
            "set dry_run=true untuk uji fondasi")
    boot_dry(config)


if __name__ == "__main__":
    main()
