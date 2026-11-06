#!/usr/bin/env python3
"""Bench tanpa hardware: video sebagai kamera + MCU virtual.

    # dari program/robot:
    ../../.venv/bin/python tools/mcu_bench.py --video ../../recordings/capture_001.mp4
    ../../.venv/bin/python tools/mcu_bench.py --video laps.mp4 --duration 60
    ../../.venv/bin/python tools/mcu_bench.py --video laps.mp4 --driver external_esc

Menjalankan loop closed-loop PENUH (persepsi -> Stanley -> MCU) dengan:

- kamera : file video pengganti CSI. Tidak butuh GStreamer, jadi jalan
  dengan cv2 pip `.venv` di laptop — tidak perlu Jetson.
- MCU    : VirtualMcu (tools/virtual_mcu.py) — pemetaan output firmware
  dijalankan di Python, PWM per roda / pulse ESC dicatat. TIDAK ADA
  serial, TIDAK ADA hardware, roda tidak pernah bisa menyala di sini.

Output: runs/run-*/ dengan telemetry.csv + events.jsonl seperti run
biasa (bisa dibuka `dashboard.py --runs runs`), plus bench_summary.json
berisi statistik output "motor virtual".

Yang TIDAK diuji alat ini (butuh hardware, lihat PANDUAN butir 6):
polaritas kabel motor, titik netral ESC sungguhan, batas servo, E-stop
fisik, watchdog dengan kabel dicabut, dan getaran kamera di chasis.
"""
from __future__ import annotations

import argparse
import dataclasses
import json
import sys
import threading
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))          # tools/
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))   # program/robot

from main import build_detector, serve_dashboard  # noqa: E402
from mizuro.camera import Camera  # noqa: E402
from mizuro.config import Config  # noqa: E402
from mizuro.controller import StanleyController  # noqa: E402
from mizuro.perception.calibration import CameraCalibration  # noqa: E402
from mizuro.perception.classical import LaneTracker  # noqa: E402
from mizuro.protocol import FIRMWARE_MODES  # noqa: E402
from mizuro.runner import ClosedLoopRunner  # noqa: E402
from mizuro.states import StateMachine  # noqa: E402
from mizuro.telemetry import RunLog  # noqa: E402
from virtual_mcu import VirtualMcu  # noqa: E402


def build_bench_config(base: Config, video: str,
                       driver: str | None) -> Config:
    """Config bench: kamera = file video, MCU virtual (dry_run tetap true).

    dry_run TIDAK bisa dimatikan lewat alat ini — tidak ada serial yang
    boleh disentuh; roda fisik adalah sesi bench terpisah dengan
    interlock (PANDUAN-LANJUTAN-TIM butir 6).
    """
    cfg = dataclasses.replace(
        base,
        camera_pipeline=video,
        dry_run=True,
        motor_driver=driver or base.motor_driver,
    )
    cfg.validate()   # driver ngawur/format salah gagal DI SINI
    return cfg


def _calib_and_undistort(config: Config):
    """Sama dengan run_closed(): homography terukur kalau ada, default
    kalau tidak."""
    if config.homography_path:
        calib = CameraCalibration.from_json(config.homography_path)
        undistort = ((calib.camera_matrix(), calib.dist)
                     if calib.dist else None)
    else:
        calib = CameraCalibration.from_config(config)
        undistort = None
    return calib, undistort


def _progress(runner: ClosedLoopRunner, mcu: VirtualMcu, stop: threading.Event,
              interval_s: float = 5.0) -> None:
    """Cetak kemajuan bench tiap beberapa detik (thread daemon)."""
    while not stop.wait(interval_s):
        last = mcu.last_output
        bits = [f"iter {runner.iterations}"]
        if "wheels" in last:
            pwm = {w: p for w, (_, p) in last["wheels"].items()}
            bits.append("PWM " + "/".join(f"{w}:{p}" for w, p in pwm.items()))
        elif "esc_us" in last:
            bits.append(f"ESC {last['esc_us']}us")
        if "steer_us" in last:
            bits.append(f"steer {last['steer_us']}us")
        print("  [bench] " + "  ".join(bits), flush=True)


def _report(cfg: Config, runner: ClosedLoopRunner, mcu: VirtualMcu,
            video: str, fps: float) -> None:
    s = mcu.summary()
    print("\n=== ringkasan bench (motor virtual, tanpa hardware) ===")
    print(f"video          : {video} ({fps:.1f} fps)")
    print(f"mode           : {s['mode']} (driver {s['driver']})")
    print(f"iterasi        : {runner.iterations}  "
          f"overrun {runner.overruns}  lajur hilang {runner.lane_lost_count}")
    print(f"perintah       : {s['commands']} total, "
          f"{s['zero_fraction'] * 100:.0f}% nol, "
          f"max |speed| {s['max_abs_speed_mps']:.2f} m/s, "
          f"max |steer| {s['max_abs_steer_rad']:.2f} rad")
    if s["mode"] == "skid4":
        print(f"roda (skid4)   : PWM maks {s['max_pwm']}/255, "
              f"ganti arah {s['dir_changes']}x, "
              f"clamp bersama {s['clamp_events']}x")
    if "esc_us_range" in s:
        lo, hi = s["esc_us_range"]
        print(f"ESC            : {lo}-{hi} us "
              f"(netral 1500, deadband 40)")
    if "steer_us_range" in s:
        lo, hi = s["steer_us_range"]
        print(f"servo kemudi   : {lo}-{hi} us (tengah 1500)")
    if s["neutral_holds"]:
        print(f"ESC arming     : {s['neutral_holds']} sampel netral "
              f"(2 s pertama, mirror firmware)")
    if s["stale_status_lines"]:
        print(f"PERINGATAN     : {s['stale_status_lines']} baris status "
              f"stale — loop tidak mengirim setpoint < 100 ms "
              f"(watchdog MCU akan trip di hardware)")


def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--video", required=True,
                   help="file video pengganti kamera (mp4/avi/...)")
    p.add_argument("--config", type=Path, default=Path("robot.json"),
                   help="config JSON dasar (default: robot.json)")
    p.add_argument("--driver", choices=sorted(FIRMWARE_MODES), default=None,
                   help="override motor_driver dari config")
    p.add_argument("--duration", type=float, default=30.0,
                   help="batas waktu bench detik (default: 30)")
    p.add_argument("--no-loop", action="store_true",
                   help="berhenti saat video habis (default: putar ulang)")
    p.add_argument("--dashboard-port", type=int, default=None,
                   help="nyalakan dashboard run di port ini (bench saja; "
                        "MATIKAN saat lomba)")
    args = p.parse_args()

    if not Path(args.video).is_file():
        raise SystemExit(f"video tidak ada: {args.video}")
    if args.duration <= 0:
        raise SystemExit("--duration harus positif")

    if args.config.is_file():
        config = Config.load(args.config)
    else:
        config = Config()
    cfg = build_bench_config(config, args.video, args.driver)

    calib, undistort = _calib_and_undistort(cfg)
    tracker = LaneTracker(polarity=cfg.lane_polarity,
                          roi_top=cfg.lane_roi_top,
                          min_confidence=cfg.lane_min_confidence)
    controller = StanleyController.from_config(cfg)
    detector = build_detector(cfg)

    with RunLog(cfg.output_dir) as log, \
            Camera.from_config(cfg, undistort=undistort,
                               video_loop=not args.no_loop) as cam, \
            VirtualMcu(cfg.motor_driver) as mcu:
        machine = StateMachine()
        runner = ClosedLoopRunner(
            camera=cam, tracker=tracker, controller=controller, mcu=mcu,
            log=log, machine=machine, calib=calib,
            loop_hz=cfg.loop_hz, run_timeout_s=args.duration,
            max_inference_ms=cfg.max_inference_ms, detector=detector)
        print("bench virtual : TANPA serial, TANPA hardware — roda fisik "
              "tidak bisa menyala dari sini")
        print(f"video         : {args.video} "
              f"({cfg.image_width}x{cfg.image_height} config, "
              f"{cam.source_fps:.1f} fps sumber)")
        print(f"mode          : {mcu.mode} (driver {cfg.motor_driver})")
        print(f"log           : {log.path}")
        dash = (serve_dashboard(cfg, args.dashboard_port)
                if args.dashboard_port is not None else None)
        stop = threading.Event()
        watcher = threading.Thread(target=_progress,
                                   args=(runner, mcu, stop), daemon=True)
        watcher.start()
        try:
            runner.run()
        finally:
            stop.set()
            if dash is not None:
                dash.shutdown()
                dash.server_close()

        _report(cfg, runner, mcu, args.video, cam.source_fps)
        summary = {
            "video": str(args.video),
            "video_fps": cam.source_fps,
            "duration_s": args.duration,
            "video_loop": not args.no_loop,
            "config": dataclasses.asdict(cfg),
            "runner": {
                "iterations": runner.iterations,
                "overruns": runner.overruns,
                "lane_lost": runner.lane_lost_count,
                "state": machine.state.name,
            },
            "mcu": mcu.summary(),
        }
        (log.path / "bench_summary.json").write_text(
            json.dumps(summary, indent=2, default=str), encoding="utf-8")
        print(f"\nLog: {log.path} (+ bench_summary.json)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
