# AGENTS.md — MIZURO KSTI 2026 Mini Autonomous Vehicle

Autonomous robot for team MIZURO at KSTI 2026 (ITS Surabaya, November 12, 2026). Hardware: Jetson Orin Nano plus a body MCU. Lane keeping is camera-based.

All repository documents are written in Indonesian. Follow that convention. The files in `docs/project-notes/` are in English, but they are working notes, not final specifications.

**Read first:** `docs/KSTI-2026-MAV-GUIDEBOOK-RECAP.md` (rules, track, scoring) and `docs/PANDUAN-LANJUTAN-TIM.md` (handoff status, integration order, safety limits).

**Rules that shape the code:**
- Infrared and line-follower sensors are banned. Camera vision is mandatory.
- All communication is prohibited during a run. The robot must be standalone and log locally.
- The run timer is 300 seconds.
- Checkpoints are the dominant scoring factor. Reliability matters more than speed.

## Code map: which files are canonical

- `program/jetson/`: the canonical Python robot runtime. Entry point is `main.py`, with flags `--self-check`, `--stream-only`, and `--config PATH`.
- `program/robot-fw/`: an old English scaffold copy. Its files overlap with `jetson/` and are NOT guaranteed to be in sync. `requirements.txt` and `requirements-jetson.txt` exist only here.
- `program/body-mcu/body_controller.ino`: MCU firmware for Arduino-compatible boards. The Uno is only for syntax checking. The real target is the STM32F401/F411 Blackpill, per `docs/project-notes/`.
- `program/arena-3d/`: a 3D arena simulator that runs in the browser. Open `index.html` directly, no server needed. It also exports datasets with ground truth.
- `program/sim-viz/server_viz.py`: ONNX inference server (default port 8766, endpoint `POST /predict`).
- `program/test/`: OUTDATED prototype scripts from the ESP32 era. They use the `S0,V0` protocol and `F/B/L/R/S` characters on `/dev/ttyTHS1`. This is NOT the current robot protocol. Do not use them as a reference.
- `test-yolo/` and `test-yolo-seg/`: ultralytics bench scripts (bottle follower and sim video replay). Their serial protocol is `M <left> <right>\n` with PWM values from 0 to 255, sent to an ESP32 on `/dev/ttyCH341USB0` or `/dev/ttyUSB0`. This is DIFFERENT from the robot protocol. Run them with the repo's `.venv`, for example `../.venv/bin/python test_botol_web.py --no-motor` (web page on port 8091). `test-yolo-seg/test_botol_seg.py` uses port 8090, and `lane_replay.py` uses port 8092.
- `bin/arduino-cli`: a static aarch64 binary. It runs only on the Jetson, not on x86 laptops.
- `electronic/` and `mechanical/`: wiring and design notes. The CAD source files have not been copied yet and still live in the old workspace.

## Verification (no hardware needed)

This repo has no pytest, linter, or CI. Verification means running the self-check and compile steps. Always run them from `program/jetson`, because the `runs/` path and config paths are relative to the working directory:

```sh
cd program/jetson
/usr/bin/python3 main.py --self-check
/usr/bin/python3 dashboard.py --self-check
/usr/bin/python3 -m py_compile main.py camera.py core.py stream.py vision.py controller.py mcu.py telemetry.py
```

## Interpreter and camera (Jetson): the biggest pitfalls

- The runtime MUST use `/usr/bin/python3`. It has the JetPack OpenCV build with GStreamer. The repo's `.venv` uses the pip `opencv-python` package, which has NO GStreamer, so `camera.py` fails with an explicit error message. Never run `pip install opencv-python` on the Jetson. `requirements-jetson.txt` explains why.
- To check the backend, run `python3 -c "import cv2; print(cv2.getBuildInformation())"`. The output must include `GStreamer: YES`.
- Only ONE process may hold the CSI camera at a time. `--stream-only` opens the camera without arming the robot. The endpoints `/`, `/stream`, and `/snapshot` are served on `stream_host:stream_port` (default `0.0.0.0:8080`).
- `camera.py` rejects any frame whose size differs from `image_width` and `image_height` in the config.
- `.pt` model files are gitignored. Bench scripts expect the file in their own folder. Copies exist at the repo root.

## Binding protocol and config

- Serial from Jetson to MCU: ASCII `speed_mps,steering_rad\n`, 4 decimal places, 115200 baud. Hard limits are ±0.8 m/s and ±0.55 rad. `format_command` raises an error outside these ranges. Changing the limits means changing the protocol on both sides and the firmware.
- `Config.load()` rejects unknown keys and validates ranges. Live mode requires `camera_pipeline`, `model_path`, and `serial_device` to be set. `dry_run` defaults to `true`.
- The control loop targets `loop_hz` (default 60 Hz). Overruns are logged as `loop_overrun` events. The camera cannot be forced to run that fast.
- Run logs go to `runs/run-<timestamp>/telemetry.csv` and `events.jsonl` (gitignored). The dashboard reads these logs: `dashboard.py --runs PATH`, default `127.0.0.1:18765`.

## Perception status: do not expect the robot to move yet

`vision.py` is intentionally a placeholder. `OnnxDetector.infer` only returns `raw_shape`. `LaneEstimator` returns confidence 0, so `Controller` always sends speed 0. The next core tasks are writing the ONNX decoder and the lane estimator. Validate them against ground truth in `program/arena-3d/` before connecting motors.

## Safety (competition rules, not preferences)

- A physical E-stop cuts drive power independently of the Jetson. The MCU watchdog (100 ms) is the last line of defense. Never disable `dry_run` or connect the drive motors before bench testing with the wheels lifted off the ground.
- During a competition attempt there must be NO communication. Stop the stream, dashboard, and all HTTP servers before each attempt.
- `dashboard.py --host 0.0.0.0` is only for trusted pit networks. It has no authentication.

## Git and working tree

- The working tree often holds large staged changes. As of October 29, 2026, 256 files are uncommitted, including `program/jetson/robot.json`, the `bin/arduino-cli` binary, and about 240 dataset frames. Check `git status` before editing. Do not run `commit`, `reset`, `clean`, or `pull` without team approval.
- `robot.json` is the local working config. Section 8 of `PANDUAN-LANJUTAN-TIM.md` forbids committing it. Confirm before changing its staging status.
- Keep one change per branch or PR. Include how it was tested and what has NOT been tested yet.

## Simulation, datasets, and bench

- `program/arena-3d/`: open `index.html` directly (file://). `track_floor_data.js` is an inlined texture. Regenerate it with `python tools/inline_texture.py` after the map changes. Rebuild the geometry with `python extract_track.py`, which needs `opencv-python`, `numpy`, and `pymupdf`.
- The baseline loader `code/baseline_line_follower.py` (inside the sim ZIP export) needs `numpy` and `opencv-python-headless`.
- `program/sim-viz/server_viz.py`: run `KSTI_BACKEND=cuda python server_viz.py` on the Jetson. It needs onnxruntime-gpu from the Jetson AI Lab index (`jp6/cu126`, host `pypi.jetson-ai-lab.io`). The `.dev` host is no longer working.
- `test-yolo-seg/lane_replay.py` needs the sim-viz server running in another terminal. Motors are gated on the page behind an "Enable motors" control. Lift the wheels first.