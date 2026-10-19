# Robot runtime scaffold

Jetson Orin Nano owns image capture, ONNX inference, lane geometry, planning and run logs. A body MCU owns PWM, encoder/IMU sampling, start/e-stop input and the setpoint watchdog. The runtime must never replace that watchdog.

## Safe first run

```sh
python3 main.py --self-check
python3 main.py --config robot.example.json
```

The example config runs dry. It creates a timestamped run directory with `telemetry.csv` and `events.jsonl`, and sends no actuator commands. Copy it to `robot.json` before setting values. Do not switch `dry_run` off until camera, trained model, start input, MCU protocol, e-stop behavior and watchdog have been bench-tested with wheels raised.

## Layout

- `main.py`: launch, state orchestration and loop.
- `core.py`: config, command/observation data and state machine.
- `camera.py`: CSI/GStreamer capture.
- `vision.py`: ONNX inference and lane-estimation seam.
- `controller.py`: lane-to-steering command seam.
- `mcu.py`: serial command driver. Protocol is ASCII `speed_mps,steering_rad\\n`, both decimal, fixed to four places; range is ±0.8 m/s and ±0.55 rad at 115200 baud.
- Main loop runs at configured `loop_hz` (default 60 Hz), records overrun events when one cycle exceeds its period, and cannot force the CSI camera to produce frames at that rate.
- `telemetry.py`: local run logs.
- `dashboard.py` + `dashboard.html`: optional read-only dashboard on `127.0.0.1:18765` by default. From `robot-fw/`, run `python3 dashboard.py`; point at another output directory with `--runs PATH`. For a trusted pit LAN only, pass `--host 0.0.0.0` and open `http://<jetson-ip>:18765/` from another device. Toggle **Show clearly labeled simulated telemetry** to preview fabricated sample values; they come from `/api/demo` and are never written into run logs. Enter the camera streamer URL (Jetson USB-network default: `http://192.168.55.1:8080/stream`) and connect to view MJPEG. The feed is served by a separate HTTP service and may be blocked by browser mixed-content or network-origin policy when the dashboard is HTTPS or another device cannot reach the Jetson camera endpoint. The dashboard and camera feed are bench/pit only; stop them before competition attempts.
- `arduino/body_controller/`: Arduino-compatible motor/steering MCU sketch.

## Runtime seams

- `Camera` opens the configured Jetson GStreamer pipeline through OpenCV and rejects unexpected frame sizes.
- `OnnxDetector` loads ONNX through OpenCV DNN, but currently only reports output shape. It does not decode detections; live commands stay at zero until the model-specific decoder and lane estimator return valid geometry.
- `LaneEstimator` still needs the calibrated lane/dash detector and ground projection. Validate estimates against simulator dataset ground truth before connecting a motor.
- `Controller` sends zero speed while the lane estimate is missing. Its steering law is a starting point, not tuned vehicle control.
- `McuLink` sends exactly `speed_mps,steering_rad\\n`; the Arduino sketch uses that same format. `motor_driver` is recorded configuration for the MCU sketch selection, not additional packet data.
- `StartGate` is a console Enter gate for bench runs. The physical start-button adapter and MCU e-stop status return are not yet wired; MCU e-stop input must still cut drive independently.
- `max_inference_ms` and `lane_lost_frames` request a zero command / DEGRADED state when vision is late or absent. MCU watchdog remains the last line of defense.

## Arduino folder

`arduino/body_controller.ino` is a pin-configurable Arduino-compatible body-controller example. It outputs PWM + direction for a driver such as a BTS7960-style H-bridge, or a servo-PWM signal to an external ESC when wired accordingly. Verify the exact driver, PWM frequency/current limits, steering pulse range, board timers, and pin capabilities before use. The pin map and speed-to-PWM conversion require calibration for the actual motor and driver. It does not implement encoder/IMU feedback or parse MCU telemetry back to Jetson.

## First integration order

1. Confirm carrier and JetPack, then test CSI capture with the chassis lifted.
2. Implement the real ONNX output decoder, then lane geometry and dataset comparison.
3. Agree MCU packet fields and units; the current shared packet is two comma-separated floats.
4. Verify driver pin wiring, e-stop wiring, watchdog and steering calibration with the drive wheels lifted.
5. Wire the physical start input. Do not rely on the console gate for a competition deployment.

No wireless or tethered telemetry should run during competition; store logs locally. Stop the dashboard before any attempt. The HTTP server binds to localhost by default; exposing it with `--host 0.0.0.0` is only for a trusted pit network and serves run details without authentication. This framework is not competition-ready until the model decoder, lane estimator, MCU status/fault reporting, physical start/e-stop integration and hardware calibration are finished.
