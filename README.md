# MIZURO KSTI 2026 — Mini Autonomous Vehicle

Repository for the KSTI 2026 MUV robot. The existing workspace is left untouched; this repository contains working copies organized by discipline.

## Repository layout

- `electronic/` — electronics architecture, wiring, power, and safety interfaces.
- `program/jetson/` — Jetson Orin Nano runtime: camera, perception, controller, telemetry.
- `program/body-mcu/` — body-MCU firmware and its interface with the Jetson.
- `mechanical/` — chassis, drivetrain, steering, camera mounts, and CAD exports.
- `docs/` — guidebook, decisions, integration and test records.

## Intended robot operating sequence

1. Power-up leaves motor output disabled. Jetson and body MCU boot; MCU holds motor output at zero.
2. Operator presses the required physical **start** button. MCU accepts the start only when e-stop is released and the Jetson/runtime is ready. Navigation then runs standalone; no tether or telemetry is used during an attempt.
3. While navigating, camera-based lane estimates steer the car. Loss of valid perception requests zero drive; the independent MCU command watchdog stops drive if valid setpoints cease.
4. Physical **emergency stop** cuts drive independently of Jetson software. It latches until deliberately reset.
5. For a referee-directed restart, position the car at the last checkpoint, reset the e-stop as directed, and press start again. Navigation resumes from the checkpoint index retained for that attempt.
6. Stop autonomously at the 300-second attempt limit, on confirmed completion, on a latched safety fault, or when a physical e-stop is pressed.

## Current implementation status

The copied `program/` files are a prototype snapshot, not a competition-ready implementation. The current Python runtime uses a console Enter gate; the copied MCU sketch is Arduino-compatible and has no physical start input, checkpoint state, or run timer. The project decision records STM32F401/F411 Blackpill as the intended body MCU, so that firmware and the serial protocol must be reconciled before hardware integration. Do not treat the copied prototype sketch as validated or flash it into the competition robot without port, wiring, and safety review.

## Safety requirements

- E-stop must physically inhibit motor/ESC drive independently of the Jetson and be readily reachable.
- MCU watchdog must default to zero drive on reset, malformed command, stale command, or MCU fault.
- Do not start with wheels on the ground until e-stop, watchdog, motor polarity, steering limits, and command-loss behavior are bench-tested with drive wheels raised.
- No wireless or wired communication during a competition attempt.

## First development target

Implement and test the body-MCU state machine and interlock first: `BOOT_SAFE → READY → RUNNING`, with e-stop/timeout/fault taking precedence and requiring a deliberate physical restart. Then add Jetson-ready handshake, 300-second cutoff, checkpoint persistence/resume, and finish detection. Keep the runtime testable without hardware.
