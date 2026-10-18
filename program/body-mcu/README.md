# Arduino-compatible body controller

Open `body_controller/body_controller.ino` in Arduino IDE or run `arduino-cli compile --fqbn arduino:avr:uno body_controller` from this folder. It reads `speed_mps,steering_rad\n` over serial at 115200 baud, has an MCU-side stale-command stop, and reads an active-low e-stop input. The Uno target is only a syntax/build check; confirm your intended MCU and pin mapping before flashing.

Pin assignments are placeholders. Match the board and driver datasheets before wiring. The sketch uses generic PWM + direction output for an H-bridge. For the current planned 540 motor + RC ESC hardware, use the ESC's supported servo-PWM protocol instead of connecting its signal input to a DC motor driver's PWM/DIR pins. Keep the Jetson isolated from motor current and drive power.

Before testing: motor wheels lifted, current-limited supply, verified e-stop cuts motor output, and no unattended run. Speed-to-PWM mapping and steering centre/endpoints need calibration on the actual chassis. Encoder/IMU sampling and MCU status telemetry are not included yet.
