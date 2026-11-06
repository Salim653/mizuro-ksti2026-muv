#!/bin/sh
# Tes host untuk body_controller.ino — tanpa hardware, tanpa Arduino IDE.
#
# Mock Arduino API (mock_arduino.h/mock_servo.h) mengganti library board,
# jadi logika firmware yang sama berjalan di laptop lewat g++.
#
# Untuk cek sintaks/build board sungguhan (jalankan di Jetson):
#   arduino-cli compile --fqbn arduino:avr:uno .                 # Uno
#   arduino-cli compile --fqbn STMicroelectronics:stm32:GenF1 .  # Blackpill
# (FQBN STM32 tergantung core stm32duino yang terpasang — cek dengan
#  `arduino-cli board listall`.)
set -eu
cd "$(dirname "$0")"

CXX="${CXX:-g++}"
OUT="${TMPDIR:-/tmp}/mizuro_body_controller_test"

echo "compile: $CXX -std=c++17 -Wall -Wextra -Werror -I . test_body_controller.cpp"
# shellcheck disable=SC2086
$CXX -std=c++17 -Wall -Wextra -Werror -I . -o "$OUT" test_body_controller.cpp

echo "run: $OUT"
exec "$OUT"
