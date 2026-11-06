# Body controller (firmware MCU bodi) — v2

Firmware MCU untuk robot MIZURO KSTI 2026. Satu file sketsa
(`body_controller.ino`) melayani **tiga tahap output** yang dipilih saat
runtime lewat serial — tidak perlu flash ulang saat hardware berganti:

| mode    | hardware                                            | output |
|---------|-----------------------------------------------------|--------|
| `skid4` | **PROTOTYPE SEKARANG**: 4x motor DC lewat L298N, skid steer (tanpa servo) | 4x PWM+DIR, mixing `speed+steer` → roda kiri/kanan dilakukan **di MCU** |
| `esc1`  | **KOMPETISI**: 1x motor brushless lewat ESC + servo kemudi Ackermann | 1x servo-PWM (ESC) + 1x servo-PWM (kemudi) |
| `hb1`   | **BENCH**: 1x motor DC lewat driver PWM+DIR + servo kemudi | 1x PWM+DIR + 1x servo-PWM |

`DEFAULT_MODE` (compile-time) = `DriverMode::Skid4`, jadi kalau Jetson
belum mengirim apa pun pun firmware langsung dalam mode prototype.
Sumber kebenaran sebenarnya adalah `motor_driver` di `robot.json`
(`skid4_l298n` / `external_esc` / `mcu_hbridge`), yang diterjemahkan
`mizuro/protocol.py` → `#M,<mode>` dan dikirim saat port serial dibuka.

## Protokol (115200 baud, ASCII, satu baris per paket)

Jetson → MCU:

```
speed_mps,steering_rad\n    setpoint, 4 desimal, batas ±0.8 m/s / ±0.55 rad
#M,<mode>\n                 pilih mode: skid4 | esc1 | hb1
```

MCU → Jetson (10 Hz):

```
#v1,<mode>,<estop>,<stale>,<speed_out>,<steer_out>,<rx_ok>,<uptime_ms>\n
```

| field       | arti |
|-------------|------|
| `mode`      | mode output yang sedang aktif |
| `estop`     | 1 = tombol E-stop tertekan, penggerak dipotong |
| `stale`     | 1 = watchdog trip / belum ada setpoint segar |
| `speed_out` | kecepatan yang BENAR-BENAR diterapkan (m/s, 0 = dipotong) |
| `steer_out` | sudut kemudi yang diterapkan (rad, 0 = dipotong) |
| `rx_ok`     | jumlah setpoint valid yang diterima (uint32, wrap) |
| `uptime_ms` | `millis()` MCU |

Di mode `skid4`, `speed_out`/`steer_out` adalah perintah **sebelum**
mixing; kecepatan per-roda tidak dikirim (lihat "Yang belum ada").
Di-parse oleh `mizuro/protocol.py::parse_status_line()` dan dibaca
non-blocking oleh `mizuro/mcu.py::read_status()`.

**Batas protokol HARUS sama di dua sisi.** Ubah `MAX_SPEED_MPS`,
`MAX_STEER_RAD`, `WATCHDOG_MS`, atau baud = ubah `body_controller.ino`
DAN `mizuro/protocol.py` sekaligus.

## Kontrak keselamatan

1. Boot: semua output mati/netral sebelum ada setpoint valid.
2. Watchdog: tidak ada setpoint segar dalam `WATCHDOG_MS` (100 ms) →
   semua output mati. Loop Jetson WAJIB kirim setiap iterasi — itu
   heartbeat-nya.
3. E-stop (active-low, fisik) memotong output terlepas dari serial.
   Bukan pengganti pemutusan daya — lapisan terakhir saja.
4. Setpoint di luar batas / baris rusak **ditolak**; output terakhir
   yang valid tetap dipakai sampai watchdog trip.
5. Ganti mode selalu melewati `safeStop()` dulu.
6. Setelah E-stop, firmware butuh perintah BARU untuk jalan lagi
   (`stale=1` sampai saat itu) — tidak auto-resume.

## Pin map (PLACEHOLDER — cocokkan dengan wiring sebelum flashing)

Blackpill F401/F411 (STM32, `ARDUINO_ARCH_STM32`):

| pin    | fungsi |
|--------|--------|
| PB6/PB7 | motor depan KIRI: PWM / DIR |
| PB8/PB9 | motor depan KANAN: PWM / DIR |
| PB3/PB4 | motor belakang KIRI: PWM / DIR |
| PB5/PB10 | motor belakang KANAN: PWM / DIR |
| PB0/PB1 | mode `hb1`: motor tunggal PWM / DIR |
| PA0 | mode `esc1`: sinyal ESC (servo-PWM) |
| PA1 | mode `esc1`/`hb1`: sinyal servo kemudi |
| PB13 | E-stop (active-low, `INPUT_PULLUP`) |

Hinduki di Blackpill: PA9/PA10 (Serial), PA13/PA14 (SWD),
PA11/PA12/PA15 (USB), PC14/PC15 (oscilator). Cabang non-STM32
(Arduino Uno) ada hanya untuk cek sintaks — **bukan target**.

## Mixing skid steer (mode `skid4`)

`steering_rad` diperlakukan sebagai **yaw-rate ternormalisasi**: steer
penuh = `SKID_MAX_YAW_RATE` (2.0 rad/s). Beda kecepatan kiri/kanan =
`yaw * SKID_TRACK_M / 2`; `SKID_TRACK_M` (0.20 m) adalah jarak
kiri-kanan roda — **kalibrasi di sasis nyata**.

Kalau puncak kecepatan roda melebihi `MAX_SPEED_MPS`, kedua sisi
diskalakan bersama supaya rasio belok tetap (tidak pernah memotong satu
sisi saja). `speed=0` + steer penuh = putar di tempat (kiri mundur,
kanan maju).

## Kalibrasi

- **L298N**: arah motor bisa terbalik kabel — flip `MOTOR_DIR_FORWARD`
  kalau roda berputar salah arah. PWM 0-255 diskalakan linear dari
  `MAX_SPEED_MPS`; kalau kecepatan nyata tidak linear, kalibrasi
  mapping-nya (bukan menaikkan `MAX_SPEED_MPS`).
- **ESC**: `ESC_NEUTRAL_US` harus sama dengan titik netral ESC
  sungguhan (kalibrasi throttle di TX/ESC kalau perlu). Deadband
  `ESC_DEADBAND_US` (40 us) sengaja dijepit: pada gir 1:10, 1 m/s ≈
  1-3% throttle penuh = pulse 1515-1560 us, DI DALAM deadband ESC biasa
  — window maju karena itu dimulai dari netral+deadband supaya perintah
  kecil pun benar-benar menggerakkan roda.
- **Servo kemudi**: `STEER_CENTER_US`/`STEER_MIN_US`/`STEER_MAX_US`
  harus sama dengan mekanik kemudi (sudut maksimum yang tidak bikin
  roda gesek badan).

## Cara uji

### Tanpa hardware (host, g++)

Logika firmware yang sama berjalan di laptop lewat mock Arduino API:

```sh
cd program/body-mcu/test
./run_tests.sh          # g++ -std=c++17 -Wall -Wextra -Werror
```

120 check / 18 tes: parser ketat (sampah serial ditolak), watchdog,
e-stop, mixing skid + clamp rasio, pivot di tempat, pemetaan ESC/servo,
ganti mode, format baris status, formatter angka tanpa `printf %f`.

Yang TIDAK diuji di sini: timing hardware sungguhan (PWM 50 Hz servo,
arus L298N), kalibrasi netral ESC di ESC sungguhan, dan compile board.

### Compile board (Jetson — `arduino-cli` di repo ini aarch64-only)

```sh
# core AVR sudah terpasang; core STM32 BELUM — pasang sekali:
arduino-cli core install STMicroelectronics:stm32
# cek FQBN yang tersedia:
arduino-cli board listall | grep -i stm32

arduino-cli compile --fqbn arduino:avr:uno .                    # Uno (cek sintaks)
arduino-cli compile --fqbn STMicroelectronics:stm32:GenF1 .     # Blackpill (FQBN
# tergantung core stm32duino yang terpasang — sesuaikan setelah board listall)
```

Belum ada compile Blackpill yang terverifikasi di repo ini — lakukan di
Jetson sebelum flashing, dan cocokkan FQBN dengan board yang ada.

### Sebelum roda menyala

1. Roda terangkat dari lantai.
2. Sumber arus berbatas.
3. E-stop terbukti mematikan output motor (uji fisik, bukan logika).
4. `dry_run=true` dulu di `robot.json`, lihat log, baru `false`.

## Yang belum ada

- Kecepatan per-roda tidak dikirim balik ke Jetson (status hanya
  melaporkan perintah sebelum mixing).
- Protokol status MCU → Jetson belum ada feedback encoder/arus.
- `SKID_TRACK_M`, pin map, dan deadband ESC masih placeholder kalibrasi.
- Compile Blackpill (stm32duino) belum diverifikasi.
