# program/robot — runtime robot MIZURO (build baru)

Runtime robot KSTI 2026 yang dibangun dari nol, terpisah dari scaffold
`program/jetson/` (yang tetap ada sebagai referensi sampai migrasi
disetujui tim). Arsitektur dan aturan lomba: `../../docs/`.

## Status: Fase 0 — fondasi (tanpa hardware)

Sudah ada: config tervalidasi, state machine, protokol serial, log run,
self-check, dan test suite. Loop persepsi (kamera → lajur → kontrol → MCU)
datang di Fase 1–3; sampai itu ada, mode live sengaja ditolak.

## Menjalankan

Semua perintah dijalankan **dari folder ini** (`runs/` dan path config
relatif terhadap cwd):

```sh
# verifikasi fondasi (tanpa hardware, tanpa pyserial)
python3 main.py --self-check

# test suite (butuh pytest; install: pip install -r requirements-dev.txt)
python3 -m pytest tests/ -q

# boot kering: BOOT -> READY -> STOPPED, menulis log, roda tidak bergerak
cp robot.example.json robot.json   # lalu sesuaikan
python3 main.py --config robot.json
```

Di Jetson, pakai `/usr/bin/python3` (OpenCV JetPack + GStreamer). `.venv`
repo memakai opencv-python pip tanpa GStreamer dan hanya untuk skrip bench
`test-yolo*/`.

## Isi paket

| Modul | Peran |
|---|---|
| `mizuro/config.py` | dataclass config; key asing ditolak, rentang divalidasi |
| `mizuro/protocol.py` | protokol serial `speed_mps,steering_rad\n` + batas ±0.8/±0.55 |
| `mizuro/states.py` | BOOT/READY/RUNNING/DEGRADED/STOPPED/FAULT; FAULT terminal |
| `mizuro/types.py` | `Command` (muatan serial) dan `Observation` (hasil persepsi) |
| `mizuro/telemetry.py` | `runs/run-*/telemetry.csv` + `events.jsonl` |
| `mizuro/mcu.py` | tautan serial; `send()` = heartbeat; gagal tulis → `alive=False` |

## Peta fase

1. **Fase 0 — fondasi** (sesudah ini): config, state, protokol, log, test.
2. **Fase 1 — persepsi lajur**: LaneTracker CV klasik + evaluasi terhadap
   ground truth `program/arena-3d/`.
3. **Fase 2 — detektor semantik**: dataset dari sim → train YOLO-seg →
   decoder ONNX + logika checkpoint.
4. **Fase 3 — kontrol & closed-loop sim**: Stanley + speed profile,
   replay dataset, tuning hanya di sim.
5. **Fase 4 — MCU & safety**: spesifikasi firmware STM32, protokol status
   balik, timer 300 s, persistensi checkpoint.

## Keselamatan

- E-stop fisik memutus penggerak independen dari software; watchdog MCU
  (100 ms) lapis terakhir. Jangan matikan `dry_run` sebelum uji meja dengan
  roda terangkat.
- Saat lomba: tidak ada stream/dashboard/telemetri — log lokal saja.
- `stream_host` default `127.0.0.1`; `0.0.0.0` hanya untuk jaringan pit
  tepercaya.
