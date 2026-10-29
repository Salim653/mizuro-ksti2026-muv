# program/robot — runtime robot MIZURO (build baru)

Runtime robot KSTI 2026 yang dibangun dari nol, terpisah dari scaffold
`program/jetson/` (yang tetap ada sebagai referensi sampai migrasi
disetujui tim). Arsitektur dan aturan lomba: `../../docs/`.

## Status: Fase 1 — persepsi lajur (inti selesai, bench-only)

Fase 0 (fondasi) selesai. Fase 1: `LaneTracker` CV klasik sudah jalan
end-to-end dan dievaluasi terhadap ground truth sintetis eksak — median
error lateral **0.73 mm**, heading **0.11°** pada 100 frame acak (target
Fase 1: < 3 cm / < 5°). Loop persepsi (kamera → lajur → kontrol → MCU)
masih menunggu Fase 2–3; mode live tetap ditolak sampai controller ada.

## Menjalankan

Semua perintah dijalankan **dari folder ini** (`runs/` dan path config
relatif terhadap cwd):

```sh
# verifikasi fondasi (tanpa hardware, tanpa pyserial)
python3 main.py --self-check

# test suite (butuh pytest; install: pip install -r requirements-dev.txt)
python3 -m pytest tests/ -q

# eval persepsi — sintetis (tanpa hardware):
python3 tools/eval_perception.py --frames 100

# eval persepsi — dataset asli arena-3d (zip export):
python3 tools/eval_perception.py --dataset /path/ke/laps.zip

# boot kering: BOOT -> READY -> STOPPED, menulis log, roda tidak bergerak
cp robot.example.json robot.json   # lalu sesuaikan
python3 main.py --config robot.json
```

Di Jetson, pakai `/usr/bin/python3` (OpenCV JetPack + GStreamer). `.venv`
repo memakai opencv-python pip dan hanya untuk skrip bench `test-yolo*/`
serta test suite repo ini.

## Isi paket

| Modul | Peran |
|---|---|
| `mizuro/config.py` | dataclass config; key asing ditolak, rentang divalidasi |
| `mizuro/protocol.py` | protokol serial `speed_mps,steering_rad\n` + batas ±0.8/±0.55 |
| `mizuro/states.py` | BOOT/READY/RUNNING/DEGRADED/STOPPED/FAULT; FAULT terminal |
| `mizuro/types.py` | `Command` (muatan serial) dan `Observation` (hasil persepsi) |
| `mizuro/telemetry.py` | `runs/run-*/telemetry.csv` + `events.jsonl` |
| `mizuro/mcu.py` | tautan serial; `send()` = heartbeat; gagal tulis → `alive=False` |
| `mizuro/camera.py` | CSI/GStreamer; gagal keras kalau cv2 tanpa GStreamer |
| `mizuro/stream.py` | MJPEG bench-only: FrameHub encode-once + server HTTP |
| `mizuro/perception/calibration.py` | piksel ↔ tanah (konvensi ray arena-3d), homography |
| `mizuro/perception/geometry.py` | fit kuadratik robust + bobot, state lajur, confidence |
| `mizuro/perception/classical.py` | `LaneTracker`: Otsu → stasiun → fit, dua pass |
| `mizuro/perception/synth.py` | generator frame sintetis + ground truth eksak |

### Catatan persepsi (pelajaran yang mahal)

- **Urutan argumen** `ground_to_pixel(x_forward, y_right)` — tertukar di
  beberapa call site dulu dan jadi sumber bug berulang. Titik tanah di
  seluruh paket: `(forward, right)`.
- **adaptiveThreshold hanya melihat TRANSISI**: interior marka lebar =
  mean lokal = tidak terdeteksi. Otsu global dipakai instead (arena-3d
  membuktikan brightness threshold saja ~1 mm dari oracle).
- **Stasiun ambigu tidak boleh ditebak di pass 1**: satu run bisa tepi,
  dash, atau noise. Pass 1 hanya menerima stasiun unambiguous (≥3 run →
  dash tengah; 2 run ~0.5 m → dua tepi); stasiun ambigu baru dipakai
  prediksi di pass 2. Tanpa ini, bias ±0.25 m (satu setengah lebar lajur)
  mengunci diri lewat prediksi.
- **`forward_range()` dekat horizon meledak** (baris nyaris-horizon →
  jarak puluhan meter). Selalu kirim `roi_top`; hasil di-clamp 12 m.
- **opencv-python 4.8 headless**: warna skalar + `LINE_AA` di `fillPoly`
  menggambar NOL piksel — selalu pakai tuple BGR.
- Mount 0.16 m / 12° hanya melihat lajur dari ~0.5 m di depan; heading
  adalah besaran lookahead. Truth dataset dipindah ke stasiun yang sama
  dengan fit sebelum dibandingkan (`tools/eval_perception.py`).

## Peta fase

1. **Fase 0 — fondasi** ✅: config, state, protokol, log, test.
2. **Fase 1 — persepsi lajur** (inti selesai): LaneTracker + eval sintetis;
   tinggal validasi bench dengan kamera asli (roda terangkat, `dry_run`).
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
