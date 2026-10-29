# Changelog — program/robot

Riwayat paket runtime robot (`program/robot/`). Scaffold lama
(`program/jetson/`) punya riwayat commit sendiri dan tidak disentuh sejak
build baru dimulai. Format: satu entri per commit besar, plus catatan
verifikasi dan yang BELUM diuji.

## 2026-10-29 — Fase 1: persepsi lajur, kamera, stream, eval

### Persepsi (`mizuro/perception/`)
- `calibration.py` — proyeksi piksel ↔ tanah dengan konvensi ray arena-3d
  (X maju, Y kanan, Z bawah), homography batch, `forward_range()`
  ter-clamp 12 m. Tanpa clamp, baris nyaris-horizon memetakan ke jarak
  puluhan meter dan merusak segmen render.
- `geometry.py` — fit polinomial kuadratik robust dengan bobot (stasiun
  dekat lebih reliabel per piksel), `LaneFit.lateral_at/slope_at/
  curvature_inv_at`, state lajur, confidence dari coverage × kualitas fit.
- `classical.py` — `LaneTracker`: grayscale → ROI → binarisasi Otsu
  global → sampling stasiun dua pass → fit robust. Otsu dipakai, bukan
  adaptiveThreshold: adaptive hanya menandai TRANSISI, interior marka
  lebar = mean lokal = tidak terdeteksi. Pass 1 hanya menerima stasiun
  unambiguous (≥3 run → run tengah = dash; 2 run berjarak ~0.5 m → dua
  tepi, pusat = midpoint). Stasiun ambigu (1 run, atau 2 run ~0.25 m =
  tepi + dash) baru resolve pakai prediksi di pass 2 — tanpa aturan ini,
  bias ±0.25 m (setengah lebar lajur) mengunci diri lewat prediksi
  lintasan.
- `synth.py` — generator frame sintetis + ground truth eksak
  (`render_lane_frame`, `random_truth`, `LaneTruth`). Catatan opencv:
  warna skalar + `LINE_AA` di `fillPoly` menggambar NOL piksel pada
  opencv-python 4.8 headless — selalu tuple BGR.

### Kamera & stream
- `mizuro/camera.py` — CSI/GStreamer (`nvargus_pipeline`). Gagal keras
  kalau cv2 tanpa backend GStreamer, dengan pesan eksplisit memakai
  `/usr/bin/python3` (JetPack). Frame dengan ukuran beda dari kalibrasi
  ditolak, bukan diterima diam-diam. `opener` bisa disuntikkan untuk tes.
- `mizuro/stream.py` — `FrameHub`: encode sekali per frame, throttle ke
  `stream_fps`, skip total kalau tidak ada penonton (viewer tidak boleh
  memperlambat loop 60 Hz), `publish()` menangkap semua exception encode
  supaya satu frame buruk tidak membunuh loop kontrol. `MjpegServer`:
  `/`, `/stream`, `/snapshot`, default localhost. Bench only — wajib
  mati selama kompetisi.

### Eval & tes
- `tools/eval_perception.py` — mode sintetis (tanpa hardware) dan mode
  `--dataset` (zip export arena-3d). Mode dataset memindahkan truth ke
  stasiun yang sama dengan fit: mount 0.16 m hanya melihat lajur dari
  ~0.5 m di depan, jadi membandingkan estimasi stasiun 0.6 m dengan
  truth di poros mengukur jarak stasiun, bukan detector.
- 9 file tes, 140 tes lolos: round-trip proyeksi, logika stasiun
  (3-run → dash, 2-tepi → midpoint, 1-run dilewati di pass 1), paint
  synth landing di posisi terproyeksi, kamera + stream tanpa hardware
  (opener suntik, server di port ephemeral).

### Fase 0 — patch review (ikut masuk di commit ini)
- `Config._check_types()`: tipe tiap field diperiksa eksplisit.
- `protocol._fixed()`: normalisasi negatif-nol supaya tidak pernah
  muncul `-0.0000` di serial.
- `require()` tahan `python3 -O` (assert tidak dipakai untuk validasi).
- Tes sinkronisasi `robot.example.json` dengan default `Config`.
- Kasus tipe ditambah di tes config/protocol.

### Hasil
- Eval 100 frame sintetis: median error lateral **0.73 mm**, heading
  **0.11°** (target Fase 1: < 3 cm / < 5°). Satu frame no-lane: lajur
  keluar frame penuh di tikungan R≈2.4 m — jujur, bukan bias.

### Verifikasi
```sh
cd program/robot
../../.venv/bin/python -m pytest tests/ -q                        # 140 passed
../../.venv/bin/python main.py --self-check                       # lolos
../../.venv/bin/python -O main.py --self-check                    # lolos
../../.venv/bin/python tools/eval_perception.py --frames 100      # LOLOS
```

### Belum diuji
- Bench dengan kamera CSI asli (roda terangkat, `dry_run`,
  `--stream-only`) — belum ada hardware di meja.
- `tools/eval_perception.py --dataset` — kode ada, belum pernah jalan
  pada zip asli karena belum ada dataset diekspor dari arena-3d di repo.
