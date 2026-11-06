# program/robot — runtime robot MIZURO (build baru)

Runtime robot KSTI 2026 yang dibangun dari nol, terpisah dari scaffold
`program/jetson/` (yang tetap ada sebagai referensi sampai migrasi
disetujui tim). Arsitektur dan aturan lomba: `../../docs/`.

## Status: Fase 4 — closed-loop runner selesai (bench-only)

Fase 0 (fondasi), Fase 1 (persepsi lajur), Fase 2 (detektor semantik),
Fase 3 (controller Stanley), dan Fase 4 (closed-loop runner) selesai.
Tracker dievaluasi terhadap ground truth sintetis eksak — median error
lateral **0.73 mm**, heading **0.11°** pada 100 frame acak (target
Fase 1: < 3 cm / < 5°). Stanley replay sim converge ke **3.1 mm /
1.47°** steady-state dari offset 0.2 m + heading 20°.

Motor **belum pernah menyala** di build ini. Mode live butuh
`dry_run=false` + roda terangkat + E-stop fisik.

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

# replay controller Stanley di sim (tanpa hardware, tanpa motor):
python3 tools/replay_control.py --offset 0.2 --heading 20

# rekam dataset: frame mentah + index.csv timestamped (Jetson):
/usr/bin/python3 tools/record_dataset.py --frames 200

# boot kering: BOOT -> READY -> STOPPED, menulis log, roda tidak bergerak
cp robot.example.json robot.json   # lalu sesuaikan
python3 main.py --config robot.json

# dashboard run (bench/pit): baca runs/run-*/ di browser
python3 dashboard.py --runs runs --port 18765
```

Di Jetson, pakai `/usr/bin/python3` (OpenCV JetPack + GStreamer). `.venv`
repo memakai opencv-python pip dan hanya untuk skrip bench `test-yolo*/`
serta test suite repo ini.

## Flag main.py

| Flag | Kegunaan | Hardware | Motor |
|---|---|---|---|
| `--self-check` | Verifikasi fondasi: config, state machine, protokol, MCU stub | Tidak ada | Tidak |
| `--stream-only` | Bench persepsi: kamera CSI + MJPEG (`:8080/stream`) + tracker lajur | Kamera | Tidak |
| `--closed-loop` | Loop kontrol: kamera → tracker → Stanley → MCU. `dry_run` default true = log saja, serial tidak disentuh | Kamera (+ MCU kalau live) | Hanya kalau `dry_run=false` |
| `--dashboard` | Bench/pit: serve dashboard run (`:18765`) selama mode berjalan. Butuh `--stream-only`/`--closed-loop` | Tidak | Tidak |
| `--dashboard-port` | Port dashboard (default 18765) | — | — |
| `--config PATH` | Tunjuk file config (default `robot.json`) | — | — |

Tanpa flag → `boot_dry` (heartbeat nol, langsung exit; sisa Fase 0).

```sh
# Bench kamera + tracker (Jetson):
/usr/bin/python3 -u main.py --stream-only

# Closed-loop dry-run — loop penuh, serial TIDAK disentuh:
/usr/bin/python3 -u main.py --closed-loop

# LIVE (MOTOR NYALA) — syarat: dry_run=false di config + roda
# terangkat + E-stop fisik + arus terbatas:
/usr/bin/python3 -u main.py --closed-loop --config robot.json
```

## Dashboard dari main.py (`--dashboard`)

Daripada jalanin `dashboard.py` terpisah, flag `--dashboard` menyalakan
server dashboard di thread daemon **selama mode berjalan** — membaca
log run yang sedang ditulis proses ini, jadi telemetry terlihat hidup
di browser:

```sh
# bench kamera + tracker + dashboard (URL MJPEG otomatis terisi):
/usr/bin/python3 -u main.py --stream-only --dashboard
# → stream   : http://127.0.0.1:8080/
# → dashboard: http://127.0.0.1:18765/  (panel kamera sudah nyambung)

# closed-loop dry-run + dashboard (lihat command Stanley hidup):
/usr/bin/python3 -u main.py --closed-loop --dashboard
```

- Default **OFF** — saat lomba semua HTTP wajib mati; flag eksplisit
  supaya tidak pernah nyala tidak sengaja.
- Butuh `--stream-only` atau `--closed-loop` (dashboard membaca log
  yang ditulis mode tersebut); sendirian = error.
- URL MJPEG di-inject ke halaman (panel kamera langsung terisi, tidak
  perlu ketik manual) — di-escape dulu sebelum masuk literal JS.
- Server berhenti bersih saat mode selesai/Ctrl+C (`finally`).
- Port bisa diubah: `--dashboard-port 18766`.

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
| `dashboard.py` + `dashboard.html` | dashboard run read-only (bench/pit); HTML terpisah supaya bisa diedit tim |

### Catatan persepsi (pelajaran yang mahal)

- **Mount kamera = bagian dari kalibrasi, bukan afterthought**: offset
  x/y, yaw, roll semuanya menggeser estimasi lajur (bias = offset mount,
  persis). Semua bisa di-set di `robot.json` (field `mount_*`, satuan
  meter/derajat); default 0 = kamera tepat di titik referensi robot.
  Output persepsi selalu di frame robot, siap dipakai controller.
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
- **Intrinsik default = tebakan hFOV 62°**, bukan ukuran. Untuk akurasi
  tinggi: cetak checkerboard (hitam-di-putih, 9x6 sudut, kotak 25 mm),
  jalankan `tools/calibrate.py` di Jetson, lalu set `homography_path`
  di `robot.json` ke JSON hasilnya. `--stream-only` otomatis memakai
  intrinsik terukur + undistort frame. Height/pitch TIDAK diukur alat
  kalibrasi — itu tetap pengukuran fisik (field `mount_*`).
- **findChessboardCorners butuh kotak GELAP di background TERANG** —
  pola terbalik gagal total (tes sintetis membuktikan). Cetakan
  standar hitam-di-putih aman.

## Dashboard run (`dashboard.py`)

Pembaca log lokal — menampilkan `runs/run-*/telemetry.csv` +
`events.jsonl` di browser: kecepatan, command kemudi, error lajur,
confidence, state machine, event runtime, dan grafik deret waktu.
Stdlib only (tanpa dependensi), jadi jalan dengan `/usr/bin/python3`
di Jetson apa adanya.

```sh
# dari folder program/robot (path runs/ relatif terhadap cwd):
python3 dashboard.py --runs runs --port 18765

# verifikasi tanpa hardware (tes baca log + serve HTTP beneran):
python3 dashboard.py --self-check
```

| Flag | Default | Kegunaan |
|---|---|---|
| `--runs PATH` | `runs` | direktori log runtime (sama seperti `output_dir` di config) |
| `--host` | `127.0.0.1` | `0.0.0.0` hanya untuk jaringan pit tepercaya (tidak ada autentikasi) |
| `--port` | `18765` | port HTTP |
| `--self-check` | — | verifikasi stdlib-only, tanpa hardware |

Endpoint JSON (read-only): `/api/status` (run terbaru atau `?run=`),
`/api/runs` (daftar run), `/api/history?points=` (deret waktu buat
grafik), `/api/demo` (data sintetis buat preview UI).

Fitur halaman (`dashboard.html` — file terpisah, bisa diedit anggota
tim tanpa menyentuh Python):

- pemilih run (terbaru atau run lama buat analisis setelah lomba)
- grafik canvas: kecepatan (m/s) + error lateral (mm) terhadap waktu,
  downsampled server-side (240 titik dari 18.000 baris)
- preview kamera MJPEG opsional (URL disimpan di localStorage) —
  cocok dengan `main.py --stream-only` (`http://<ip>:8080/stream`)
- mode simulasi: data sintetis buat cek UI tanpa robot
- indikator freshness: log dianggap hidup kalau mtime < 3 detik

Keamanan: hanya nama event yang dikirim (detail event bisa berisi
config — tidak pernah diekspos); nama run dari query divalidasi
regex (path traversal ditolak, HTTP 400); CSP + `no-store` +
`nosniff` di setiap respons.

**Aturan lomba: matikan dashboard sebelum setiap percobaan** —
tidak ada komunikasi apa pun dengan robot selama run.

## Deteksi checkpoint (`checkpoint_mode`)

Detektor checkpoint **tidak mengontrol robot** — hasilnya masuk log
(`Observation.detections` / telemetry), lane keeping tetap classical
CV. Backend dipilih lewat config:

| Mode | Backend | Butuh | Kecepatan |
|---|---|---|---|
| `none` (default) | mati | — | — |
| `cv` | `CheckpointDetectorCV` — threshold HSV + morphology + filter kontur | nol training | ~5 ms |
| `model` | `SemanticDetector` — YOLO ONNX (cv2.dnn) | model terlatih kelas arena | ~270 ms (CPU) |

```json
{
  "checkpoint_mode": "cv",
  "checkpoint_color": "green",
  "checkpoint_hsv_low": [],
  "checkpoint_hsv_high": [],
  "checkpoint_min_area_px": 300,
  "checkpoint_max_area_px": 400000,
  "checkpoint_min_solidity": 0.3,
  "checkpoint_label": "checkpoint"
}
```

- `checkpoint_hsv_low/high` kosong = pakai preset warna
  (`checkpoint_color`: green/red/orange/yellow/blue). Diisi tuple 3
  angka `(H, S, V)` untuk rentang eksplisit.
- Mode `model` mensyaratkan `model_path` (divalidasi saat load);
  mode `cv`/`none` jalan tanpa model.
- Detektor CV melaporkan "ada blob warna target di sini" — tidak
  bisa bedakan checkpoint dari benda warna sama. Kalau arena tidak
  konsisten warnanya, pakai mode `model` setelah ada dataset asli.

### Ngumpulkan dataset (mode `model`)

```sh
# 1. Rekam frame mentah dari kamera robot (headless, preview browser)
/usr/bin/python3 tools/record_dataset.py --frames 300

# 2. Output: runs/dataset-<timestamp>/
#      frames/frame-000001.png ... + index.csv (timestamp per frame)

# 3. Label (di laptop): LabelImg / Roboflow / labelStudio

# 4. Train + export (model disimpan di models/, lihat models/README.md):
.venv/bin/yolo segment train data=arena.yaml model=models/yolo11n-seg.pt epochs=100
.venv/bin/python tools/export_model.py runs/segment/train/weights/best.pt \
    --output models/checkpoint.onnx

# 5. Set model_path + checkpoint_mode: "model" di robot.json
```

Kontrol rekam (jalan lewat SSH, tanpa display):

| Tombol | Fungsi |
|---|---|
| `SPACE` | mulai / stop rekam — bisa beberapa take per sesi, penomoran frame lanjut |
| `Q` | keluar |
| `Ctrl+C` | stop darurat (frame yang sudah tersimpan tetap di-index) |

Tanpa terminal (piped/nohup), pakai `--frames N` / `--duration S`.
Default format **PNG** (lossless — JPEG chroma subsampling menggeser
hue, penting buat detektor berbasis warna); `--format jpg` kalau disk
sempit (~3x lebih kecil). Frame di-undistort kalau `homography_path`
di-set — dataset SAMA dengan input detector di runtime.

## Peta fase

1. **Fase 0 — fondasi** ✅: config, state, protokol, log, test.
2. **Fase 1 — persepsi lajur** (inti selesai): LaneTracker + eval sintetis;
   tinggal validasi bench dengan kamera asli (roda terangkat, `dry_run`).
3. **Fase 2 — detektor semantik** (infrastruktur selesai): ONNX via
   cv2.dnn + decode YOLO11-seg + NMS per-kelas + mask mapping; export
   tool + bench integration. Tinggal: dataset arena-3d -> train kelas
   checkpoint -> logika checkpoint.
4. **Fase 3 — kontrol** ( Stanley + speed profile selesai): replay sim
   against track geometry lolos (converge <5mm steady-state). Tinggal:
   tuning terhadap persepsi nyata (butuh mock lane/arena).
5. **Fase 4 — closed-loop + MCU** (runner selesai): `--closed-loop`
   persepsi -> Stanley -> MCU, dry_run default. Tinggal: MCU status
   protocol (feedback MCU -> Jetson), bench roda terangkat.
6. **Fase 5 — checkpoint** (groundwork selesai): `checkpoint_mode`
   di config memilih backend deteksi checkpoint — `none` (mati,
   default) | `cv` (classical CV warna/bentuk, nol training, ~5 ms)
   | `model` (YOLO ONNX, butuh model terlatih). Tinggal: spec
   Technical Meeting + foto arena untuk tuning threshold / training.

## Keselamatan

- E-stop fisik memutus penggerak independen dari software; watchdog MCU
  (100 ms) lapis terakhir. Jangan matikan `dry_run` sebelum uji meja dengan
  roda terangkat.
- Saat lomba: tidak ada stream/dashboard/telemetri — log lokal saja.
- `stream_host` default `127.0.0.1`; `0.0.0.0` hanya untuk jaringan pit
  tepercaya.
