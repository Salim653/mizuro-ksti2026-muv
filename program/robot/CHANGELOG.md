# Changelog — program/robot

Riwayat paket runtime robot (`program/robot/`). Scaffold lama
(`program/jetson/`) punya riwayat commit sendiri dan tidak disentuh sejak
build baru dimulai. Format: satu entri per commit besar, plus catatan
verifikasi dan yang BELUM diuji.

## 2026-10-30 — Fase 2: detektor semantik ONNX (cv2.dnn)

Branch `feature/camera-mount-offset`.

- `mizuro/perception/detector.py` (baru): `SemanticDetector` —
  frame BGR -> list[Detection] (class_id, label, confidence, bbox,
  mask) via cv2.dnn. Kenapa cv2.dnn bukan onnxruntime: runtime WAJIB
  /usr/bin/python3 (JetPack) dan di sana tidak ada onnxruntime.
- Decode YOLO11-seg sebagai fungsi murni numpy (testable tanpa model):
  `letterbox` (aspect-preserve + meta), `decode_yolo_seg` (116 =
  4 bbox + 80 kelas + 32 koefisien mask; prototipe 32x160x160),
  `nms_per_class` (per-kelas — objek kelas beda yang overlap tidak
  saling menekan), `mask_to_original` (proto -> letterbox -> crop
  bbox -> peta balik skala/pad -> frame asli).
- Bug yang ketangkap tes: `squeeze(0)` gagal tanpa batch axis (pakai
  `squeeze()`); NMS awal menghitung IoU box terhadap DIRINYA SENDIRI
  bukan terhadap semua box (overlap lolos semua).
- `tools/export_model.py` (baru): export .pt -> ONNX + PERINGATAN
  venv (export ultralytics meng-upgrade numpy/opencv; kalau cv2 rusak:
  pip install numpy==1.26.4 opencv-python==4.8.0.76).
- `main.py --stream-only`: detektor ikut jalan kalau `model_path`
  di-set (log label:conf tiap 15 frame + waktu inferensi).
- `.gitignore`: `*.onnx` (model 11.7 MB tidak boleh ke repo).
- Validasi: decode vs array sintetis layout asli (recover bbox/kelas/
  mask); integrasi model nyata `yolo11n-seg.onnx` (COCO) — frame bench
  terdeteksi person 0.82 / cup 0.82 / bottle 0.69 dengan mask; live
  kamera via /usr/bin/python3: laptop 0.93, bottle 0.44.
- Kinerja: ~260-310 ms/frame CPU. JetPack cv2 4.8.0 TIDAK di-build
  dengan CUDA DNN backend (cek: "DNN module was not built with CUDA
  backend"), jadi tidak ada akselerasi GPU via cv2.dnn. Kalau nanti
  perlu lebih cepat: TensorRT atau onnxruntime-gpu (belum diuji).
- Belum diuji: model kelas arena/checkpoint (butuh dataset arena-3d +
  training); logika checkpoint (butuh spec arena).

## 2026-10-29 — Kalibrasi kamera: tools/calibrate.py + wiring homography_path

Branch `feature/camera-mount-offset`.

- `tools/calibrate.py` (baru): kalibrasi checkerboard -> intrinsik
  terukur. Preview MJPEG live, deteksi sudut (adaptive threshold +
  normalize, robust di lantai bertekstur), capture N frame dengan
  anti-duplikat (papan harus pindah >= 30 px + jeda minimal), lalu
  `cv2.calibrateCamera` (zero tangent, fix k3). Output JSON kompatibel
  `from_json` + laporan rpe/hFOV. Frame tersimpan di
  `runs/calib-frames/`; `--replay` hitung ulang tanpa kamera.
  CATATAN: height_m & pitch TIDAK diukur alat ini — pengukuran fisik
  dari robot.json/argumen.
- `CameraCalibration`: field `dist` (koefisien distorsi, divalidasi
  panjang), `camera_matrix()` (3x3 buat cv2), `hfov_deg()` (laporan).
  `from_json` menerima "dist".
- `Camera`: parameter `undistort=(K, dist)` — frame direktifikasi sekali
  di read() kalau kalibrasi punya distorsi; ukuran tidak berubah jadi
  validasi ukuran tetap jalan.
- `main.py --stream-only`: `homography_path` (sebelumnya TIDAK PERNAH
  di-load) sekarang dipakai — kalau di-set, kalibrasi terukur mengganti
  default hFOV 62 deg dan undistort otomatis aktif. Log startup
  mencetak sumber kalibrasi + hFOV efektif.
- Tes: +14 (round-trip dist, camera_matrix, hfov, penolakan panjang
  dist, undistort mengubah isi bukan ukuran, object_points, deteksi
  checkerboard sintetis, kalibrasi recover intrinsik dikenal dalam
  2%, zero-tangent by design, coverage hint). Total 174 lolos.
- Belum diuji: kalibrasi dengan kamera NYATA (butuh checkerboard
  cetak + LO jalankan di Jetson).

## 2026-10-29 — Bench: mode `--stream-only` (kamera + MJPEG + tracker)

Branch `feature/camera-mount-offset`.

- `main.py`: flag `--stream-only` — buka kamera CSI, publish MJPEG
  (`http://127.0.0.1:8080/`), dan jalankan `LaneTracker` per frame.
  TANPA motor, TANPA serial: mode bench murni. Tanpa `robot.json`
  memakai Config default (mount arena-3d).
- Frame sampel ditulis ke `runs/bench-frames/frame-NNNNNN.jpg` setiap
  30 frame buat diagnosa offline (lantai nyata tidak bisa direproduksi
  synth).
- Log per-frame: lateral/heading/confidence + stasiun terdeteksi;
  frame tanpa lajur log alasan + polarity terpilih.
- **Bench kamera nyata LOLOS** (Jetson Orin Nano, Pi Cam v2 imx219,
  1280x720@60 mode 4): kamera terbuka, frame mengalir, exposure wajar,
  tracker menolak scene non-lajur dengan benar (confidence rendah,
  16/16 stasiun, polarity light_on_dark). Validasi tracker di lantai
  NYATA dengan marka lajur masih pending — arena/lajur belum tersedia.
- Saga diagnosa kamera (buat catatan tim): dmabuf error nvargus ->
  sunshine pegang /dev/video0 (dimatikan) -> restart nvargus-daemon ->
  reload nvmap/reboot -> kamera tidak terdeteksi (error -121 EREMOTEIO,
  ribbon CSI tidak seat) -> reseat -> kamera hidup. Pelajaran: JANGAN
  open/close kamera berulang-ulang cepat; state daemon degradasi dan
  butuh ~30 s idle untuk recover.
- Verifikasi: 160 tes lolos, self-check lolos (dengan & tanpa `-O`),
  py_compile OK. Belum diuji: tracker di lantai nyata dengan marka
  lajur (butuh arena/mock lane), `--dataset` against zip arena-3d asli.

## 2026-10-29 — Kamera: konfigurasi mount x/y/z + yaw/roll

Branch `feature/camera-mount-offset`.

- `Config`: enam field mount baru — `mount_x_m`, `mount_y_m`,
  `mount_z_m`, `mount_pitch_deg`, `mount_yaw_deg`, `mount_roll_deg`.
  Posisi meter, orientasi derajat (diukur di bench, bukan ditebak);
  default = mount arena-3d (0.16 m / 12 derajat, offset 0). Rentang
  divalidasi; ikut masuk `_check_types` dan `robot.example.json`.
- `CameraCalibration`: field mount yang sama + `from_config()` (Config
  -> kalibrasi, derajat -> radian). `pixel_to_ground`/`ground_to_pixel`
  sekarang menerima/mengembalikan titik di frame ROBOT: roll dibatalkan
  di ruang piksel (rotasi di sekitar titik principal), yaw + offset x/y
  diterapkan setelah proyeksi pitch. Default 0 = perilaku pitch-only
  persis; eval 100 frame identik (0.73 mm / 0.11 derajat) sebelum dan
  sesudah patch.
- `homography()` mengkomposisi roll (piksel) + pitch + yaw/offset
  (tanah); `ground_from_pixels` ikut konsisten (diuji).
- `horizon_row_worst`: batas ROI aman untuk horizon miring (roll).
  Tracker memakainya; synth menggambar batas dinding miring per kolom
  (`_above_horizon_mask`).
- `from_hfov`/`from_json` menerima parameter mount (opsional);
  `tools/eval_perception.py` dapat flag `--mount-x/-y/-yaw/-roll`.
- Karakteristik terukur (bukan bug): offset x/y saja -> error 2.15 mm
  (terjemahan murni tertangani eksak); yaw/roll -> 7.41 mm, residual dari
  window pencarian stasiun yang terotasi + crop horizon miring yang
  memotong stasiun jauh. Semua skenario masih LOLOS target Fase 1.
- Tes: +20 (round-trip mount penuh, offset geser tepat, tanda yaw,
  roll round-trip + end-to-end tracker kamera roll/offset, from_config,
  from_json mount, rentang Config). Total 160 lolos.

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
