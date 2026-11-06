# Changelog — program/robot

Riwayat paket runtime robot (`program/robot/`). Scaffold lama
(`program/jetson/`) punya riwayat commit sendiri dan tidak disentuh sejak
build baru dimulai. Format: satu entri per commit besar, plus catatan
verifikasi dan yang BELUM diuji.

## 2026-11-06 (tambahan) — video-file sebagai kamera + bench tanpa hardware

Alat bench baru untuk menguji loop penuh (persepsi → Stanley → MCU)
**tanpa Jetson, tanpa motor, tanpa hardware**, hanya dengan:

- `tools/virtual_mcu.py` — cermin pemetaan output firmware `body_controller.ino`
  (skid4/esc1/hb1, clamp, ESP arming hold 2s, status MCU sintetis).
- `tools/mcu_bench.py` CLI:

  ```
  ../../.venv/bin/python tools/mcu_bench.py --video ../../recordings/capture_001.mp4 --duration 30
  ../../.venv/bin/python tools/mcu_bench.py --video laps.mp4 --driver external_esc
  ../../.venv/bin/python tools/mcu_bench.py --video laps.mp4 --no-loop
  ../../.venv/bin/python tools/mcu_bench.py --video laps.mp4 --dashboard-port 18765
  ```

  - Kamera = file video (bukan CSI, tidak butuh GStreamer, kecuali `--dashboard-port`).
  - MCU = VirtualMcu (dry-run selalu, wheels tidak pernah menyala di sini).

  Output: `runs/run-*/telemetry.csv` + `events.jsonl` +
  `bench_summary.json` (statistik motor virtual: PWM maks, ganti arah,
  clamp bersama, rentang pulse ESC/servo).

- `tests/test_camera.py` ditambah:
  - sumber file video (heuristic `"!"` → GStreamer).
  - EndOfVideo handling (bukan RuntimeError).

- `tests/test_virtual_mcu.py` (baru) mengunci nilai cermin ke angka tes host firmware.

- `tests/test_mcu_bench.py` (baru) verifikasi pembentukan config dan argumen CLI.

- `program/robot/mizuro/camera.py`:
  - sumber file video: heuristik `"!"` = pipeline GStreamer, selain itu =
    path file; ukuran beda di-resize ke config; habis = `EndOfVideo`
    (bukan error); flag `video_loop` buat putar ulang file.
- `program/robot/main.py`: `stream_only` menangkap `EndOfVideo` (video
  habis = berhenti bersih, bukan traceback).

- **Verifikasi:** 385 pytest + self-check + py_compile + run bench video
  berhasil — loop penuh + wheel-summary berhasil pada persepsi nyata
  (video capture_001.mp4).

## 2026-11-06 — MCU v2: tiga mode output + protokol status balik

Branch `feature/mcu-v2-status`.

- **Firmware** (`program/body-mcu/body_controller.ino`, ditulis ulang):
  tiga mode output runtime-selectable lewat serial `#M,<mode>\n` —
  `skid4` (prototype: 4x L298N, skid steer, mixing `speed+steer` → roda
  kiri/kanan dilakukan di MCU), `esc1` (kompetisi: ESC brushless +
  servo kemudi), `hb1` (bench: 1 motor PWM/DIR + servo). Tanpa
  `printf %f` (newlib-nano AVR) — formatter manual `appendFixed`.
- **Protokol dua arah** (`mizuro/protocol.py`): `FIRMWARE_MODES`
  (`external_esc`→`esc1`, `mcu_hbridge`→`hb1`, `skid4_l298n`→`skid4`),
  `format_mode()`, dan `parse_status_line()` untuk baris status
  `#v1,<mode>,<estop>,<stale>,<speed_out>,<steer_out>,<rx_ok>,<uptime_ms>`
  (validasi ketat: 8 field, mode whitelist, batas ±0.8/±0.55, int
  non-negatif; baris rusak diabaikan, bukan exception).
- **`mizuro/mcu.py`**: param `driver` (divalidasi selalu, dry-run pun —
  config salah gagal di konstruktor, bukan jadi output salah di roda),
  kirim `#M,<mode>` saat port dibuka, `read_status()` non-blocking
  dengan buffer dibatasi (`MAX_STATUS_BUFFER=256`).
- **`mizuro/runner.py`**: `_poll_mcu_status()` di loop setelah
  `mcu.send()` — event `mcu_status_ready`/`mcu_status_lost`/
  `mcu_status_recovered`, `mcu_estop`/`mcu_estop_clear`,
  `mcu_stale`/`mcu_stale_clear`, `mcu_mode_mismatch`/`mcu_mode_ok`
  (bandingkan `status.mode` vs `FIRMWARE_MODES[mcu.driver]`), param
  `status_timeout_s` (default 2.0). Sengaja TIDAK mengubah state
  machine: watchdog MCU sudah memutus penggerak independen dari Python.
- **`mizuro/config.py`**: `MOTOR_DRIVERS` = kunci `FIRMWARE_MODES`
  (sumber kebenaran tunggal); default `motor_driver` jadi
  `skid4_l298n` (hardware prototype sekarang). `robot.json` dan
  `robot.example.json` disamakan.
- **Tes**: `tests/test_protocol.py` (+17: `format_mode`,
  `parse_status_line` valid/rusak/batas), `tests/test_mcu.py` (+10:
  FakeSerial `in_waiting`/`read`, baris mode saat port dibuka, driver
  invalid, parsing/partial/junk/OSError), `tests/test_runner.py` (+8:
  StubMcu `driver`/`read_status`, event status, dry-run tidak poll),
  `tests/test_config.py` (+2: semua driver firmware diterima).
  `program/body-mcu/test/`: tes host g++ + mock `Arduino.h`/`Servo.h`
  — 120 check, 18 tes, `-Wall -Wextra -Werror`.
- **Verifikasi**: `pytest tests/ -q` = 354 lolos (baseline 312);
  `main.py --self-check`, `dashboard.py --self-check`, `py_compile`
  lolos; `test/run_tests.sh` 120 check 0 failure.
- **Belum diuji**: compile Blackpill (core `stm32duino` belum terpasang
  di Jetson — `arduino-cli` repo ini aarch64-only), pin map dan
  `SKID_TRACK_M`/deadband ESC masih placeholder, status balik dengan
  MCU sungguhan, roda belum pernah menyala.

## 2026-11-06 — main.py --dashboard: dashboard nyala bersama runtime

Branch `feature/camera-mount-offset`.

- `main.py`: flag `--dashboard` (+ `--dashboard-port`, default 18765)
  menyalakan server dashboard di thread daemon selama mode
  `--stream-only` atau `--closed-loop` berjalan. Dashboard membaca
  log run yang SEDANG ditulis proses ini (`latest_status` memilih run
  terbaru dari disk) — telemetry terlihat hidup tanpa pipa
  antar-proses. Default OFF: saat lomba semua HTTP wajib mati, jadi
  opt-in eksplisit; `--dashboard` sendirian (tanpa mode) = SystemExit
  dengan penjelasan.
- URL MJPEG di-inject ke halaman: di `--stream-only`, URL stream
  (`server.url`) dikirim ke `make_server(..., feed_url=)` dan
  menggantikan placeholder `__FEED_URL__` di `dashboard.html` —
  panel kamera langsung terisi, operator tidak ketik manual.
  Di-escape lewat `dashboard._js_string()` (kutip, backslash, `<`)
  sebelum masuk literal JS inline.
- Shutdown bersih: `dash.shutdown()` + `server_close()` di `finally`
  kedua mode — Ctrl+C atau run selesai tidak meninggalkan server
  nyangkut.
- `dashboard.py`: `make_server` menerima `feed_url` opsional;
  `_js_string()` escape; self-check sekarang assert injeksi URL +
  token bersih.
- `dashboard.html`: `const DEFAULT_FEED = "__FEED_URL__";` — nilai
  awal field kamera (localStorage tetap menang kalau ada).
- `tests/test_main.py` (baru): 5 tes — `--dashboard` tanpa mode
  ditolak, `--dashboard-port` di luar [1, 65535] ditolak (termasuk
  0), `serve_dashboard` start/serve/stop + injeksi URL + pesan
  peringatan, dan dashboard melihat RunLog hidup (run terbaru,
  fresh, nilai command benar).
- `tests/test_dashboard.py`: +3 tes — `_js_string` escape,
  injeksi URL (payload `" onload=` tidak bocor mentah), token bersih
  kalau `feed_url` kosong.
- `README.md`: baris flag `--dashboard`/`--dashboard-port` + bagian
  "Dashboard dari main.py" dengan contoh output.
- Belum diuji: `--stream-only --dashboard` dengan kamera CSI asli
  (butuh Jetson); panel kamera same-origin vs cross-port di browser
  sungguhan.

## 2026-11-06 — dashboard.py + dashboard.html: dashboard run full (bench/pit)

Branch `feature/camera-mount-offset`.

- `dashboard.py` (baru, stdlib only): server HTTP read-only buat
  `runs/run-*/telemetry.csv` + `events.jsonl`. Tanpa dependensi luar —
  jalan dengan `/usr/bin/python3` di Jetson apa adanya. Endpoint:
  `/api/status` (run terbaru atau `?run=`), `/api/runs` (daftar run +
  jumlah baris), `/api/history?points=` (deret waktu buat grafik,
  downsample indeks rata — 240 titik dari 18.000 baris), `/api/demo`
  (data sintetis buat preview UI). Flag: `--runs` (default `runs`
  relatif cwd, sama seperti `output_dir` main.py), `--host` (default
  127.0.0.1), `--port` (default 18765), `--self-check`.
- `dashboard.html` (baru, file TERPISAH — bisa diedit anggota tim
  tanpa menyentuh Python): UI bahasa Indonesia, tema gelap. Isi:
  readout kecepatan/kemudi besar + meter kemudi, readout confidence /
  error lateral (mm) / error heading (derajat) / radius belokan,
  grafik canvas dua sumbu (kecepatan m/s + error lateral mm vs waktu,
  DPR-aware, null-safe), pemilih run (analisis run lama setelah
  lomba), preview kamera MJPEG opsional (URL di localStorage, validasi
  http/https saja), mode simulasi, indikator freshness (mtime < 3 s =
  log hidup), daftar event runtime.
- Keamanan: hanya NAMA event yang dikirim — detail event (bisa berisi
  config/secret seperti `serial_device`) tidak pernah diekspos; nama
  run dari query divalidasi regex (`run-[\w.-]+`), path traversal
  ditolak HTTP 400; CSP + `Cache-Control: no-store` +
  `X-Content-Type-Options: nosniff` di setiap respons; semua endpoint
  read-only (tidak ada yang menulis/menggerakkan motor).
- Freshness + run asli: `latest_status` memilih run terbaru secara
  leksikografis (`run-YYYYmmdd-HHMMSS`), `list_runs` menghitung baris
  per run, `run_history` membaca CSV penuh dengan `csv.reader` +
  peta indeks header (lebih cepat dari DictReader untuk 18k baris).
- `tests/test_dashboard.py` (baru): 24 tes — number() junk/NaN/inf,
  latest_status (baris terakhir, whitelist event + anti-kebocoran
  secret, freshness, run terbaru, run eksplisit, traversal ditolak,
  run hilang, header-only, FAULT reason), list_runs (urutan + flag
  latest + hitung baris), run_history (downsample rata, clamp points,
  run kosong, traversal), demo_status, server HTTP (halaman, status,
  runs, history, demo, 400 traversal, 404, header keamanan), dan
  `self_check()` stdlib-only.
- Bug yang ketangkap tes saat development: downsample awal pakai
  stride + append baris terakhir -> jumlah titik bisa melampaui
  `points` (21 titik untuk points=10 karena clamp minimum 20);
  diganti indeks rata `round(i*last/(count-1))` — tepat `points`
  titik, ujung selalu termasuk. Assertion self-check juga salah
  (run lama tanpa baris data masih `available` kalau ada
  `t_monotonic`) — sudah diperbaiki jadi cek state STOPPED.
- `README.md`: bagian "Dashboard run" (flag, endpoint, fitur,
  keamanan, peringatan lomba) + baris di tabel isi paket.
- `AGENTS.md` (root): code map diperbaiki — `program/robot/` adalah
  runtime KANONIK (package `mizuro`, flag `--closed-loop`),
  `program/jetson/` adalah scaffold Inggris lama (referensi saja,
  jangan salin kode/protokolnya), `program/robot-fw/` copy scaffold
  lama. Verification, interpreter/camera, protokol/config, status
  persepsi, dan keselamatan diupdate ke path + status paket `robot`
  (lane keeping CV + Stanley + closed-loop runner selesai; motor
  belum pernah menyala).
- Belum diuji: tampilan di browser sungguhan (grafik canvas, feed
  MJPEG, mode simulasi) — butuh buka halaman di laptop; beban
  `list_runs` + `run_history` pada file 18k baris di SD card Jetson.

## 2026-11-05 — models/: direktori file model (gitignored)

Branch `feature/camera-mount-offset`.

- `program/robot/models/` (baru): tempatkan semua file model —
  `yolo11n.pt` dan `yolo11n-seg.pt` dipindahkan dari root repo ke
  sini (copy di `test-yolo*/` tidak disentuh; script bench mengharapkan
  file di folder masing-masing). Root `.gitignore` sudah memuat
  `*.pt`/`*.onnx` jadi isi direktori tidak pernah ke-commit; yang
  masuk git hanya `models/README.md` (konvensi penamaan + alur
  train -> export -> `model_path`).
- `yolo11n-seg.onnx` TETAP di root: direferensiasi by path oleh
  `tests/test_detector.py` (`parents[3] / "yolo11n-seg.onnx"`).
  Pindah butuh update tes + `_ONNX` — tidak dilakukan di commit ini.
- README paket: contoh export sekarang menulis ke
  `models/checkpoint.onnx`.

## 2026-11-03 — tools/record_dataset.py: kontrol manual SPACE/Q + default PNG

Branch `feature/camera-mount-offset`.

- Kontrol manual lewat terminal raw mode (cbreak): SPACE = mulai/stop
  rekam, Q = keluar. Jalan lewat SSH TANPA display — `cv2.waitKey`
  butuh jendela GUI yang tidak ada di Jetson headless. ISIG tetap
  aktif jadi Ctrl+C masih raise KeyboardInterrupt.
- Mode PREVIEW: dengan keyboard aktif, alat mulai BELUM rekam;
  browser preview jalan, SPACE baru mulai. Beberapa take per sesi
  dibolehkan — penomoran frame dan index.csv menyatu across takes.
- Tanpa terminal (piped/nohup/systemd) keyboard otomatis mati;
  `--frames`/`--duration` jadi satu-satunya stop condition (dan
  `--no-keyboard` mematikan eksplisit). Ctrl+C di titik mana pun
  (termasuk `cam.read()`) tetap mengembalikan rows yang sudah
  tersimpan.
- Default format PNG (lossless). Alasan: JPEG chroma subsampling
  4:2:0 menggeser hue — kalau nanti pakai `checkpoint_mode: "cv"`
  (deteksi berbasis warna), dataset PNG menjaga warna persis seperti
  yang robot lihat. `--format jpg` tetap ada (~3x lebih kecil/cepat).
- `Keyboard` class: read_key disuntikkan bisa (tes); restore terminal
  dipanggil di finally — terminal rusak kalau dilewat.
- `tests/test_record_dataset.py`: 27 tes — Keyboard (space/quit/
  tombol lain diabaikan/close mati total/reader error selamat),
  parse_args (keyboard-only valid di TTY, ditolak tanpa TTY),
  mode manual (preview sampai SPACE, stop-restart takes, penomoran
  menyatu, quit sebelum rekam = 0 frame, --frames tetap berlaku,
  preview frame tetap dipublish).
- Bug yang ketangkap tes: FakeKeyboard awal mengembalikan byte mentar
  (`b" "`) bukan "space" — loop masuk preview selamanya. Mapping
  tombol sekarang diverifikasi tes.
- Belum diuji: SPACE/Q dengan terminal asli di Jetson (butuh bench
  interaktif); throughput tulis PNG di SD card Jetson.

## 2026-11-03 — tools/record_dataset.py: perekam dataset headless

Branch `feature/camera-mount-offset`.

Adaptasi dari script rekam teman (`jetson_record_720p60.py`) ke
konvensi repo: pakai `Camera.from_config` (bukan pipeline GStreamer
tulis ulang), preview MJPEG headless (bukan `cv2.imshow` — Jetson
di robot tidak punya display), dan simpan frame MENTAH + timestamp
(bukan mp4 lossy tanpa waktu).

- `tools/record_dataset.py` (baru): rekam frame + `index.csv`
  (filename, t_monotonic_s, t_wall_s, width, height) ke
  `runs/dataset-<timestamp>/frames/`.
- Kontrol headless: `--frames N` / `--duration S` / Ctrl+C (tidak
  ada keyboard listener; SPACE-to-record butuh display).
- `--every N` decimasi (kamera 60fps -> data 30fps), `--warmup S`
  buang frame awal (auto-exposure settle), `--format jpg|png`,
  `--quality`, `--no-stream` (matikan server preview).
- Frame di-undistort kalau `homography_path` di-set — dataset SAMA
  dengan input detector di runtime (tidak ada domain gap).
- Ctrl+C di titik mana pun (termasuk `cam.read()`) tetap
  mengembalikan rows yang sudah tersimpan — index.csv konsisten
  dengan file di disk.
- `tests/test_record_dataset.py` (baru): 15 tes — parse_args
  (wajib punya stop condition, validasi rentang), writer jpg/png
  (png lossless, jpg near-exact untuk warna solid), loop rekam
  (jumlah frame, durasi, decimasi, warmup, kamera gagal, Ctrl+C
  keeps rows, publish ke hub, timestamp naik, isi file = isi frame).
- Belum diuji: rekam dengan kamera CSI asli di Jetson (butuh bench);
  throughput tulis PNG di SD card Jetson belum diukur.

## 2026-11-03 — Fase 5 (groundwork): checkpoint_mode + detektor CV klasik

Branch `feature/camera-mount-offset`.

- `mizuro/perception/checkpoint_cv.py` (baru): `CheckpointDetectorCV` —
  detektor checkpoint classical CV, nol training. Threshold warna HSV
  + morphology (open/close) + filter kontur (area, solidity).
  Mengembalikan tipe `Detection` yang sama dengan `SemanticDetector`,
  jadi pemanggil tidak peduli backend mana yang aktif.
  Preset warna: green/red/orange/yellow/blue (bisa di-override
  dengan rentang HSV eksplisit di config). ~5 ms/frame.
- Config: field baru `checkpoint_mode` ("none" | "cv" | "model"),
  `checkpoint_color`, `checkpoint_hsv_low/high`,
  `checkpoint_min_area_px`, `checkpoint_max_area_px`,
  `checkpoint_min_solidity`, `checkpoint_label`. Default "none" =
  detektor mati, lane keeping tetap jalan penuh.
- `Config.validate()`: `checkpoint_mode` harus salah satu dari
  none/cv/model; mode "model" mensyaratkan `model_path`; HSV harus
  tuple 3 angka (atau kosong = preset) dengan low <= high per kanal;
  area & solidity divalidasi rentangnya.
- Perubahan semantik live: `model_path` TIDAK lagi wajib saat
  `dry_run=false` — yang mengatur adalah `checkpoint_mode`. Live
  dengan `checkpoint_mode: "cv"` jalan tanpa model sama sekali.
- `main.py`: helper `build_detector(config)` dipakai bersama oleh
  `--stream-only` dan `--closed-loop`; pilih backend dari
  `checkpoint_mode`.
- `robot.example.json`: semua field checkpoint baru ditambahkan
  (tes `test_example_config_matches_dataclass_fields` menegakkan).
- `tests/test_checkpoint_cv.py` (baru): 22 tes — blob solid terdeteksi,
  frame kosong/None aman, filter area (kecil/besar) ditolak, filter
  solidity (garis diagonal, cross) ditolak, kelima preset warna,
  override HSV eksplisit, `from_config`, kecepatan < 50 ms/frame.
- `tests/test_config.py`: tes baru untuk validasi checkpoint_mode,
  HSV, area, solidity; tes live-mode disesuaikan dengan semantik baru.
- Belum diuji: detektor CV terhadap frame arena ASLI (butuh foto
  arena / spec checkpoint dari Technical Meeting 8 Nov). Threshold
  warna kemungkinan perlu tuning terhadap pencahayaan nyata.

## 2026-11-03 — Fase 4: closed-loop runner (persepsi -> Stanley -> MCU)

Branch `feature/camera-mount-offset`.

- `mizuro/runner.py` (baru): `ClosedLoopRunner` — satu loop kontrol
  dengan semua dependensi disuntik (kamera, tracker, controller, mcu,
  log, state machine) supaya bisa diuji TANPA hardware.
- Kontrak keselamatan yang ditegakkan loop:
  1. Setiap iterasi WAJIB kirim setpoint ke MCU (watchdog 100 ms) —
     bahkan nol saat berhenti. Ini heartbeat.
  2. Confidence 0 / lajur hilang -> controller kirim nol; state ->
     DEGRADED tapi loop TETAP JALAN (robot berhenti, bukan crash).
  3. Loop overrun (> 1/loop_hz) di-log; bukan fatal.
  4. Timeout run (default 300 s) -> STOPPED.
  5. Detektor error TIDAK menghentikan loop — di-log, loop lanjut.
- `main.py --closed-loop`: mode closed-loop. dry_run default true
  (loop penuh jalan, MCU tidak menulis serial). Motor hanya nyala
  kalau dry_run=false + roda terangkat + E-stop fisik.
- `tests/test_runner.py` (baru): 11 tes — heartbeat setiap iterasi,
  lane lost -> DEGRADED, lane recovered -> RUNNING, no-lane kirim
  nol, shutdown kirim nol final, dry_run tidak sentuh serial,
  detektor error tidak stop loop, telemetry rows ditulis.
- Fix test detektor: frame bench terakhir bisa 0 byte (proses di-kill
  saat nulis) — test sekarang buang file kosong dan verifikasi
  pipeline jalan, bukan objek spesifik (frame lane wajar kosong
  untuk model COCO).
- Belum diuji: closed-loop dengan kamera asli (butuh bench roda
  terangkat); MCU status protocol (MCU -> Jetson feedback).

## 2026-10-30 — Fase 3: controller Stanley + speed profile + replay sim

Branch `feature/camera-mount-offset`.

- `mizuro/controller.py` (baru): `StanleyController` — lane state ->
  Command (speed + steering). Stanley: `steer = heading_gain * heading
  + atan(lateral_gain * lane_error / (v + softening))`. Speed dari
  kurvatur: `v = sqrt(a_lat * R)`, di-clamp [min_speed, max_speed],
  di-smooth dengan clamp akselerasi/rem. Confidence 0 / lane_error
  None -> Command nol (kontrak keselamatan).
- Konvensi tanda terverifikasi dari geometry.py + firmware: lane_error
  > 0 = lajur di kanan robot -> steer positif; heading_error > 0 =
  lajur mengarah kanan -> steer positif; curvature > 0 = belok kanan.
- Config: 5 field baru (`min_speed_mps`, `max_accel_mps2`,
  `max_decel_mps2`, `stanley_softening_mps`, `min_curve_radius_m`) +
  validasi + example json.
- `tools/replay_control.py` (baru): simulasi closed-loop Stanley
  against track_data.json (arc-length model, bicycle dynamics).
  Skenario: start centred + start dengan offset/heading error.
  Lapor transient + steady-state error + max lateral accel.
- Hasil replay: dari offset 0.2m + heading 20deg -> converge ke
  3.1mm / 1.47deg steady-state, lap 19.4m selesai 24.8s, max a_lat
  0.43 m/s^2 (batas 3.0). Speed profile inert di track spec (R >= 1.5m
  di 0.8 m/s hanya butuh 0.43 m/s^2) — berfungsi sebagai safety net
  untuk kurva tak terduga, bukan speed controller utama.
- Bug ketangkap saat development: sign convention simulasi terbalik
  (d > 0 = robot di kanan centreline = lajur di KIRI robot, bukan
  kanan) — robot spin 180 derajat sebelum diperbaiki.
- Belum diuji: controller terhadap persepsi NYATA (butuh mock lane /
  arena fisik); tuning gain final (butuh data mount final).

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
