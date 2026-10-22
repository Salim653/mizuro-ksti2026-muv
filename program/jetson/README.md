# Scaffold runtime robot

Jetson Orin Nano memegang akuisisi gambar, inferensi ONNX, geometri lajur, perencanaan, dan log run. MCU bodi memegang PWM, sampling encoder/IMU, input start/e-stop, dan watchdog setpoint. Runtime tidak boleh menggantikan watchdog tersebut.

## Menjalankan pertama kali dengan aman

Pakai `/usr/bin/python3` (OpenCV JetPack, punya backend GStreamer). Jangan pakai `.venv` repositori: `opencv-python` dari pip tidak membawa GStreamer, jadi kamera CSI gagal dibuka. Jalankan `deactivate` dulu bila venv aktif — `Camera` akan menyebutkan masalah ini secara eksplisit bila salah interpreter.

```sh
python3 main.py --self-check
python3 main.py --config robot.example.json
```

Konfigurasi contoh berjalan kering (dry). Ia membuat direktori run bertimestamp berisi `telemetry.csv` dan `events.jsonl`, dan tidak mengirim perintah aktuator. Salin ke `robot.json` sebelum mengisi nilai. Jangan matikan `dry_run` sampai kamera, model terlatih, input start, protokol MCU, perilaku e-stop, dan watchdog sudah diuji di meja dengan roda terangkat.

## Stream kamera (bench/pit)

Streamer ada di dalam framework ini, bukan proses terpisah. Satu capture, satu pemilik kamera CSI.

```sh
python3 main.py --stream-only --config robot.json   # kamera saja, robot tidak di-arm
```

Endpoint di `stream_host:stream_port` (default `0.0.0.0:8080`): `/` halaman viewer, `/stream` MJPEG, `/snapshot` satu JPEG.

- `--stream-only` membuka kamera dan menyajikan MJPEG tanpa MCU dan tanpa gate start. Jalankan manual di terminal sendiri, atau di latar belakang dengan `nohup`/`tmux`. Tidak ada unit systemd yang terpasang untuk framework ini; satu proses saja yang boleh memegang kamera CSI.
- Run live (`stream_enabled: true`) mem-publish frame yang sama dengan loop kontrol; tidak ada capture kedua.
- Encode JPEG hanya terjadi bila ada client terhubung dan dibatasi `stream_fps`; tanpa penonton biayanya nol. Ini encoder software OpenCV, karena image JetPack ini tidak menyediakan `nvjpegenc` di dalam proses Python. Ukur ulang bila menaikkan `stream_width`/`stream_height`/`stream_quality`.
- `stream_width`/`stream_height` = `0` berarti resolusi kamera penuh.
- Saat lomba, jangan jalankan streamer: feed kamera tidak boleh berjalan.

Contoh latar belakang tanpa supervisor:

```sh
cd ~/mizuro-ksti2026-muv/program/jetson
nohup python3 main.py --stream-only --config robot.json > /tmp/mizuro-stream.log 2>&1 &
tail -f /tmp/mizuro-stream.log
```

Hentikan dengan `Ctrl-C` (foreground) atau `pkill -f "main.py --stream-only"`.

## Isi folder

- `main.py`: peluncuran, orkestrasi state, dan loop.
- `core.py`: konfigurasi, data command/observation, dan state machine.
- `camera.py`: capture CSI/GStreamer. `camera_pipeline` kosong berarti pipeline bawaan `nvarguscamerasrc → nvvidconv → videoconvert → appsink` (BGR) pada `image_width` x `image_height`.
- `stream.py`: MJPEG over HTTP dari frame yang sama dengan loop kontrol (`FrameHub` + `MjpegServer`). Lihat bagian Stream kamera.
- `vision.py`: inferensi ONNX dan seam estimasi lajur.
- `controller.py`: seam lajur ke perintah kemudi.
- `mcu.py`: driver perintah serial. Protokol ASCII `speed_mps,steering_rad\n`, keduanya desimal dengan empat angka di belakang koma; rentang ±0.8 m/s dan ±0.55 rad pada 115200 baud.
- Loop utama berjalan pada `loop_hz` terkonfigurasi (default 60 Hz), mencatat event overrun saat satu siklus melewati periodenya, dan tidak bisa memaksa kamera CSI menghasilkan frame pada laju itu.
- `telemetry.py`: log run lokal.
- `dashboard.py` + `dashboard.html`: dashboard read-only opsional di `127.0.0.1:18765` secara default. Dari folder ini jalankan `python3 dashboard.py`; arahkan ke direktori output lain dengan `--runs PATH`. Khusus LAN pit yang tepercaya, lewati `--host 0.0.0.0` lalu buka `http://<jetson-ip>:18765/` dari perangkat lain. Aktifkan **Show clearly labeled simulated telemetry** untuk melihat contoh nilai palsu; nilainya berasal dari `/api/demo` dan tidak pernah ditulis ke log run. Masukkan URL streamer kamera (default jaringan USB Jetson: `http://192.168.55.1:8080/stream`) lalu connect untuk melihat MJPEG. Feed dilayani service HTTP terpisah dan bisa terblokir mixed-content atau kebijakan origin browser bila dashboard memakai HTTPS atau perangkat lain tidak bisa menjangkau endpoint kamera Jetson. Dashboard dan feed kamera hanya untuk bench/pit; matikan sebelum percobaan lomba.
- `../body-mcu/`: sketsa MCU motor/kemudi berbasis Arduino-compatible.

## Seam runtime

- `Camera` membuka pipeline GStreamer Jetson lewat OpenCV dan menolak ukuran frame yang tidak sesuai.
- `OnnxDetector` memuat ONNX lewat OpenCV DNN, tetapi saat ini hanya melaporkan bentuk output. Deteksi belum didekode; perintah live tetap nol sampai decoder spesifik model dan estimator lajur mengembalikan geometri valid.
- `LaneEstimator` masih butuh detektor lajur/dash terkalibrasi dan proyeksi ke bidang tanah. Validasi estimasi terhadap ground truth dataset simulator sebelum menyambungkan motor.
- `Controller` mengirim kecepatan nol selama estimasi lajur tidak tersedia. Hukum kemudinya baru titik awal, belum control kendaraan yang sudah dituning.
- `McuLink` mengirim tepat `speed_mps,steering_rad\n`; sketsa MCU memakai format yang sama. `motor_driver` hanya konfigurasi tercatat untuk pemilihan sketsa MCU, bukan tambahan data paket.
- `StartGate` adalah gate Enter di konsol untuk run bench. Adaptor tombol start fisik dan pengembalian status e-stop dari MCU belum terpasang; input e-stop di MCU tetap harus memutus penggerak secara independen.
- `max_inference_ms` dan `lane_lost_frames` meminta perintah nol / state DEGRADED saat visi terlambat atau tidak ada. Watchdog MCU tetap menjadi lapisan pertahanan terakhir.

## Folder MCU bodi

`../body-mcu/body_controller.ino` adalah contoh body controller berbasis Arduino-compatible dengan pin yang bisa dikonfigurasi. Ia mengeluarkan PWM + arah untuk driver seperti H-bridge kelas BTS7960, atau sinyal servo-PWM ke ESC eksternal bila dikawinkan demikian. Verifikasi driver persisnya, frekuensi/batas arus PWM, rentang pulsa kemudi, timer board, dan kemampuan pin sebelum dipakai. Pin map dan konversi speed ke PWM perlu kalibrasi untuk motor dan driver nyata. Sketsa ini belum mengimplementasikan umpan balik encoder/IMU maupun mengirim telemetri MCU kembali ke Jetson.

## Urutan integrasi pertama

1. Konfirmasi carrier dan JetPack, lalu uji capture CSI dengan sasis terangkat.
2. Implementasikan decoder output ONNX yang sebenarnya, lalu geometri lajur dan perbandingan dengan dataset.
3. Sepakati field dan satuan paket MCU; paket bersama saat ini dua float dipisah koma.
4. Verifikasi pengkabelan pin driver, pengkabelan e-stop, watchdog, dan kalibrasi kemudi dengan roda penggerak terangkat.
5. Pasang input start fisik. Jangan mengandalkan gate konsol untuk deploy lomba.

Tidak ada telemetri nirkabel atau berkabel yang boleh berjalan saat lomba; simpan log secara lokal. Matikan dashboard sebelum setiap percobaan. HTTP server bind ke localhost secara default; mengeksposnya dengan `--host 0.0.0.0` hanya untuk jaringan pit tepercaya dan menyajikan detail run tanpa autentikasi. Framework ini belum siap lomba sampai decoder model, estimator lajur, pelaporan status/fault MCU, integrasi start/e-stop fisik, dan kalibrasi hardware selesai.
