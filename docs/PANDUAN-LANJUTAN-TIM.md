# Panduan Memahami dan Melanjutkan MIZURO KSTI 2026

Panduan serah-terima untuk anggota tim yang baru masuk ke repositori. Dokumen ini membedakan **kode yang tercatat di Git**, **perubahan lokal Jetson per 22 Oktober 2026**, dan **perilaku yang sudah benar-benar diuji**. Jangan menyamakan kode prototipe dengan robot siap lomba.

## 1. Ringkasan kondisi

- Repositori menggabungkan firmware/runtime, simulasi, model, dokumentasi, dan CAD.
- Runtime Jetson menggunakan Python dan OpenCV/GStreamer. MCU mengendalikan aktuator dan watchdog.
- Snapshot Jetson di Git memakai `program/jetson/`. Pada perangkat Jetson terdapat perubahan stream kamera yang belum dikomit, file konfigurasi lokal, dan file kerja tambahan.
- `/usr/bin/python3` di Jetson lolos `main.py --self-check`, `dashboard.py --self-check`, dan kompilasi sintaks modul. Pemeriksaan kamera live tidak lolos: proses stream pernah mencatat `camera frame read failed`; pada pemeriksaan terakhir prosesnya sudah tidak mendengarkan di port 8080.
- Persepsi belum siap mengemudikan robot. Decoder ONNX dan estimasi geometri lajur masih placeholder, sehingga `confidence` tetap nol dan runtime meminta kecepatan nol.
- **Jangan menonaktifkan `dry_run` atau menghubungkan penggerak sebelum interlock, E-stop fisik, watchdog, protokol, aktuator, dan kamera diuji dengan roda terangkat.**

## 2. Tata letak repositori

| Jalur | Peran | Catatan status |
|---|---|---|
| `program/jetson/` | Aplikasi Python pada Jetson: kamera, inferensi, kontrol, MCU serial, telemetri, stream MJPEG, dashboard | Runtime masih scaffold; periksa status Jetson langsung untuk perubahan terbaru |
| `program/body-mcu/` | Contoh firmware Arduino-compatible untuk motor, servo, serial, E-stop dan watchdog | Target board/pin masih perlu dicocokkan dengan perangkat nyata; belum sama dengan keputusan STM32 |
| `program/robot-fw/` | Salinan/runtime alternatif dengan README dan dependensi | Banyak berkas tumpang tindih dengan `program/jetson/`; jangan menganggap keduanya satu implementasi otomatis |
| `program/sim-viz/` | Server inferensi visualisasi model dan UI simulasi | Memuat `model_fp32_480.onnx`; terpisah dari loop robot |
| `program/arena-3d/` | Simulasi arena 3D, capture frame dan label sintetis, alat ekstraksi lintasan | Sumber validasi/test dataset; kamera simulasi tidak menggantikan pengujian nyata |
| `electronic/` | Catatan wiring, daya, keselamatan, pin map | Pin map, skematik, arus dan rating aktual belum dicatat |
| `mechanical/3d design/` | Sumber CAD Autodesk Inventor, assembly dan versi lama | Buka assembly dari workspace yang mempertahankan seluruh referensi file |
| `mechanical/3d print/` | Ekspor mesh/STEP untuk manufaktur | Verifikasi orientasi, toleransi dan revisi terhadap assembly sumber sebelum mencetak |
| `docs/` | Aturan lomba, keputusan, catatan dataset dan panduan ini | Beberapa catatan proyek masih berbahasa Inggris dan merupakan keputusan kerja, bukan selalu spesifikasi final |

Dokumen untuk mulai: `README.md`, `docs/KSTI-2026-MAV-GUIDEBOOK-RECAP.md`, `docs/project-notes/project-ksti-2026-mav.md`, lalu README subfolder sesuai tugas.

## 3. Arsitektur runtime Jetson

Aliran yang dituju:

`kamera CSI → OnnxDetector → LaneEstimator → Observation → Controller → Command → McuLink/serial → MCU`

Log lokal ditulis ke direktori run; stream MJPEG memakai frame capture yang sama agar satu kamera CSI tidak dibuka dua proses.

- `main.py`: entry point, self-check, pilihan `--stream-only`, state machine dan loop utama.
- `core.py`: `Config`, tipe `Observation`/`Command` dan state (`BOOT`, `READY`, `RUNNING`, `DEGRADED`, `STOPPED`, `FAULT`). `Config.load()` menolak key yang tidak dikenal dan membatasi rentang perintah.
- `camera.py`: buka kamera menggunakan OpenCV `CAP_GSTREAMER`; pipeline kosong memilih pipeline Jetson `nvarguscamerasrc` bawaan. Frame yang tidak sesuai ukuran konfigurasi ditolak.
- `vision.py`: memuat ONNX melalui OpenCV DNN, tetapi saat ini hanya mengembalikan metadata `raw_shape`; belum membaca kelas, kotak/mask, garis, atau confidence sesungguhnya. `LaneEstimator` mengembalikan observasi tanpa geometri lajur.
- `controller.py`: hanya membuat perintah gerak ketika posisi lajur, heading, dan confidence tersedia. Koefisien steering belum dituning untuk kendaraan.
- `mcu.py`: menulis satu baris ASCII `speed_mps,steering_rad\n`, empat angka desimal, 115200 baud. Belum membaca status, fault, E-stop, atau handshake dari MCU.
- `telemetry.py`: catat baris CSV dan event JSONL per percobaan.
- `stream.py`: berbagi frame terbaru; encode JPEG hanya saat ada pemirsa dan dibatasi `stream_fps`. Endpoint `/`, `/stream`, `/snapshot`.
- `dashboard.py` + `dashboard.html`: HTTP read-only untuk log, default bind `127.0.0.1:18765`. Endpoint `/api/status` dan `/api/demo`; demo berisi data sintetis yang ditandai sebagai demo.

Loop target default 60 Hz bukan jaminan kamera atau inferensi berjalan 60 Hz. Ukur timestamp capture, inferensi, kontrol dan pengiriman perintah terpisah sebelum menetapkan batas kecepatan.

## 4. Perubahan lokal Jetson yang ditemukan

Pemeriksaan SSH ke `kazero@10.14.68.231`, checkout `/home/kazero/mizuro-ksti2026-muv`, dilakukan 22 Oktober 2026. **Ini catatan keadaan saat pemeriksaan, bukan bukti bahwa alamat DHCP akan tetap sama.**

Di Jetson, branch `main` berada di `2cac96a` dan tertinggal satu commit remote (`024bf2e`, penambahan assembly differential/gearbox/ekspor 3D print). File terlacak yang berubah lokal:

- `program/jetson/camera.py`: cek backend GStreamer, pipeline CSI bawaan, dan pesan error lebih jelas.
- `program/jetson/core.py`: konfigurasi stream dan validasi port, FPS, kualitas serta resolusi.
- `program/jetson/main.py`: stream terintegrasi, mode `--stream-only`, dan self-check tambahan.
- `program/jetson/robot.example.json`: parameter stream.
- `program/jetson/README.md`: instruksi penggunaan stream dan kamera.

File tak terlacak yang terlihat:

- `program/jetson/stream.py`: implementasi server MJPEG.
- `program/jetson/robot.json`: konfigurasi lokal. File ini menunjukkan `dry_run: true`, stream aktif, bind `0.0.0.0`, port 8080, 60 fps, kualitas JPEG 90, resolusi stream 1280x720. Periksa file langsung sebelum memakai; jangan menimpa konfigurasi kerja tim.
- `.vscode/settings.json` dan `program/jetson/error message`: file lokal, bukan sumber implementasi kanonis.

**Jangan jalankan `git pull`, `git reset`, `git checkout`, `git clean`, atau menimpa file Jetson sebelum perubahan tersebut disalin/dicadangkan dan pemiliknya menyetujui penggabungan.** Tidak ada perubahan yang dilakukan ke Jetson selama audit ini.

### Status layanan kamera pada saat audit

Sebuah proses `python3 main.py --stream-only --config robot.json` sempat tampak berjalan, tetapi port 8080 tidak mendengarkan ketika diperiksa. Event terbaru pada `program/jetson/runs/run-20261022-132152/events.jsonl` menunjukkan stream mulai lalu berhenti karena `camera frame read failed`. Layanan `mizuro-stream.service` tidak ditemukan melalui pemeriksaan ini, walau catatan konfigurasi lama menyebut pernah dibuat. Verifikasi ulang dengan `systemctl` dan log sebelum mengandalkan boot otomatis. Penyebab kegagalan baca frame belum dipastikan.

Self-check pada versi Jetson saat itu:

```sh
cd ~/mizuro-ksti2026-muv/program/jetson
/usr/bin/python3 main.py --self-check
/usr/bin/python3 dashboard.py --self-check
/usr/bin/python3 -m py_compile main.py camera.py core.py stream.py
```

Ketiganya lolos saat pemeriksaan. Itu hanya memverifikasi logika dasar/sintaks, bukan akses kamera CSI, koneksi MCU, keselamatan aktuator, atau operasi lomba. Kamera terdeteksi sebagai IMX219 di `/dev/video0`; perangkat serial yang terlihat mencakup `/dev/ttyTHS1` dan `/dev/ttyTHS2`, tetapi belum dipastikan mana yang terhubung ke MCU.

## 5. Cara menjalankan dengan aman

Pada Jetson, gunakan OpenCV JetPack dari `/usr/bin/python3`, bukan `.venv` repositori yang memakai OpenCV pip tanpa backend GStreamer. Jangan menjalankan dua proses yang sama-sama membuka kamera CSI.

```sh
cd ~/mizuro-ksti2026-muv/program/jetson
/usr/bin/python3 main.py --self-check
/usr/bin/python3 dashboard.py --self-check
/usr/bin/python3 main.py --config robot.example.json
```

Contoh config menjalankan dry-run dan tidak mengirim perintah gerak. Buat/ubah `robot.json` berdasarkan config yang disetujui tim, jangan simpan credential di Git. `--stream-only` memang membuka kamera tetapi tidak meng-arm robot:

```sh
/usr/bin/python3 main.py --stream-only --config robot.json
```

Stream dan dashboard hanya untuk bench/pit tepercaya. Stream default dapat bind ke `0.0.0.0`; dashboard default hanya localhost, dan opsi `--host 0.0.0.0` mengekspos log tanpa autentikasi. Hentikan keduanya sebelum percobaan lomba. Jangan menghentikan proses berdasarkan pola `pkill -f` yang bisa mencocokkan command shell pemanggil; identifikasi PID dan executable dahulu.

## 6. Batas MCU dan keselamatan

`program/body-mcu/body_controller.ino` menggunakan pin placeholder, serial 115200, timeout watchdog 100 ms, PWM motor dan servo kemudi. E-stop dibaca sebagai input active-low. Sketsa ini **belum** mengimplementasikan tombol start fisik, state run/checkpoint, timer lomba 300 detik, handshake Jetson-ready, status MCU balik, encoder atau IMU.

Ada selisih penting antara target desain dan contoh firmware: catatan proyek memilih STM32F401/F411 Blackpill, tetapi sketsa memakai `Arduino.h`, `Servo.h` dan contoh perintah Arduino Uno untuk uji sintaks. Konfigurasi driver `external_esc` juga tidak otomatis cocok dengan keluaran `PWM + arah` H-bridge. Untuk ESC RC, sinyal throttle dan rentang netral harus dirancang khusus, diverifikasi dengan datasheet dan diuji tanpa motor/roda menyentuh tanah.

E-stop harus memutus daya/enable penggerak secara fisik dan independen dari Jetson. Watchdog MCU harus membuat output mati pada boot, data malformed/stale, kehilangan serial, dan fault. Input perangkat lunak bukan pengganti E-stop fisik.

## 7. Urutan kelanjutan yang disarankan

1. **Amankan dan satukan sumber.** Salin patch Jetson ke branch/patch terpisah atau commit di perangkat setelah direview. Ambil commit remote terbaru secara aman. Bandingkan `program/jetson/` dengan `program/robot-fw/`; putuskan mana implementasi kanonis dan hapus duplikasi hanya melalui keputusan tim, bukan saat merge terburu-buru.
2. **Pulihkan kamera bench.** Pastikan satu proses pemilik CSI, jalankan stream foreground dengan `/usr/bin/python3`, periksa log `events.jsonl`, endpoint `/snapshot`, dan stabilitas capture selama durasi yang disepakati. Catat sensor, pipeline, resolusi/FPS aktual, suhu dan konsumsi CPU. Jangan menganggap stream 60 fps terukur sebelum pengukuran.
3. **Tentukan antarmuka MCU.** Tetapkan board persis, driver/ESC persis, pin map, satuan, batas, format paket, timeout, status balik, start dan E-stop. Tulis tes parser dan uji watchdog dengan output motor terlepas.
4. **Selesaikan persepsi offline.** Pastikan model ONNX yang dipakai dan layout outputnya. Implementasikan decoder berbasis format model yang terverifikasi, kalibrasi kamera, proyeksi ground-plane, estimasi garis tengah, confidence dan deteksi kehilangan lajur. Bandingkan pada frame/ground truth dari `program/arena-3d/` sebelum kontrol motor.
5. **Buat pengujian controller tanpa hardware.** Uji tanda error, satuan, saturasi steering/speed, perilaku confidence rendah, inferensi lambat dan frame stale. Tambahkan test otomatis yang bisa dijalankan di komputer pengembang tanpa kamera.
6. **Integrasikan di bench dengan roda terangkat.** Uji setiap arah motor, titik netral ESC, batas servo, polaritas, E-stop fisik, watchdog, reset, kehilangan komunikasi dan shutdown. Mulai dari sumber arus terbatas dan kecepatan rendah.
7. **Validasi kendaraan dan aturan.** Ukur rasio drivetrain, roda, radius belok, traksi, getaran kamera, baterai/tegangan dan temperatur. Konfirmasi interpretasi aturan (differential terbuka, lintasan, penalti, timer) pada Technical Meeting.
8. **Baru lakukan uji lintasan bertahap.** Catat tiap revisi hardware/software dan run lokal. Matikan semua stream, dashboard jarak jauh dan tether sebelum attempt resmi. Tetapkan acceptance criteria tertulis sebelum menaikkan kecepatan.

## 8. Alur kerja kontribusi tim

- Sebelum mengedit, periksa `git status`, branch, versi board/hardware, serta apakah file yang akan diubah punya perubahan lokal di Jetson.
- Satu perubahan per branch/PR: kamera, persepsi, controller, firmware, elektronik, mekanik. Sertakan alasan, cara uji, hasil uji dan batas yang belum diuji.
- Jangan memasukkan `robot.json`, log run, file credential, cache `__pycache__`, atau hasil sementara kecuali disetujui sebagai artefak proyek.
- Simpan CAD sumber beserta dependensi assembly dan ekspor dengan nama/revisi/material/satuan yang jelas. Ekspor cetak bukan pengganti sumber Inventor.
- Dokumentasikan klaim performa sebagai hasil ukur dengan kondisi, alat, jumlah pengulangan dan log; jangan mencatat perkiraan sebagai angka hasil.
- Untuk perubahan protokol, koordinasikan Jetson dan MCU dalam satu perubahan antarmuka dan uji parser pada kedua sisi.

## 9. File rujukan utama

- `README.md`: tujuan, urutan run yang dituju, batas keselamatan dan target pengembangan.
- `program/jetson/README.md`: penggunaan runtime dan konfigurasi Jetson.
- `program/body-mcu/README.md`: batas sketch dan wiring yang belum dipastikan.
- `program/arena-3d/README.md`: simulator dan ekspor data sintetis.
- `docs/KSTI-2026-MAV-GUIDEBOOK-RECAP.md`: ringkasan peraturan serta isu yang perlu dikonfirmasi.
- `docs/project-notes/project-ksti-2026-mav.md`: keputusan desain dan pertanyaan terbuka hardware/mekanik.
- `docs/project-notes/reference-ksti-arena3d-dataset.md`: struktur dataset/ground truth.
- `docs/project-notes/reference-ksti-model-sim.md`: catatan simulasi model.
- `mechanical/README.md` dan `electronic/README.md`: status CAD dan dokumentasi wiring.
