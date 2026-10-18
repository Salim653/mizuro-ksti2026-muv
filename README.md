# MIZURO KSTI 2026 — Mini Autonomous Vehicle

Repositori robot MUV untuk KSTI 2026. Workspace lama tidak diubah; repositori ini berisi salinan kerja yang dirapikan per disiplin.

## Struktur repositori

- `electronic/` — arsitektur elektronika, pengkabelan, daya, dan antarmuka keselamatan.
- `program/jetson/` — runtime Jetson Orin Nano: kamera, persepsi, kontroler, telemetri.
- `program/body-mcu/` — firmware MCU bodi dan antarmukanya dengan Jetson.
- `mechanical/` — sasis, drivetrain, kemudi, dudukan kamera, dan ekspor CAD.
- `docs/` — guidebook, keputusan, serta catatan integrasi dan pengujian.

## Urutan operasi robot yang dituju

1. Saat dinyalakan, output motor dalam keadaan mati. Jetson dan MCU bodi boot; MCU menahan output motor di nol.
2. Operator menekan tombol **start** fisik yang diwajibkan. MCU hanya menerima start jika e-stop tidak aktif dan runtime Jetson siap. Navigasi lalu berjalan mandiri; tidak ada tether maupun telemetri selama satu percobaan.
3. Selama bernavigasi, estimasi lajur dari kamera mengarahkan kemudi. Hilangnya persepsi valid meminta kecepatan nol; watchdog perintah di MCU yang independen menghentikan penggerak jika setpoint valid berhenti datang.
4. **Emergency stop** fisik memutus penggerak secara independen dari software Jetson. Statusnya terkunci sampai direset dengan sengaja.
5. Untuk restart atas instruksi juri, posisikan mobil di checkpoint terakhir, reset e-stop sesuai arahan, lalu tekan start lagi. Navigasi dilanjutkan dari indeks checkpoint yang disimpan untuk percobaan tersebut.
6. Berhenti otomatis pada batas 300 detik, saat penyelesaian terkonfirmasi, saat fault keselamatan terkunci, atau saat e-stop fisik ditekan.

## Status implementasi saat ini

File `program/` yang disalin adalah snapshot prototipe, bukan implementasi siap lomba. Runtime Python saat ini memakai gate Enter di konsol; sketsa MCU yang disalin berbasis Arduino-compatible dan belum punya input start fisik, state checkpoint, maupun timer run. Catatan proyek menetapkan STM32F401/F411 Blackpill sebagai MCU bodi yang dituju, sehingga firmware dan protokol serial masih harus diselaraskan sebelum integrasi hardware. Jangan menganggap sketsa prototipe yang disalin sudah tervalidasi, dan jangan mem-flash-nya ke robot lomba tanpa penyesuaian port, pengkabelan, dan tinjauan keselamatan.

## Persyaratan keselamatan

- E-stop harus mematikan penggerak motor/ESC secara fisik, independen dari Jetson, dan mudah dijangkau.
- Watchdog MCU harus berdefault ke penggerak mati saat reset, perintah cacat, perintah basi, atau fault MCU.
- Jangan mulai dengan roda menyentuh tanah sebelum e-stop, watchdog, polaritas motor, batas kemudi, dan perilaku kehilangan perintah diuji di meja dengan roda penggerak terangkat.
- Tidak ada komunikasi nirkabel maupun kabel selama percobaan lomba.

## Target pengembangan pertama

Implementasikan dan uji state machine serta interlock MCU bodi lebih dulu: `BOOT_SAFE → READY → RUNNING`, dengan e-stop/timeout/fault selalu diutamakan dan wajib restart fisik yang disengaja. Setelah itu tambahkan handshake Jetson-ready, batas 300 detik, persistensi/resume checkpoint, dan deteksi finish. Jaga runtime tetap bisa diuji tanpa hardware.
