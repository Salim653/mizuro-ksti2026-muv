# Body controller berbasis Arduino-compatible

Buka `body_controller.ino` di Arduino IDE atau jalankan `arduino-cli compile --fqbn arduino:avr:uno .` dari folder ini. Sketsa membaca `speed_mps,steering_rad\n` lewat serial pada 115200 baud, punya penghentian di sisi MCU saat perintah basi, dan membaca input e-stop active-low. Target Uno hanya untuk cek sintaks/build; konfirmasi MCU dan pin mapping yang diingat sebelum flashing.

Assignmen pin masih placeholder. Cocokkan dengan datasheet board dan driver sebelum mengawinkan. Sketsa memakai output PWM + arah generik untuk H-bridge. Untuk hardware yang direncanakan (motor 540 + RC ESC), gunakan protokol servo-PWM yang didukung ESC, jangan sambungkan input sinyalnya ke pin PWM/DIR driver motor DC. Jaga Jetson tetap terisolasi dari arus motor dan daya penggerak.

Sebelum pengujian: roda penggerak diangkat, sumber arus berbatas, e-stop sudah terbukti mematikan output motor, dan tanpa pengawasan. Pemetaan speed ke PWM serta titik tengah/ujung kemudi perlu kalibrasi di sasis nyata. Sampling encoder/IMU dan telemetri status MCU belum ada di sketsa ini.
