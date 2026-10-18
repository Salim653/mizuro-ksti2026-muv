# Elektronika

Catat pengkabelan yang benar-benar terpasang dan terverifikasi di sini. Jangan menyimpulkan pinout konektor dari konstanta di software.

## Blok yang diperlukan

- Jetson Orin Nano + carrier dan kamera
- MCU bodi: keputusan proyek adalah STM32F401/F411 Blackpill (verifikasi board persisnya sebelum menetapkan pin)
- Motor/ESC dan servo kemudi
- Baterai, regulator, ground, sekring, dan distribusi daya
- Tombol start dan emergency-stop fisik yang terpisah dan mudah dijangkau

## Persyaratan antarmuka keselamatan

- E-stop harus memutus penggerak secara independen dari software Jetson dan tetap terkunci sampai direset dengan sengaja.
- MCU berdefault ke motor mati saat reset dan saat setpoint dari Jetson basi atau tidak valid.
- Start hanya diterima jika e-stop tidak aktif dan sistem siap; setelah e-stop, wajib ada aksi start baru.
- Tidak ada komunikasi kabel maupun nirkabel selama satu percobaan.

Pin map, skematik, ukuran kawat, nilai sekring, rating tegangan/arus, dan hasil pengukuran sengaja dibiarkan kosong sampai komponen dan pengkabelan nyata dikonfirmasi.
