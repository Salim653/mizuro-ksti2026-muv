# 📋 KSTI 2026 — Mini Autonomous Vehicle · Ringkasan Guidebook

> ⚡ **TL;DR** — Lintasan loop tertutup, lebar 50 cm, durasi 5 menit per percobaan, 3 percobaan, skor terbaik yang diambil. Sensor infrared/line-follower **dilarang**, jadi deteksi lajur wajib berbasis kamera. Skor = `(1 + sisa_waktu/300) × checkpoint`, dan total akhir dikalikan `(1.25 jika kemudi Ackerman) × (1 + skor_challenge)`. **Checkpoint adalah suku dominan** — satu lap yang melewatkan satu checkpoint jauh lebih mahal daripada lap lambat yang mengenai semuanya.
>
> **Aksi berikutnya** — Konfirmasi stack sensor terhadap daftar larangan (lihat §8), lalu ukur panjang lintasan sebenarnya dari `track_map_200dpi.png` untuk menetapkan target kecepatan.

---

## 📅 1 · Fakta acara

| Item | Nilai |
|---|---|
| Nama | Mini Autonomous Vehicle — Robotics Competition at KSTI 2026 |
| Penyelenggara | Institut Teknologi Sepuluh Nopember (ITS), Surabaya |
| Tanggal & lokasi | **12 November 2026**, Hall Robotika ITS (offline) |
| Technical Meeting | **8 November 2026** — konfirmasi registrasi + pembagian grup diumumkan di sini |
| Kelayakan | Mahasiswa (wajib KTM); maksimal **5 orang** per tim, 1 ketua tim |
| Batas tim/institusi | Tidak dibatasi — beberapa tim dari kampus yang sama diperbolehkan |
| Biaya pendaftaran | Rp150.000 (batch 1) / Rp175.000 (batch 2) · **tidak dapat dikembalikan** |
| Pendaftaran | https://its-robotics.com/ → KSTI 2026 → Mini Autonomous Vehicle |
| SLA verifikasi | Maksimal 2×24 jam setelah submit |
| Hadiah | Juara 1 emas + Rp4.000.000 · Juara 2 perak + Rp3.000.000 · Juara 3 perunggu + Rp2.000.000 |
| Kontak | Nindita +62 896-4920-8124 · Yuke +62 857-5577-3985 |

⚠️ Nominal hadiah ditulis "Rp4.00.000,00" di PDF. Dibaca sebagai Rp4.000.000 — verifikasi di technical meeting.

---

## 🤖 2 · Aturan robot

**Platform**
- ⛔ Mobil berbahan bakar dilarang → **listrik saja**, skala 1:10.
- ✅ Wajib punya minimal satu as kemudi.
- 🔴 **Tombol `emergency stop` fisik** (mudah dijangkau) **dan tombol `start` terpisah** yang memulai navigasi. Keduanya termasuk aturan yang diperiksa.
- Satu mobil / satu platform per tim. Ganti platform = dilarang; perbaikan/modifikasi komponen = diperbolehkan.

**Daya** — Tidak ada batas jumlah maupun kapasitas baterai.

**Penggerak & kemudi**
- 2WD atau 4WD diperbolehkan.
- ⛔ Tidak boleh omni, mecanum, atau konfigurasi roda yang membuat robot bisa bergerak segala arah.
- ✅ Sistem penggerak differential **wajib**.
- Sistem kemudi selain itu bebas.
- 💎 **Kemudi Ackerman mendapat STP = 0.25** (satu as atau dua as). Lihat §5 — ini *pengali*, bukan bonus.

**Dimensi (mm)**

| Dimensi | Min | Max |
|---|---|---|
| Lebar jejak roda | 130 | 350 |
| Wheel base | 160 | 550 |
| Tinggi | — | 450 |
| Lebar (termasuk seluruh part terpasang) | — | 400 |
| Panjang (termasuk seluruh part terpasang) | — | 600 |

**Sensor**
- ⛔ **Sensor infrared dan/atau line-follower dilarang.**
- ✅ Sensor lain yang *bukan* sensor penjejak garis langsung diperbolehkan.
- 📡 **Semua komunikasi dilarang selama run** — nirkabel *maupun* kabel. Tidak ada fallback RC, tidak ada tether telemetri, tidak ada koneksi laptop. Sepenuhnya standalone setelah tombol start ditekan.

---

## 🛣️ 3 · Lintasan

- Lebar **50 cm ± 10%**, dan **tidak konstan** sepanjang lap.
- Radius belok minimum **1.5 m ± 10%** (radius luar).
- Marka: **dua garis tepi lurus solid** (putih **atau** hitam) + **satu garis tengah putus-putus** dengan warna sama seperti tepi.
- Lebar marka ≥ 1 cm · jarak antar dash ≥ 5 cm.
- Warna marka dipilih menyesuaikan lantai arena → **siapkan untuk putih-di-gelap maupun hitam-di-terang**.
- Checkpoint ditempatkan di sepanjang lintasan, ditampilkan **hijau**; start/finish memakai warna berbeda dari marka jalan (digambarkan merah, **tidak dijamin**). Sebuah checkpoint boleh **tanpa marka sama sekali** — peta hanya ilustrasi.
- 📎 Aset lokal di folder ini: `track_map_200dpi.png` (juga `_small.png`, `track_map_raw.bin`) — loop tertutup dengan layout long-straight berbelok S, lantai gelap, tepi putih, garis tengah putus-putus, logo "KSTI 26" di infield. Tidak ada garis start maupun checkpoint yang digambar. Lihat §7.

---

## 🏁 4 · Format & alur run

Dua tahap: **Eliminasi** (harus melewati skor minimum untuk lolos) → **Final**. **Challenge** bersifat opsional dan hanya dijalankan di final.

Setiap percobaan, kedua format memakai alur yang sama:

1. Berada di area start **5 menit sebelum** slot — terlambat = percobaan gugur.
2. Dipanggil → **10 menit persiapan**: penyesuaian kecil, kalibrasi sensor, review strategi, perbaikan.
3. Persiapan selesai → **jendela percobaan 5 menit**. Letakkan mobil di garis start, tekan start.
4. 🚫 **Setelah menekan start, tidak ada yang boleh menyentuh robot.** Kontak apa pun mengakhiri percobaan.
5. Saat pelanggaran: juri memanggil → tekan `emergency`, mobil dipindah ke **checkpoint terakhir**, tekan `start` lagi, **tanpa jeda waktu**.
6. Maksimal **3 pelanggaran per percobaan** — pelanggaran ke-4 mengakhiri percobaan.
7. Part lepas/jatuh = pelanggaran **dan run berakhir seketika**.
8. Tidak selesai dalam 5 menit → hentikan mobil; skor hanya menghitung progres yang tercapai.
9. **Terbaik dari 3 percobaan** menjadi hasil akhir.

| | Autonomous Race | AR Challenge |
|---|---|---|
| Checkpoint | Marka hijau di lintasan, dilewati berurutan | Checkpoint **adalah obstacle** yang harus dihindari; menghindarinya = melewatinya |
| Ambang keluar lintasan | **30%** bodi mobil di luar | **50%** bodi mobil di luar |
| Ketersediaan | Semua babak | Final saja, opsional |

---

## 🧮 5 · Penilaian

**Autonomous Race**

```
S_AR = (1 + max{(T_stage − T_total)/T_stage, 0}) × cp
```
`T_stage` = 300 s · `T_total` = waktu tempuh **termasuk penalti** · `cp` = checkpoint yang dilewati.

**AR Challenge** — bentuk sama, lalu dinormalisasi:

```
S_ARC = [(1 + max{(T_stage − T_total)/T_stage, 0}) × cp] / (2 × Tcp)
```
`Tcp` = total checkpoint di lintasan. Kontribusi maksimum ≈ **0.5**.

**Total akhir**

```
Total = S_AR × (1 + STP + S_ARC)          STP = 0.25 if Ackerman
```

**Waktu penalti yang ditambahkan ke `T_total`** (semuanya dinyatakan sebagai fraksi waktu stage, dibagi jumlah checkpoint):

| Pelanggaran | Waktu tambahan |
|---|---|
| Checkpoint terlewat | `0.5 × T_stage / N_all_checkpoints` |
| Part lepas | `0.2 × T_stage / N_all_checkpoints` |
| Keluar lintasan | `0.8 × T_stage / N_all_checkpoints` |

📐 **Kenapa matematikanya penting** — misal 10 checkpoint, `T_stage/N_cp` = 30 s, jadi satu kejadian keluar lintasan berbiaya **+24 s** dan satu checkpoint terlewat berbiaya **+15 s** *sekaligus* menurunkan `cp` sebesar 1. Hitungannya:

| Skenario | cp | T_total | S_AR |
|---|---|---|---|
| Lap cepat bersih | 10 | 150 s | 15.0 |
| Lap lambat bersih | 10 | 250 s | 11.7 |
| Cepat tapi sekali keluar lintasan | 10 | 174 s | 14.2 |
| Cepat tapi satu checkpoint terlewat | 9 | 165 s | 12.5 |
| Cepat tapi 3× keluar lintasan (batas) | 10 | 222 s | 13.1 |

➡️ **Kecepatan bernilai ~1.3 poin per 30 s yang dihemat; satu checkpoint terlewat bernilai ~2.5 poin hilang.** Keandalan pada kecepatan moderat mengalahkan kecepatan mentah. Itulah design brief-nya.

💎 Ackerman adalah kemenangan termurah yang tersedia: mengalikan seluruh skor dengan 1.25 tanpa usaha tambahan per run.

---

## 🚩 6 · Juri, protes, dan fine print

- Juri/referee ditunjuk panitia; keputusan bersifat **final dan absolut**.
- Protes: hanya dari anggota tim, dengan sopan, **langsung setelah** pertandingan dan sebelum pertandingan berikutnya dimulai; tidak pernah saat run berlangsung. ⛔ Protes mengenai keputusan pelanggaran atau penilaian **tidak diterima** — hanya dugaan pelanggaran aturan robot atau prosedur.
- Panitia dapat mengubah aturan ini; perubahan diumumkan lewat platform pendaftaran dan sosial media resmi. Mendaftar = menerima guidebook.
- Panitia tidak bertanggung jawab atas properti peserta yang hilang/rusak.

---

## 🐞 7 · Inkonsistensi guidebook yang perlu diangkat di Technical Meeting

1. 🏷️ Judul bagian hadiah tertulis **"Humanoid Penalty Kick"** — copy-paste dari kategori lain.
2. 📅 §II.A.7 menyebut peserta harus mematuhi aturan **"KSTI 2025"**.
3. 💰 Nominal hadiah ditulis "Rp4.00.000,00" (pengelompokan digit ambigu).
4. 📏 Ambang keluar lintasan berbeda antar format (30% vs 50%) tanpa alasan tertulis — konfirmasi mana yang berlaku di final.
5. ❓ **Skor minimum kelayakan** untuk lolos eliminasi disebut tetapi tidak pernah dinyatakan.
6. ❓ `N_all_checkpoints` dan `Tcp` tidak didefinisikan secara numerik — seluruh skala penalti bergantung padanya.
7. 🎨 Warna checkpoint/garis start hanya "demonstration only" → tanyakan apakah palet final sudah tetap sebelum acara.
8. 🖥️ "Tombol lain untuk memulai navigasi" + "no touching after start" menyiratkan tombol start harus dapat dijangkau tanpa membuka chassis — tanyakan bagaimana mereka memeriksanya.

---

## 👁️ 8 · Implikasi persepsi (kenapa urusan kamera penting)

Larangan IR/line-follower menghapus solusi murah sepenuhnya: **lane keeping harus berasal dari kamera RGB + software**, karena hanya sensor non-penjejak-garis yang diperbolehkan.

- 🎯 **Lane keeping tidak butuh depth.** Kamera menghadap ke bawah memberikan garis tengah dan tepi sebagai geometri di bidang citra; fit lajur, lalu kemudikan. Stereo/depth tidak memberi nilai tambah di sini.
- 📐 **Anggaran resolusi:** marka lebarnya ≥ 1 cm pada lintasan 50 cm. Memetakan ~70 cm melintasi lajur ke sensor 640 px berarti ≈ 0.9 mm/px — marka 1 cm mencakup ~11 px, cukup. Lensa 1/4" pada tinggi pemasangan 20–40 cm menutupinya dengan nyaman.
- 🚧 **Di mana depth *benar-benar* berguna:** pada Challenge, di mana checkpoint adalah obstacle fisik yang harus dihindari. Marka selotip hijau datar dan objek padat di lajur terlihat identik bagi satu kamera top-down. Membedakannya butuh ketinggian di atas lantai, pandangan samping/depan, atau dua sudut pandang.
- 📷 **Pembagian dua kamera yang praktis** bukan stereo — melainkan **satu kamera miring ke depan** untuk jalur jarak jauh dan penemuan obstacle, plus **satu kamera menghadap ke bawah** untuk centering lajur jarak dekat yang presisi di mana pandangan depan kehilangan resolusi. Fixed focus cukup untuk keduanya, karena masing-masing punya jarak kerja tetap dan DOF dalam pada skala itu.
- 🗺️ **Peta sudah diketahui.** `track_map_200dpi.png` + `arena-3d/track_data.json` berarti geometri lintasan bisa dimuat lebih dulu, sehingga odometri/IMU dapat difusikan dengan visi untuk penghitungan checkpoint dan untuk pulih setelah direset ke checkpoint terakhir.
- 💡 **Pembeda obstacle termurah:** kamera depan plus model lintasan yang sudah diketahui menjawab "apakah blob ini ada di lajur saya, dan apakah ia berada di atas bidang lantai?" tanpa perlu stereo metrik sama sekali.

---

## 📎 9 · File di direktori ini

| File | Isinya |
|---|---|
| `GUIDEBOOK PESERTA MINI AUTONOMOUS VEHICLE KSTI 2026.pdf` | Dokumen sumber, 11 halaman (Bab I–IV). Halaman 1 = cover, tanpa text layer. |
| `track_map_200dpi.png` / `track_map_small.png` / `track_map_raw.bin` | Contoh peta lintasan dalam tiga resolusi. |
| `arena-3d/` | `extract_track.py`, `track_data.json` / `.js`, `track_floor.jpg`, `sanity_overlay.png`, `three.min.js` — rekonstruksi arena 3D dari peta. |

---

## ✅ 10 · Checklist aksi

| # | Tugas | Penanggung jawab | Tenggat |
|---|---|---|---|
| 1 | Daftar di its-robotics.com, unggah KTM + bukti bayar | Ketua tim | sebelum batch 2 tutup |
| 2 | Konfirmasi keputusan sasis Ackerman (bernilai ×1.25) | Mekanikal | sebelum build freeze |
| 3 | Uji bench stack visi terhadap rumusan larangan IR/line-follower | Software | sebelum 8 Nov |
| 4 | Bawa daftar pertanyaan §7 ke Technical Meeting | Ketua tim | 8 Nov 2026 |
| 5 | Kalibrasi untuk marka putih-di-gelap dan hitam-di-terang | Software | sebelum Test Day |
| 6 | Tetapkan target lap time dari panjang lintasan terukur ÷ 5 menit dikurangi margin | Kontrol | setelah pengukuran peta |
