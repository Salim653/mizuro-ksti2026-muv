#!/usr/bin/env python3
"""Perekam dataset: kamera -> frame mentah + index.csv (timestamp).

Headless: preview lewat browser (MJPEG), BUKAN jendela OpenCV —
Jetson di robot tidak punya display.

Kontrol (kalau stdin adalah terminal, mis. lewat SSH):
    SPACE  = mulai / stop rekam (bisa beberapa take per sesi)
    Q      = keluar
Tanpa terminal (piped/nohup/systemd), pakai --frames/--duration;
Ctrl+C selalu bisa dipakai buat stop.

Kenapa frame MENTAH + CSV, bukan video mp4:
- mp4 (mp4v) lossy dan tanpa timestamp per frame; buat training
  YOLO/labeling, frame utuh + waktu jauh lebih berguna.
- Default PNG = lossless. JPEG chroma subsampling (4:2:0) menggeser
  hue — kalau nanti pakai detektor berbasis warna (checkpoint_mode
  "cv"), dataset PNG menjaga warna persis seperti yang robot lihat.
- t_wall (epoch) memungkinkan sinkronisasi dengan log lain kalau
  nanti ada rekaman bersamaan.

Frame disimpan APA ADANYA dari kamera (setelah undistort kalau ada
kalibrasi) — persis apa yang robot lihat di loop kontrol, supaya
tidak ada domain gap antara dataset dan runtime.

Pakai /usr/bin/python3 di Jetson (OpenCV JetPack + GStreamer);
.venv repo TIDAK punya GStreamer.

Contoh:
    # manual: SPACE mulai/stop, Q keluar; preview di :8080
    /usr/bin/python3 tools/record_dataset.py

    # berhenti otomatis setelah 200 frame (tetap bisa SPACE lebih awal)
    /usr/bin/python3 tools/record_dataset.py --frames 200

    # batas sesi 60 detik, ambil tiap frame ke-2 (kamera 60fps -> 30fps)
    /usr/bin/python3 tools/record_dataset.py --duration 60 --every 2

    # JPG kalau disk sempit (~3x lebih kecil & cepat tulis)
    /usr/bin/python3 tools/record_dataset.py --frames 100 --format jpg

    # tanpa server preview (port sibuk / mau senyap)
    /usr/bin/python3 tools/record_dataset.py --frames 100 --no-stream

    # paksa matikan keyboard (mis. dijalankan dari script lain)
    /usr/bin/python3 tools/record_dataset.py --frames 100 --no-keyboard

Output: runs/dataset-YYYYmmdd-HHMMSS/
    frames/frame-000001.png ...
    index.csv  (filename, t_monotonic_s, t_wall_s, width, height)
"""
from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path

# Jalankan sebagai skrip dari tools/: izinkan `import mizuro`.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Rekam dataset: frame mentah + index.csv timestamped")
    parser.add_argument("--config", type=Path, default=Path("robot.json"),
                        help="file config JSON (default: robot.json; "
                             "kalau tidak ada, pakai default Config())")
    parser.add_argument("--output", type=Path, default=None,
                        help="direktori output (default: "
                             "runs/dataset-<timestamp>)")
    parser.add_argument("--frames", type=int, default=0,
                        help="berhenti setelah N frame tersimpan "
                             "(0 = tanpa batas)")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="berhenti setelah N detik (0 = tanpa batas)")
    parser.add_argument("--every", type=int, default=1,
                        help="simpan setiap N frame yang terbaca "
                             "(1 = semua; 2 = setengah fps)")
    parser.add_argument("--warmup", type=float, default=1.0,
                        help="buang frame selama N detik pertama "
                             "(auto-exposure butuh waktu settle)")
    parser.add_argument("--format", choices=("jpg", "png"), default="png",
                        help="format frame (default: png lossless; "
                             "jpg ~3x lebih kecil & cepat tulis)")
    parser.add_argument("--quality", type=int, default=95,
                        help="kualitas JPG 1-100 (default: 95; "
                             "diabaikan untuk png)")
    parser.add_argument("--no-stream", action="store_true",
                        help="matikan server preview MJPEG")
    parser.add_argument("--no-keyboard", action="store_true",
                        help="matikan kontrol SPACE/Q (dipakai kalau "
                             "stdin bukan terminal)")
    args = parser.parse_args()
    if args.frames < 0 or args.duration < 0 or args.every < 1:
        parser.error("--frames/--duration tidak boleh negatif, "
                     "--every minimal 1")
    if not 1 <= args.quality <= 100:
        parser.error("--quality harus dalam [1, 100]")
    # Stop condition: batas argumen ATAU keyboard manual ATAU Ctrl+C.
    # Keyboard aktif kalau stdin terminal dan tidak dimatikan eksplisit.
    keyboard_ok = (not args.no_keyboard and sys.stdin.isatty())
    if args.frames == 0 and args.duration == 0.0 and not keyboard_ok:
        parser.error("set --frames atau --duration (atau jalankan di "
                     "terminal buat kontrol SPACE/Q, atau Ctrl+C buat "
                     "stop manual)")
    return args


def build_writer(fmt: str, quality: int):
    """Return fungsi tulis(frame, path) -> (ms, ok)."""
    import cv2
    if fmt == "png":
        def write(frame, path):
            t0 = time.perf_counter()
            ok = cv2.imwrite(str(path), frame)
            return (time.perf_counter() - t0) * 1000.0, ok
    else:
        params = [cv2.IMWRITE_JPEG_QUALITY, quality]

        def write(frame, path):
            t0 = time.perf_counter()
            ok = cv2.imwrite(str(path), frame, params)
            return (time.perf_counter() - t0) * 1000.0, ok
    return write


def _make_tty_reader():
    """Reader satu tombol non-blocking dari terminal (raw/cbreak mode).

    cbreak = matikan line-buffering + echo, TAPI ISIG tetap aktif
    (Ctrl+C masih raise KeyboardInterrupt). Return (read_key, restore);
    restore WAJIB dipanggil di finally, kalau tidak terminal rusak.
    """
    import os
    import select
    import sys
    import termios
    import tty

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd, termios.TCSANOW)

    def read_key():
        ready, _, _ = select.select([sys.stdin], [], [], 0)
        if not ready:
            return None
        return os.read(fd, 1)

    def restore():
        termios.tcsetattr(fd, termios.TCSANOW, old)

    return read_key, restore


def termios_error():
    """Tipe error termios (import lokal; di platform tanpa termios
    kembalikan Exception supaya except di Keyboard tetap sah)."""
    try:
        import termios
        return termios.error
    except ImportError:
        return Exception


class Keyboard:
    """Kontrol manual: SPACE = toggle rekam, Q = keluar.

    Butuh stdin berupa terminal (jalan lewat SSH tanpa X). Kalau bukan
    terminal (piped, nohup, systemd) -> disabled, poll() selalu None.
    `read_key` bisa disuntik buat tes.
    """

    def __init__(self, read_key=None) -> None:
        self.enabled = False
        self._read_key = read_key
        self._restore = None
        if read_key is not None:
            # Disuntik (tes): anggap terminal, langsung aktif.
            self.enabled = True
            return
        if not sys.stdin.isatty():
            return
        try:
            self._read_key, self._restore = _make_tty_reader()
            self.enabled = True
        except (OSError, ValueError, termios_error()):
            self.enabled = False

    def poll(self):
        """Return 'space' | 'quit' | None. Tidak pernah blocking."""
        if not self.enabled or self._read_key is None:
            return None
        try:
            ch = self._read_key()
        except OSError:
            return None
        if ch is None:
            return None
        if ch == b" ":
            return "space"
        if ch in (b"q", b"Q"):
            return "quit"
        return None

    def close(self) -> None:
        """Kembalikan terminal ke keadaan semula. Panggil di finally."""
        if self._restore is not None:
            try:
                self._restore()
            except OSError:
                pass
            self._restore = None
        self.enabled = False


def record_loop(*, cam, write_frame, frames_dir, args, hub=None,
                keyboard=None):
    """Loop rekam — kamera disuntik supaya bisa dites tanpa hardware.

    keyboard aktif = mulai dalam mode PREVIEW (belum rekam); SPACE
    toggle rekam/stop, Q keluar. Tanpa keyboard = langsung rekam.
    Beberapa take dalam satu sesi dibolehkan; penomoran frame dan
    index.csv menyatu across takes.

    Return (rows, stats). Berhenti kalau: --frames tercapai,
    --duration tercapai, kamera tidak mengembalikan frame, gagal
    nulis, Q, atau Ctrl+C.
    """
    rows: list[dict] = []
    saved = 0
    read_count = 0
    rec_reads = 0          # frame terbaca SELAMA rekam (buat --every)
    write_ms_total = 0.0
    takes = 0
    t_start = time.monotonic()
    t_warmup_end = t_start + args.warmup
    # Keyboard aktif = mulai dari PREVIEW; tanpa keyboard = langsung rekam.
    recording = not (keyboard is not None and keyboard.enabled)
    if not recording:
        print("PREVIEW — tekan SPACE untuk mulai rekam, Q untuk keluar")

    while True:
        try:
            now = time.monotonic()
            if args.duration and now - t_start >= args.duration:
                print(f"\ndurasi {args.duration:.0f}s tercapai")
                break
            if args.frames and saved >= args.frames:
                print(f"\n{args.frames} frame tercapai")
                break

            frame = cam.read()
            if frame is None:
                print("\nERROR: kamera tidak mengembalikan frame")
                break
            read_count += 1

            if hub is not None:
                hub.publish(frame)

            if keyboard is not None:
                key = keyboard.poll()
                if key == "quit":
                    print("\nQ — keluar")
                    break
                if key == "space":
                    recording = not recording
                    if recording:
                        takes += 1
                        print(f"\nREKAM take {takes} — SPACE untuk stop")
                    else:
                        print(f"STOP take {takes} — {saved} frame total "
                              f"tersimpan; SPACE untuk take baru")

            if not recording:
                continue

            # Warmup: exposure/white-balance belum settle — frame awal
            # sering gelap/berubah. Buang.
            if now < t_warmup_end:
                continue
            rec_reads += 1
            if (rec_reads - 1) % args.every != 0:
                continue

            name = f"frame-{saved + 1:06d}.{args.format}"
            path = frames_dir / name
            ms, ok = write_frame(frame, path)
            if not ok:
                print(f"\nERROR: gagal menulis {path}")
                break
            write_ms_total += ms
            saved += 1
            rows.append({
                "filename": name,
                "t_monotonic_s": round(now - t_start, 6),
                "t_wall_s": round(time.time(), 6),
                "width": int(frame.shape[1]),
                "height": int(frame.shape[0]),
            })

            if saved % 30 == 0:
                elapsed = now - t_start
                fps = saved / max(elapsed - args.warmup, 1e-6)
                print(f"[{saved:5d}] frame  {fps:5.1f} fps data  "
                      f"tulis {ms:5.1f} ms  total {elapsed:6.1f}s")
        except KeyboardInterrupt:
            # Ctrl+C di titik mana pun (termasuk cam.read()) tetap
            # mengembalikan frame yang SUDAH tersimpan — index.csv
            # tidak boleh kosong kalau file sudah ada di disk.
            print(f"\nCtrl+C — stop setelah {saved} frame")
            break

    stats = {"saved": saved, "read_count": read_count,
             "write_ms_total": write_ms_total, "takes": takes,
             "elapsed_s": time.monotonic() - t_start,
             "warmup_s": args.warmup}
    return rows, stats


def main() -> None:
    args = parse_args()
    import cv2

    from mizuro.camera import Camera
    from mizuro.config import Config
    from mizuro.perception.calibration import CameraCalibration

    cfg = Config.load(args.config) if args.config.is_file() else Config()

    # Kalibrasi: homography_path (kalau ada) = intrinsik TERUKUR +
    # distorsi. Frame di-undistort sekali di Camera — dataset harus
    # sama dengan input detector di runtime, bukan gambar mentah lensa.
    if cfg.homography_path:
        calib = CameraCalibration.from_json(cfg.homography_path)
        undistort = ((calib.camera_matrix(), calib.dist)
                     if calib.dist else None)
        print(f"kalibrasi: {cfg.homography_path} "
              f"(fx={calib.fx:.1f} hFOV={calib.hfov_deg():.1f} deg"
              f"{', undistort ON' if undistort else ''})")
    else:
        calib = CameraCalibration.from_config(cfg)
        undistort = None
        print("kalibrasi: default (hFOV 62 deg) — set homography_path "
              "di robot.json buat intrinsik terukur")

    out_dir = args.output or (Path(cfg.output_dir)
                              / time.strftime("dataset-%Y%m%d-%H%M%S"))
    frames_dir = out_dir / "frames"
    frames_dir.mkdir(parents=True, exist_ok=True)

    write_frame = build_writer(args.format, args.quality)

    hub = server = None
    if not args.no_stream:
        from mizuro.stream import FrameHub, MjpegServer
        hub = FrameHub(quality=cfg.stream_quality, fps=cfg.stream_fps,
                       width=cfg.stream_width, height=cfg.stream_height)

    print(f"output  : {out_dir}")
    print(f"format  : {args.format}"
          f"{f' q{args.quality}' if args.format == 'jpg' else ''}, "
          f"setiap frame ke-{args.every}")
    print(f"stop    : {args.frames or 'unlimited'} frame / "
          f"{args.duration or 'unlimited'} detik / Ctrl+C")

    keyboard = None if args.no_keyboard else Keyboard()
    if keyboard is not None and keyboard.enabled:
        print("kontrol : SPACE = mulai/stop rekam, Q = keluar")

    rows: list[dict] = []
    stats = {"saved": 0, "read_count": 0, "write_ms_total": 0.0,
             "takes": 0, "elapsed_s": 0.0, "warmup_s": args.warmup}
    try:
        with Camera.from_config(cfg, undistort=undistort) as cam:
            ctx = (MjpegServer.from_config(cfg, hub) if hub else None)
            if ctx is not None:
                ctx.__enter__()
                print(f"preview : {ctx.url} (buka di browser buat framing)")
            try:
                rows, stats = record_loop(
                    cam=cam, write_frame=write_frame,
                    frames_dir=frames_dir, args=args, hub=hub,
                    keyboard=keyboard)
            finally:
                if ctx is not None:
                    ctx.__exit__(None, None, None)
                if keyboard is not None:
                    keyboard.close()
    finally:
        index_path = out_dir / "index.csv"
        with index_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=[
                "filename", "t_monotonic_s", "t_wall_s",
                "width", "height"])
            writer.writeheader()
            writer.writerows(rows)

        elapsed = stats["elapsed_s"]
        record_s = max(elapsed - stats["warmup_s"], 1e-6)
        saved = stats["saved"]
        avg_write = stats["write_ms_total"] / max(saved, 1)
        print(f"\n=== selesai: {saved} frame dalam {elapsed:.1f}s "
              f"(rekor {record_s:.1f}s) ===")
        print(f"fps data : {saved / record_s:.1f} "
              f"(dari {stats['read_count']} frame terbaca)")
        print(f"take     : {stats['takes']}")
        print(f"tulis    : {avg_write:.1f} ms/frame rata-rata")
        print(f"frame    : {frames_dir}")
        print(f"index    : {index_path}")
        if saved == 0:
            print("CATATAN: 0 frame — kamera terbuka? SPACE belum ditekan "
                  "(mode manual)? warmup keburu panjang? coba --warmup 0.5")


if __name__ == "__main__":
    main()
