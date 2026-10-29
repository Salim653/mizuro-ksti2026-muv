#!/usr/bin/env python3
"""Kalibrasi kamera: checkerboard -> intrinsik + distorsi.

Cetak checkerboard (default 9x6 sudut internal, kotak 25 mm), tempel
RATA di bidang keras (kardus/kayu). Jalankan dengan /usr/bin/python3
di Jetson. Preview live di browser (MJPEG); gerakkan papan ke berbagai
posisi & sudut sampai N frame terkumpul.

    /usr/bin/python3 tools/calibrate.py --frames 20

PENTING: tinggi kamera (height_m) dan pitch TIDAK diukur alat ini —
itu pengukuran fisik (meteran/protractor), diambil dari robot.json
atau argumen --mount-height/--mount-pitch. Yang dikalibrasi di sini:
fx, fy, cx, cy, dan koefisien distorsi.

Frame yang dipakai disimpan di runs/calib-frames/; pakai --replay buat
hitung ulang dari frame tersimpan tanpa membuka kamera.
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np


def find_corners(gray: np.ndarray, pattern_size: tuple[int, int]):
    """Cari sudut checkerboard. Return (found, corners Nx1x2 float32).

    Urutan flag: FAST_CHECK dulu (murah, sering gagal), lalu adaptive
    threshold + normalize image yang jauh lebih robust di lantai
    bergaris/bertekstur. cornerSubPix hanya kalau ketemu.
    """
    import cv2
    flags = cv2.CALIB_CB_FAST_CHECK
    found, corners = cv2.findChessboardCorners(gray, pattern_size, flags=flags)
    if not found:
        flags = (cv2.CALIB_CB_ADAPTIVE_THRESH
                 | cv2.CALIB_CB_NORMALIZE_IMAGE)
        found, corners = cv2.findChessboardCorners(gray, pattern_size,
                                                   flags=flags)
    if not found:
        return False, None
    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER,
                40, 0.001)
    corners = cv2.cornerSubPix(gray, corners, (7, 7), (-1, -1), criteria)
    return True, corners


def object_points(pattern_size: tuple[int, int],
                  square_m: float) -> np.ndarray:
    """Titik 3D checkerboard di frame papan (z=0), urutan baris-major."""
    cols, rows = pattern_size
    objp = np.zeros((cols * rows, 3), dtype=np.float32)
    objp[:, :2] = (np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
                   * square_m)
    return objp


def calibrate(objpoints: list[np.ndarray],
              imgpoints: list[np.ndarray],
              shape: tuple[int, int]):
    """cv2.calibrateCamera. Return (K 3x3, dist, rpe_px).

    rpe = rata-rata galat reproyeksi (piksel) — angka kesehatan
    kalibrasi; < 0.5 px bagus, > 1 px curiga (papan tidak rata,
    gerakan blur, atau sudut kurang).
    """
    import cv2
    shape_wh = (shape[1], shape[0])   # cv2 mau (width, height)
    rpe, k_matrix, dist, _, _ = cv2.calibrateCamera(
        objpoints, imgpoints, shape_wh, None, None,
        flags=cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K3)
    return k_matrix, dist, rpe


def coverage_hint(corners, shape: tuple[int, int]) -> str:
    """Petunjuk cakupan: kuadran mana yang belum pernah dilihat."""
    h, w = shape
    cx = float(corners[:, 0, 0].mean())
    cy = float(corners[:, 0, 1].mean())
    horiz = "kiri" if cx < w / 3 else ("kanan" if cx > 2 * w / 3 else "tengah")
    vert = "atas" if cy < h / 3 else ("bawah" if cy > 2 * h / 3 else "tengah")
    return f"papan di {vert}-{horiz}"


def capture_loop(cam, hub, args, pattern_size):
    """Loop capture sampai N frame. Return (objpoints, imgpoints, shape)."""
    objp = object_points(pattern_size, args.square_size)
    objpoints: list[np.ndarray] = []
    imgpoints: list[np.ndarray] = []
    seen_quads: set[str] = set()
    last_center = None
    last_t = 0.0
    n = 0
    t_start = time.monotonic()
    print(f"capture {args.frames} frame; gerakkan papan ke berbagai "
          f"posisi/sudut (min {args.min_interval}s antar capture)")
    try:
        while n < args.frames:
            if time.monotonic() - t_start > args.timeout:
                print(f"timeout {args.timeout}s")
                break
            frame = cam.read()
            hub.publish(frame)
            import cv2
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, corners = find_corners(gray, pattern_size)
            if not found or corners is None:
                if n == 0 and int(time.monotonic() - t_start) % 5 == 0:
                    print("  (papan tidak terlihat — arahkan ke kamera)")
                continue
            center = corners[:, 0, :].mean(axis=0)
            now = time.monotonic()
            # Tolak duplikat: papan harus pindah >= 30 px dari capture
            # terakhir dan jeda minimal antar capture.
            if last_center is not None and \
                    float(np.linalg.norm(center - last_center)) < 30.0:
                continue
            if now - last_t < args.min_interval:
                continue
            last_center = center
            last_t = now
            objpoints.append(objp)
            imgpoints.append(corners.astype(np.float32))
            seen_quads.add(coverage_hint(corners, gray.shape))
            n += 1
            print(f"  [{n:2d}/{args.frames}] {coverage_hint(corners, gray.shape)}"
                  f"   kuadran: {len(seen_quads)}")
            if args.save_frames:
                out = Path(args.frame_dir)
                out.mkdir(parents=True, exist_ok=True)
                cv2.imwrite(str(out / f"calib-{n:03d}.jpg"), frame)
    except KeyboardInterrupt:
        print(f"\nstop manual dengan {n} frame")
    return objpoints, imgpoints, gray.shape


def replay_loop(args, pattern_size):
    """Hitung ulang dari frame tersimpan (tanpa kamera)."""
    import cv2
    frame_dir = Path(args.frame_dir)
    files = sorted(frame_dir.glob("calib-*.jpg"))
    if not files:
        raise SystemExit(f"tidak ada frame di {frame_dir}; "
                         "jalankan tanpa --replay dulu")
    objp = object_points(pattern_size, args.square_size)
    objpoints, imgpoints = [], []
    shape = None
    for f in files:
        img = cv2.imread(str(f))
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        shape = gray.shape
        found, corners = find_corners(gray, pattern_size)
        if found:
            objpoints.append(objp)
            imgpoints.append(corners.astype(np.float32))
            print(f"  {f.name}: sudut ketemu")
        else:
            print(f"  {f.name}: sudut TIDAK ketemu (dilewati)")
    if shape is None:
        raise SystemExit("tidak ada frame yang bisa dibaca")
    return objpoints, imgpoints, shape


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Kalibrasi kamera checkerboard -> intrinsik + distorsi")
    parser.add_argument("--config", type=Path, default=Path("robot.json"))
    parser.add_argument("--output", type=Path,
                        default=Path("calib.json"))
    parser.add_argument("--cols", type=int, default=9,
                        help="jumlah sudut internal horizontal")
    parser.add_argument("--rows", type=int, default=6,
                        help="jumlah sudut internal vertikal")
    parser.add_argument("--square-size", type=float, default=0.025,
                        help="ukuran kotak checkerboard (meter)")
    parser.add_argument("--frames", type=int, default=20)
    parser.add_argument("--min-interval", type=float, default=0.7,
                        help="jeda minimal antar capture (detik)")
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--mount-height", type=float, default=None,
                        help="tinggi lensa dari tanah (m); default dari config")
    parser.add_argument("--mount-pitch", type=float, default=None,
                        help="pitch kamera (derajat); default dari config")
    parser.add_argument("--replay", action="store_true",
                        help="hitung ulang dari runs/calib-frames/ tanpa kamera")
    parser.add_argument("--frame-dir", type=Path,
                        default=Path("runs/calib-frames"))
    parser.add_argument("--save-frames", action="store_true", default=True)
    args = parser.parse_args()

    pattern_size = (args.cols, args.rows)

    # Mount height/pitch: argumen menang, lalu config, lalu default.
    height_m, pitch_deg = 0.16, 12.0
    mount = {"mount_x_m": 0.0, "mount_y_m": 0.0,
             "mount_yaw_deg": 0.0, "mount_roll_deg": 0.0}
    if args.config.is_file():
        from mizuro.config import Config
        cfg = Config.load(args.config)
        height_m, pitch_deg = cfg.mount_z_m, cfg.mount_pitch_deg
        mount = {"mount_x_m": cfg.mount_x_m, "mount_y_m": cfg.mount_y_m,
                 "mount_yaw_deg": cfg.mount_yaw_deg,
                 "mount_roll_deg": cfg.mount_roll_deg}
    if args.mount_height is not None:
        height_m = args.mount_height
    if args.mount_pitch is not None:
        pitch_deg = args.mount_pitch

    if args.replay:
        objpoints, imgpoints, shape = replay_loop(args, pattern_size)
    else:
        import sys
        sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
        from mizuro.camera import Camera
        from mizuro.config import Config
        from mizuro.stream import FrameHub, MjpegServer
        cfg = (Config.load(args.config) if args.config.is_file()
               else Config())
        cam = Camera.from_config(cfg)
        hub = FrameHub(quality=cfg.stream_quality, fps=cfg.stream_fps,
                       width=cfg.stream_width, height=cfg.stream_height)
        with MjpegServer.from_config(cfg, hub) as server:
            print(f"preview: {server.url} (buka di browser)")
            print(f"papan  : {args.cols}x{args.rows} sudut, kotak "
                  f"{args.square_size * 1000:.0f} mm")
            objpoints, imgpoints, shape = capture_loop(cam, hub, args,
                                                       pattern_size)
        cam.close()

    if len(objpoints) < 6:
        raise SystemExit(
            f"frame valid terlalu sedikit ({len(objpoints)}); "
            "minimal 6, idealnya 15-20 dari berbagai sudut")

    k_matrix, dist, rpe = calibrate(objpoints, imgpoints, shape)
    fx, fy = float(k_matrix[0, 0]), float(k_matrix[1, 1])
    cx, cy = float(k_matrix[0, 2]), float(k_matrix[1, 2])
    hfov = float(np.degrees(2.0 * np.arctan(shape[1] / (2.0 * fx))))

    print(f"\n=== hasil kalibrasi ({len(objpoints)} frame) ===")
    print(f"ukuran    : {shape[1]}x{shape[0]}")
    print(f"fx, fy    : {fx:.2f}, {fy:.2f}")
    print(f"cx, cy    : {cx:.2f}, {cy:.2f}")
    print(f"hFOV      : {hfov:.2f} derajat")
    print(f"distorsi  : {[round(float(c), 5) for c in dist.ravel()]}")
    print(f"reproyeksi: {rpe:.3f} px rata-rata")
    if rpe > 1.0:
        print("CATATAN: rpe > 1 px — cek papan rata, fokus, "
              "dan gerakan tidak blur")
    if abs(fx - fy) / fx > 0.05:
        print("CATATAN: fx != fy > 5%% — piksel tidak persegi; "
              "kalibrasi sudah menangani, tapi pastikan bukan salah capture")

    data = {
        "width": int(shape[1]), "height": int(shape[0]),
        "height_m": float(height_m), "pitch_deg": float(pitch_deg),
        **mount,
        "fx": fx, "fy": fy, "cx": cx, "cy": cy,
        "dist": [float(c) for c in dist.ravel()],
        "rpe_px": float(rpe),
        "checkerboard": {
            "cols": args.cols, "rows": args.rows,
            "square_size_m": args.square_size,
        },
    }
    args.output.write_text(json.dumps(data, indent=2) + "\n",
                           encoding="utf-8")
    print(f"\ntersimpan: {args.output}")
    print("pakai: set homography_path di robot.json ke file ini, "
          "lalu jalankan --stream-only (frame otomatis di-undistort)")


if __name__ == "__main__":
    main()
