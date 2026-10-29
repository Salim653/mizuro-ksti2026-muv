#!/usr/bin/env python3
"""Eval persepsi lajur Fase 1 — sintetis atau dataset arena-3d.

    # sintetis (tanpa hardware, tanpa sim browser):
    ../../.venv/bin/python tools/eval_perception.py --frames 100

    # dataset asli dari arena-3d (zip export):
    ../../.venv/bin/python tools/eval_perception.py --dataset laps.zip

Dua mode sengaja dipisah:

- Sintetis: frame dirender synth.py dengan ground truth EKSAK sampai
  numerik. Mengukur pipeline persepsi murni (threshold -> stasiun -> fit).
- Dataset: frame asli dari sim. Truth DIPINDAH ke stasiun yang sama dengan
  fit (track/track_data.json + ego pose), karena mount 0.16 m hanya bisa
  melihat lajur dari ~0.5 m di depan — membandingkan estimasi stasiun 0.6 m
  dengan truth di poros mengukur jarak stasiun, bukan detector.

Konvensi tanda (sama dengan mizuro.perception): lane_error_m positif =
pusat lajur di KANAN robot; heading_error_rad positif = lajur mengarah ke
KANAN heading robot. Truth dataset dicatat positif ke KIRI, jadi
dibandingkan dengan tanda plus (lihat MAP_SIGN di bawah).

Target Fase 1: median error lateral < 3 cm, median error heading < 5 derajat
pada frame yang lajur-nya terlihat.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import zipfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mizuro.perception.calibration import CameraCalibration  # noqa: E402
from mizuro.perception.classical import LaneTracker  # noqa: E402
from mizuro.perception.synth import random_truth, render_lane_frame  # noqa: E402

# Target Fase 1 (lihat PANDUAN-LANJUTAN-TIM.md).
TARGET_LAT_M = 0.03
TARGET_HEAD_DEG = 5.0


def _stats(errs: list[float]) -> dict[str, float]:
    a = np.abs(np.asarray(errs, dtype=float))
    if a.size == 0:
        return {"n": 0, "median": float("nan"), "p90": float("nan"),
                "worst": float("nan"), "mae": float("nan")}
    return {"n": int(a.size), "median": float(np.median(a)),
            "p90": float(np.percentile(a, 90)), "worst": float(a.max()),
            "mae": float(a.mean())}


def _report(name: str, lat: list[float], head_deg: list[float],
            no_lane: int, total: int) -> bool:
    sl, sh = _stats(lat), _stats(head_deg)
    print(f"\n=== {name} ===")
    print(f"frame diskor  : {sl['n']}   tanpa lajur: {no_lane} / {total}")
    print(f"lateral  (mm)  : median {sl['median'] * 1000:7.2f}   "
          f"p90 {sl['p90'] * 1000:7.2f}   worst {sl['worst'] * 1000:7.2f}   "
          f"MAE {sl['mae'] * 1000:7.2f}")
    print(f"heading  (deg) : median {sh['median']:7.3f}   "
          f"p90 {sh['p90']:7.3f}   worst {sh['worst']:7.3f}   "
          f"MAE {sh['mae']:7.3f}")
    ok = sl["median"] < TARGET_LAT_M and sh["median"] < TARGET_HEAD_DEG
    print(f"target Fase 1  : median lateral < {TARGET_LAT_M * 1000:.0f} mm, "
          f"median heading < {TARGET_HEAD_DEG:.0f} deg  ->  "
          f"{'LOLOS' if ok else 'BELUM'}")
    return ok


# ---------------------------------------------------------------- sintetis

def eval_synthetic(args: argparse.Namespace) -> bool:
    calib = CameraCalibration.from_hfov(args.width, args.height,
                                        args.mount_height, args.mount_pitch,
                                        mount_x_m=args.mount_x,
                                        mount_y_m=args.mount_y,
                                        mount_yaw_deg=args.mount_yaw,
                                        mount_roll_deg=args.mount_roll)
    tracker = LaneTracker(polarity=args.polarity, roi_top=args.roi_top,
                          near_m=args.near, far_m=args.far)
    rng = np.random.default_rng(args.seed)
    lat, head, no_lane = [], [], 0
    for i in range(args.frames):
        truth = random_truth(rng)
        frame, gt = render_lane_frame(calib, truth, seed=i,
                                      roi_top=args.roi_top)
        obs = tracker.estimate(frame, calib, float(i), station_m=args.station)
        if obs.confidence == 0:
            no_lane += 1
            continue
        lat.append(obs.lane_error_m - gt["lateral_m"])
        head.append(math.degrees(obs.heading_error_rad - gt["heading_rad"]))
    return _report(f"sintetis ({args.frames} frame, {args.width}x{args.height}, "
                   f"mount {args.mount_height} m / {args.mount_pitch} deg)",
                   lat, head, no_lane, args.frames)


# ----------------------------------------------------------------- dataset

def _truth_at_station(path_pts, x_m, z_m, yaw, xs):
    """(lateral, heading) centerline di stasiun maju xs, frame mobil.

    Mengikuti baseline_line_follower.py: cari titik centerline terdekat,
    ambil kandidat pada jarak maju ~xs, heading dari dua titik di sekitarnya.
    Mengembalikan (lateral_LEFT+, heading_LEFT+ derajat) — positif kiri,
    konvensi dataset.
    """
    c, s = math.cos(yaw), math.sin(yaw)

    def frame(px, pz):
        dx, dz = px - x_m, pz - z_m
        return dx * c - dz * s, -(dx * s + dz * c)   # (forward, left)

    n = len(path_pts)
    bi = min(range(n), key=lambda i: (path_pts[i][0] - x_m) ** 2
             + (path_pts[i][1] - z_m) ** 2)
    win = range(bi - 220, bi + 220)
    cand = min(win, key=lambda i: abs(frame(*path_pts[i % n])[0] - xs))
    k = 12
    a = frame(*path_pts[(cand - k) % n])
    b = frame(*path_pts[(cand + k) % n])
    lat = frame(*path_pts[cand % n])[1]
    head = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
    return lat, head


def eval_dataset(args: argparse.Namespace) -> bool:
    import cv2

    z = zipfile.ZipFile(args.dataset)
    meta = json.loads(z.read("dataset.json").decode("utf-8"))
    cam = meta["camera"]
    K, mount = cam["intrinsics"], cam["mount"]
    calib = CameraCalibration(
        width=int(K["w"]), height=int(K["h"]), height_m=float(mount["height_m"]),
        pitch_rad=math.radians(float(mount["pitch_deg"])),
        fx=float(K["fx"]), fy=float(K["fy"]),
        cx=float(K["cx"]), cy=float(K["cy"]))

    tracker = LaneTracker(polarity=args.polarity, roi_top=args.roi_top,
                          near_m=args.near, far_m=args.far)

    lines = z.read("telemetry.csv").decode("utf-8").strip().split("\n")
    rows = [dict(zip(lines[0].split(","), ln.split(","))) for ln in lines[1:]]

    centerline = None
    try:
        track = json.loads(z.read("track/track_data.json").decode("utf-8"))
        centerline = [(p[0], p[1]) for p in track["path"]]
    except KeyError:
        print("catatan: arsip tanpa track/track_data.json — perbandingan "
              "geometry-matched dilewati, hanya truth poros yang dilaporkan")

    lat_geo, head_geo, lat_raw, head_raw = [], [], [], []
    no_lane = skipped = 0
    for row in rows:
        if row.get("state") != "running":
            continue
        frame_no = int(row["frame"])
        raw = np.frombuffer(z.read(f"frames/{frame_no:06d}.jpg"), np.uint8)
        frame = cv2.imdecode(raw, cv2.IMREAD_COLOR)
        if frame is None:
            skipped += 1
            continue
        obs = tracker.estimate(frame, calib, float(frame_no),
                               station_m=args.station)
        if obs.confidence == 0:
            no_lane += 1
            continue
        # Truth poras (annotations/telemetry): positif kiri -> error pakai
        # tanda plus di heading, minus di lateral (lihat docstring modul).
        off_true = float(row["lane_offset_m"])
        he_true = float(row["heading_err_deg"])
        lat_raw.append(obs.lane_error_m - off_true)
        head_raw.append(math.degrees(obs.heading_error_rad) + he_true)
        if centerline is not None:
            t_lat, t_head = _truth_at_station(
                centerline, float(row["x_m"]), float(row["z_m"]),
                float(row["yaw_rad"]), args.station)
            lat_geo.append(obs.lane_error_m + t_lat)
            head_geo.append(math.degrees(obs.heading_error_rad) + t_head)

    print(f"dataset  : {args.dataset}")
    print(f"frames   : {len(rows)}   {calib.width}x{calib.height}   "
          f"mount {mount['height_m']} m / {mount['pitch_deg']} deg")
    print(f"stasiun  : {args.station} m   polarity: {args.polarity}")

    ok_raw = _report("vs truth poras (ada jarak stasiun — bukan murni "
                     "detector)", lat_raw, head_raw, no_lane, len(rows))
    if centerline is not None:
        ok_geo = _report("vs centerline di stasiun yang sama (murni "
                         "detector)", lat_geo, head_geo, no_lane, len(rows))
        return ok_geo
    return ok_raw


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--dataset", help="zip export arena-3d")
    p.add_argument("--frames", type=int, default=100)
    p.add_argument("--seed", type=int, default=1234)
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--mount-height", type=float, default=0.16)
    p.add_argument("--mount-pitch", type=float, default=12.0)
    p.add_argument("--mount-x", type=float, default=0.0,
                   help="offset kamera maju dari poras (meter)")
    p.add_argument("--mount-y", type=float, default=0.0,
                   help="offset kamera kanan dari poras (meter)")
    p.add_argument("--mount-yaw", type=float, default=0.0,
                   help="yaw kamera (derajat, + = ke kanan robot)")
    p.add_argument("--mount-roll", type=float, default=0.0,
                   help="roll kamera (derajat, + = CCW dari belakang)")
    p.add_argument("--station", type=float, default=0.6,
                   help="stasiun lookahead evaluasi (meter)")
    p.add_argument("--near", type=float, default=0.5)
    p.add_argument("--far", type=float, default=3.0)
    p.add_argument("--roi-top", type=float, default=0.35)
    p.add_argument("--polarity", default="auto",
                   choices=["auto", "light_on_dark", "dark_on_light"])
    args = p.parse_args()

    if args.dataset:
        ok = eval_dataset(args)
    else:
        ok = eval_synthetic(args)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
