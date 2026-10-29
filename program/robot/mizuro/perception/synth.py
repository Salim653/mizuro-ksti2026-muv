"""Generator frame lajur sintetis dengan ground truth eksak.

Sim browser arena-3d tidak bisa dijalankan headless, jadi eval Fase 1
memakai frame yang dirender DI SINI dengan model proyeksi yang sama
(calibration.ground_to_pixel) — ground truth-nya eksak sampai numerik,
bukan label manusia.

Scene sengaja minimal: lantai + pita lajur + dash tengah + noise. Yang
diuji adalah pipeline persepsi (threshold -> stasiun -> fit), bukan
realisme lantai. Glare/shadow lantai asli masuk daftar "known lies" sim
juga; gap itu diukur nanti di bench, bukan dikarang di sini.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import numpy as np

from .calibration import CameraCalibration
from .geometry import lane_state

# Warna scene (BGR). Lantai gelap + marka putih = light_on_dark;
# sebaliknya untuk dark_on_light.
FLOOR_DARK = 45
FLOOR_LIGHT = 205
PAINT_LIGHT = 235
PAINT_DARK = 25
WALL_DARK = 30
WALL_LIGHT = 170

EDGE_WIDTH_M = 0.05    # lebar garis tepi solid (guidebook: >= 1 cm)
DASH_ON_M = 0.15       # panjang dash tengah
DASH_OFF_M = 0.15
DASH_WIDTH_M = 0.05


@dataclass(frozen=True)
class LaneTruth:
    """Ground truth lajur: x_center(y) = a*y^2 + b*y + c (meter)."""

    a: float
    b: float
    c: float
    width_m: float = 0.50
    polarity: str = "light_on_dark"

    def x_at(self, y: float) -> float:
        return self.a * y * y + self.b * y + self.c

    def state_at(self, station_m: float) -> tuple[float, float, float]:
        """(lateral_m, heading_rad, curvature_inv_m) eksak di stasiun."""
        lateral = self.x_at(station_m)
        slope = 2.0 * self.a * station_m + self.b
        heading = math.atan(slope)
        curvature = (2.0 * self.a) / (1.0 + slope * slope) ** 1.5
        return lateral, heading, curvature

    def as_dict(self, station_m: float) -> dict:
        lateral, heading, curvature = self.state_at(station_m)
        return {"a": self.a, "b": self.b, "c": self.c,
                "width_m": self.width_m, "polarity": self.polarity,
                "station_m": station_m, "lateral_m": lateral,
                "heading_rad": heading, "curvature_inv_m": curvature}


def _quad(calib: CameraCalibration, y0: float, y1: float,
          x_left, x_right) -> np.ndarray | None:
    """Proyeksikan segmen empat persegi tanah (y0..y1 maju) ke piksel.

    x_left/x_right: fungsi lateral (meter, +kanan) -> lateral. ground_to_pixel
    menerima (x_forward, y_right) — jadi y duluan.
    """
    u0l, v0 = calib.ground_to_pixel(y0, x_left(y0))
    u1l, v1 = calib.ground_to_pixel(y1, x_left(y1))
    u0r, _ = calib.ground_to_pixel(y0, x_right(y0))
    u1r, _ = calib.ground_to_pixel(y1, x_right(y1))
    pts = np.array([[u0l, v0], [u1l, v1], [u1r, v1], [u0r, v0]],
                   dtype=np.float32)
    if not np.isfinite(pts).all():
        return None
    return pts


def _above_horizon_mask(calib: CameraCalibration,
                        h: int, w: int) -> np.ndarray:
    """Mask piksel di atas horizon (roll-aware), bentuk (h, w).

    Horizon tak-roll adalah baris v_h; roll memutarnya -roll di sekitar
    titik principal (lihat calibration.ground_to_pixel). Untuk kolom u:

        v_horizon(u) = cy + (v_h - cy)/cos r - tan r * (u - cx)

    Roll 0 -> mask persis sama dengan potongan baris datar.
    """
    v_h = calib.horizon_row
    if v_h <= 0:
        return np.zeros((h, w), dtype=bool)
    u = np.arange(w, dtype=float)
    if calib.mount_roll_rad:
        cr, sr = math.cos(calib.mount_roll_rad), math.sin(calib.mount_roll_rad)
        v_horizon = (calib.cy + (v_h - calib.cy) / cr
                     - sr / cr * (u - calib.cx))
    else:
        v_horizon = np.full(w, v_h)
    rows = np.arange(h, dtype=float)[:, None]
    return rows < v_horizon[None, :]


def render_lane_frame(calib: CameraCalibration, truth: LaneTruth, *,
                      seed: int = 0, noise_sigma: float = 4.0,
                      blur_sigma: float = 1.2,
                      roi_top: float = 0.35) -> tuple[np.ndarray, dict]:
    """Render satu frame BGR + dict ground truth (termasuk state di
    stasiun lookahead default 0.6 m — override lewat return value).

    `roi_top` WAJIB sama dengan tracker: batas jauh render diambil dari
    forward_range(roi_top). Tanpa itu, batas jauh = baris dekat horizon
    yang memetakan ke jarak ekstrem (puluhan meter) dan segmen render
    jadi terlalu besar — lajur MIRING tergambar di posisi salah.
    """
    rng = np.random.default_rng(seed)
    h, w = calib.height, calib.width
    light_paint = truth.polarity == "light_on_dark"

    # light_on_dark = marka TERANG di lantai GELAP; sebaliknya untuk
    # dark_on_light.
    floor = FLOOR_DARK if light_paint else FLOOR_LIGHT
    wall = WALL_DARK if light_paint else WALL_LIGHT
    paint = PAINT_LIGHT if light_paint else PAINT_DARK

    frame = np.full((h, w, 3), floor, dtype=np.uint8)
    # Dinding di atas horizon; batasnya per kolom supaya konsisten
    # dengan proyeksi roll (lihat _above_horizon_mask).
    frame[_above_horizon_mask(calib, h, w)] = wall

    y_near, y_far = calib.forward_range(roi_top)
    y_far = max(y_far, y_near + 0.5)
    n_seg = 48

    # Dua garis tepi SOLID di x_c +/- width/2 (class 60 di label sim).
    # Cat sebagai tuple BGR: opencv-python 4.8 headless menggambar NOL
    # piksel kalau warna skalar dipakai bersama LINE_AA.
    paint_bgr = (int(paint), int(paint), int(paint))
    for sign in (-1.0, 1.0):
        def edge(y, _s=sign):
            return truth.x_at(y) + _s * truth.width_m / 2.0
        ys = np.linspace(y_near, y_far, n_seg + 1)
        for i in range(n_seg):
            quad = _quad(calib, ys[i], ys[i + 1],
                         lambda yy: edge(yy) - EDGE_WIDTH_M / 2.0,
                         lambda yy: edge(yy) + EDGE_WIDTH_M / 2.0)
            if quad is not None:
                cv2.fillPoly(frame, [quad.astype(np.int32)], paint_bgr,
                             lineType=cv2.LINE_AA)

    # Dash tengah (class 80) — cat yang sama dengan tepi, pola putus-putus.
    y = y_near
    while y < y_far:
        y1 = min(y + DASH_ON_M, y_far)
        quad = _quad(calib, y, y1,
                     lambda yy: truth.x_at(yy) - DASH_WIDTH_M / 2.0,
                     lambda yy: truth.x_at(yy) + DASH_WIDTH_M / 2.0)
        if quad is not None:
            cv2.fillPoly(frame, [quad.astype(np.int32)], paint_bgr,
                         lineType=cv2.LINE_AA)
        y = y1 + DASH_OFF_M

    if blur_sigma > 0:
        frame = cv2.GaussianBlur(frame, (0, 0), blur_sigma)
    if noise_sigma > 0:
        noise = rng.normal(0.0, noise_sigma, frame.shape)
        frame = np.clip(frame.astype(np.float64) + noise, 0, 255).astype(np.uint8)

    return frame, truth.as_dict(station_m=0.6)


def random_truth(rng: np.random.default_rng, *,
                 polarity: str | None = None) -> LaneTruth:
    """Skenario acak untuk eval: campuran offset kecil-kemudian, lengkung
    halus sampai tajam (termasuk kasus lajur keluar frame mid-tikungan)."""
    if polarity is None:
        polarity = ("light_on_dark" if rng.random() < 0.5
                    else "dark_on_light")
    c = float(rng.uniform(-0.12, 0.12))
    # 80% halus (R >= ~6 m), 20% tajam (sampai R ~ 2 m).
    if rng.random() < 0.8:
        a = float(rng.uniform(-0.08, 0.08))
    else:
        a = float(rng.uniform(-0.25, 0.25))
    b = float(rng.uniform(-0.06, 0.06))
    return LaneTruth(a=a, b=b, c=c, polarity=polarity)
