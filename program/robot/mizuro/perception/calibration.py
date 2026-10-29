"""Kalibrasi kamera -> bidang tanah.

Konvensi mengikuti `annotations.jsonl` arena-3d (car frame: X maju, Y kanan,
Z bawah; pinhole CV). Ray piksel (a, b, 1) dengan a=(u-cx)/fx, b=(v-cy)/fy
menjadi di car frame, dengan pitch p ke bawah:

    fwd  = cos p - b sin p
    right = a
    down = sin p + b cos p
    lambda = height / down
    x_forward = lambda * fwd ,  y_right = lambda * right

Baris di atas horizon (down <= 0) tidak menyentuh tanah — homography sengaja
membuangnya lewat W <= 0, bukan mengembalikan angka sampah.

Mount default dari arena-3d: 0.16 m, pitch 12 derajat ke bawah, hFOV 62 derajat
(Pi camera module v2). Dari situ tanah baru masuk frame ~0.22 m di depan
kamera; fit lajur yang berguna mulai ~0.5 m.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

# Default mount arena-3d (README: "0.16 m high, 12 deg down, 62 deg hFOV").
DEFAULT_HEIGHT_M = 0.16
DEFAULT_PITCH_DEG = 12.0
DEFAULT_HFOV_DEG = 62.0

# Clamp jarak maju: di dekat horizon proyeksi meledak ke tak hingga.
MAX_FORWARD_M = 12.0


@dataclass(frozen=True)
class CameraCalibration:
    """Model pinhole + mount. Semua dalam piksel/meter/radian."""

    width: int
    height: int
    height_m: float = DEFAULT_HEIGHT_M
    pitch_rad: float = math.radians(DEFAULT_PITCH_DEG)
    fx: float = 0.0
    fy: float = 0.0
    cx: float = 0.0
    cy: float = 0.0

    def __post_init__(self) -> None:
        if self.fx <= 0 or self.fy <= 0:
            # Turunkan dari hFOV default (piksel persegi).
            f = (self.width / 2.0) / math.tan(math.radians(DEFAULT_HFOV_DEG) / 2.0)
            object.__setattr__(self, "fx", f)
            object.__setattr__(self, "fy", f)
        if self.cx <= 0 or self.cy <= 0:
            object.__setattr__(self, "cx", (self.width - 1) / 2.0)
            object.__setattr__(self, "cy", (self.height - 1) / 2.0)
        if self.height_m <= 0:
            raise ValueError("height_m harus positif")

    # --- proyeksi titik ---

    def pixel_to_ground(self, u, v):
        """(u, v) -> (x_forward_m, y_right_m). Bisa array numpy.

        Titik di atas horizon (tidak menyentuh tanah) -> NaN, bukan angka
        palsu; pemanggil menyaring NaN sebelum fit.
        """
        a = (np.asarray(u, dtype=float) - self.cx) / self.fx
        b = (np.asarray(v, dtype=float) - self.cy) / self.fy
        down = math.sin(self.pitch_rad) + b * math.cos(self.pitch_rad)
        with np.errstate(divide="ignore", invalid="ignore"):
            lam = np.where(down > 1e-9, self.height_m / down, np.nan)
            x = lam * (math.cos(self.pitch_rad) - b * math.sin(self.pitch_rad))
            y = lam * a
        return x, y

    def ground_to_pixel(self, x_forward, y_right):
        """(x_forward_m, y_right_m) -> (u, v). Kebalikan persis di atas tanah.

        Turunan: vektor kamera->titik (x, y, h) searah dengan ray car-frame
        (cos p - b sin p, a, sin p + b cos p); skala s = 1/(x cos p + h sin p),
        lalu a = s*y dan b = (s*h - sin p)/cos p.
        """
        x = np.asarray(x_forward, dtype=float)
        y = np.asarray(y_right, dtype=float)
        s = 1.0 / (x * math.cos(self.pitch_rad)
                   + self.height_m * math.sin(self.pitch_rad))
        a = y * s
        b = (s * self.height_m - math.sin(self.pitch_rad)) \
            / math.cos(self.pitch_rad)
        return self.cx + a * self.fx, self.cy + b * self.fy

    # --- homography (operasi batch) ---

    def homography(self) -> np.ndarray:
        """Matriks 3x3 dengan [x, y, 1] ~ H @ [u, v, 1].

        Turunan langsung dari rumus ray di atas (lihat docstring modul).
        Baris dengan W <= 0 = di atas horizon; bersihkan sebelum pakai.
        """
        h, p = self.height_m, self.pitch_rad
        sp, cp = math.sin(p), math.cos(p)
        return np.array([
            [0.0,           -h * sp,        h * (self.fy * cp + sp * self.cy)],
            [h * self.fy / self.fx, 0.0,     -h * self.fy * self.cx / self.fx],
            [0.0,            cp,            sp * self.fy - cp * self.cy],
        ])

    def ground_from_pixels(self, uv: np.ndarray) -> np.ndarray:
        """Batch (N, 2) piksel -> (N, 2) tanah; baris horizon jadi NaN."""
        uv = np.asarray(uv, dtype=float)
        ones = np.ones((len(uv), 1))
        hom = (self.homography() @ np.hstack([uv, ones]).T).T
        with np.errstate(divide="ignore", invalid="ignore"):
            out = hom[:, :2] / hom[:, 2:3]
        return np.where(hom[:, 2:3] > 1e-9, out, np.nan)

    # --- properti yang dipakai tracker ---

    @property
    def horizon_row(self) -> float:
        """Baris tempat tanah menghilang (v tak hingga)."""
        return self.cy - self.fy * math.tan(self.pitch_rad)

    def forward_range(self, roi_top_frac: float = 0.35) -> tuple[float, float]:
        """Rentar jarak maju yang terlihat: (terdekat di bawah frame, terjauh
        di batas ROI atau horizon).

        PERINGATAN: mendekati horizon, jarak maju -> tak hingga. Batas
        jauh WAJIB datang dari roi_top (bukan default 0), dan hasil
        di-clamp ke MAX_FORWARD_M supaya tidak pernah memetakan baris
        nyaris-horizon ke jarak absurd.
        """
        y_bottom, _ = self.pixel_to_ground(self.cx, self.height - 1)
        v_top = max(roi_top_frac * self.height, self.horizon_row + 2.0)
        y_top, _ = self.pixel_to_ground(self.cx, v_top)
        return float(y_bottom), float(min(y_top, MAX_FORWARD_M))

    # --- loader ---

    @classmethod
    def from_hfov(cls, width: int, height: int, height_m: float,
                  pitch_deg: float, hfov_deg: float = DEFAULT_HFOV_DEG) -> "CameraCalibration":
        f = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
        return cls(width=width, height=height, height_m=height_m,
                   pitch_rad=math.radians(pitch_deg), fx=f, fy=f,
                   cx=(width - 1) / 2.0, cy=(height - 1) / 2.0)

    @classmethod
    def from_json(cls, path: str | Path) -> "CameraCalibration":
        """Muat kalibrasi terukur (dari tools/calibrate.py atau dataset.json).

        Format: {"width", "height", "height_m", "pitch_deg", "fx", "fy",
        "cx", "cy"} — fx/fy/cx/cy opsional (diturunkan dari hfov_default).
        """
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        try:
            return cls(
                width=int(data["width"]), height=int(data["height"]),
                height_m=float(data["height_m"]),
                pitch_rad=math.radians(float(data["pitch_deg"])),
                fx=float(data.get("fx", 0)), fy=float(data.get("fy", 0)),
                cx=float(data.get("cx", 0)), cy=float(data.get("cy", 0)))
        except KeyError as exc:
            raise ValueError(f"kalibrasi kurang field: {exc}") from exc
