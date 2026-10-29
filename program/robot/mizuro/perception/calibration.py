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

Mount lengkap (opsional; default 0 = kamera tepat di titik referensi
robot, menghadap lurus, tanpa roll):

    piksel -> roll dibatalkan di ruang piksel -> ray pitch -> tanah
    frame-kamera -> rotasi yaw + geser (x, y) -> tanah frame-ROBOT.

Output selalu di frame robot (titik referensi = poras, yang dipakai
controller), bukan frame kamera. Tanpa offset, matematika di atas
persis sama dengan versi pitch-only — tidak ada regresi untuk
synth/eval yang tidak memakai mount.
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
    # Posisi kamera relatif titik referensi robot (poras), meter:
    # mount_x_m maju (+), mount_y_m kanan (+).
    mount_x_m: float = 0.0
    mount_y_m: float = 0.0
    # Orientasi kamera, radian. yaw positif = optical axis mengarah ke
    # KANAN robot. roll positif = kamera roll CCW dilihat dari belakang
    # (sisi kanan frame melihat lebih banyak lantai).
    mount_yaw_rad: float = 0.0
    mount_roll_rad: float = 0.0
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
        if abs(self.mount_x_m) > 2.0 or abs(self.mount_y_m) > 2.0:
            raise ValueError("offset mount kamera tidak masuk akal (> 2 m)")

    # --- proyeksi titik ---

    def pixel_to_ground(self, u, v):
        """(u, v) -> (x_forward_m, y_right_m) di frame ROBOT.

        Titik di atas horizon (tidak menyentuh tanah) -> NaN, bukan angka
        palsu; pemanggil menyaring NaN sebelum fit.

        Urutan transformasi (kebalikan ground_to_pixel):
        1. Roll dibatalkan di ruang piksel: (u, v) dirotasi +roll di
           sekitar titik principal.
        2. Ray dengan pitch diproyeksikan ke tanah -> titik frame-kamera.
        3. Titik frame-kamera dirotasi yaw lalu digeser (x, y) mount ->
           frame robot.
        """
        u = np.asarray(u, dtype=float)
        v = np.asarray(v, dtype=float)
        if self.mount_roll_rad:
            du, dv = u - self.cx, v - self.cy
            cr, sr = math.cos(self.mount_roll_rad), math.sin(self.mount_roll_rad)
            u = self.cx + du * cr - dv * sr
            v = self.cy + du * sr + dv * cr
        a = (u - self.cx) / self.fx
        b = (v - self.cy) / self.fy
        down = math.sin(self.pitch_rad) + b * math.cos(self.pitch_rad)
        with np.errstate(divide="ignore", invalid="ignore"):
            lam = np.where(down > 1e-9, self.height_m / down, np.nan)
            x = lam * (math.cos(self.pitch_rad) - b * math.sin(self.pitch_rad))
            y = lam * a
        if self.mount_yaw_rad or self.mount_x_m or self.mount_y_m:
            cw, sw = math.cos(self.mount_yaw_rad), math.sin(self.mount_yaw_rad)
            x, y = (x * cw - y * sw + self.mount_x_m,
                    x * sw + y * cw + self.mount_y_m)
        return x, y

    def ground_to_pixel(self, x_forward, y_right):
        """(x_forward_m, y_right_m) frame ROBOT -> (u, v).

        Kebalikan persis pixel_to_ground: titik robot digeser balik
        (-x, -y), dirotasi -yaw ke frame kamera, diproyeksikan dengan
        pitch, lalu dirotasi -roll di ruang piksel.
        """
        x = np.asarray(x_forward, dtype=float)
        y = np.asarray(y_right, dtype=float)
        if self.mount_yaw_rad or self.mount_x_m or self.mount_y_m:
            dx, dy = x - self.mount_x_m, y - self.mount_y_m
            cw, sw = math.cos(self.mount_yaw_rad), math.sin(self.mount_yaw_rad)
            x = dx * cw + dy * sw
            y = -dx * sw + dy * cw
        s = 1.0 / (x * math.cos(self.pitch_rad)
                   + self.height_m * math.sin(self.pitch_rad))
        a = y * s
        b = (s * self.height_m - math.sin(self.pitch_rad)) \
            / math.cos(self.pitch_rad)
        u = self.cx + a * self.fx
        v = self.cy + b * self.fy
        if self.mount_roll_rad:
            du, dv = u - self.cx, v - self.cy
            cr, sr = math.cos(self.mount_roll_rad), math.sin(self.mount_roll_rad)
            u = self.cx + du * cr + dv * sr
            v = self.cy - du * sr + dv * cr
        return u, v

    # --- homography (operasi batch) ---

    def homography(self) -> np.ndarray:
        """Matriks 3x3 dengan [x, y, 1] ~ H @ [u, v, 1] — frame ROBOT.

        Komposisi: piksel -> (rotasi roll di ruang piksel) -> proyeksi
        pitch ke tanah frame-kamera -> (rotasi yaw + geser mount) ->
        frame robot. Baris dengan W <= 0 = di atas horizon; bersihkan
        sebelum pakai.
        """
        h, p = self.height_m, self.pitch_rad
        sp, cp = math.sin(p), math.cos(p)
        H = np.array([
            [0.0,           -h * sp,        h * (self.fy * cp + sp * self.cy)],
            [h * self.fy / self.fx, 0.0,     -h * self.fy * self.cx / self.fx],
            [0.0,            cp,            sp * self.fy - cp * self.cy],
        ])
        if self.mount_roll_rad:
            # p' = pp + R(roll) @ (p - pp)  (lihat pixel_to_ground)
            cr, sr = math.cos(self.mount_roll_rad), math.sin(self.mount_roll_rad)
            H = H @ np.array([
                [cr, -sr, self.cx - self.cx * cr + self.cy * sr],
                [sr,  cr, self.cy - self.cx * sr - self.cy * cr],
                [0.0, 0.0, 1.0],
            ])
        if self.mount_yaw_rad or self.mount_x_m or self.mount_y_m:
            cw, sw = math.cos(self.mount_yaw_rad), math.sin(self.mount_yaw_rad)
            H = np.array([
                [cw, -sw, self.mount_x_m],
                [sw,  cw, self.mount_y_m],
                [0.0, 0.0, 1.0],
            ]) @ H
        return H

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

    @property
    def horizon_row_worst(self) -> float:
        """Baris horizon terjauh ke BAWAH di seluruh lebar frame.

        Roll memiringkan horizon di gambar (lihat
        synth._above_horizon_mask); tracker WAJIB crop ROI di bawah baris
        ini supaya tidak ada piksel dinding yang masuk. Roll 0 -> sama
        dengan horizon_row.
        """
        if not self.mount_roll_rad:
            return self.horizon_row
        cr = math.cos(self.mount_roll_rad)
        base = self.cy + (self.horizon_row - self.cy) / cr
        return base + (self.width / 2.0) * abs(math.tan(self.mount_roll_rad))

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
                  pitch_deg: float, hfov_deg: float = DEFAULT_HFOV_DEG, *,
                  mount_x_m: float = 0.0, mount_y_m: float = 0.0,
                  mount_yaw_deg: float = 0.0,
                  mount_roll_deg: float = 0.0) -> "CameraCalibration":
        f = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
        return cls(width=width, height=height, height_m=height_m,
                   pitch_rad=math.radians(pitch_deg), fx=f, fy=f,
                   cx=(width - 1) / 2.0, cy=(height - 1) / 2.0,
                   mount_x_m=mount_x_m, mount_y_m=mount_y_m,
                   mount_yaw_rad=math.radians(mount_yaw_deg),
                   mount_roll_rad=math.radians(mount_roll_deg))

    @classmethod
    def from_config(cls, config) -> "CameraCalibration":
        """Bangun dari Config robot (robot.json): mount lengkap.

        Config memakai DERAJAT untuk orientasi (diukur dengan protractor
        di bench); di sini dikonversi ke radian. Intrinsik masih dari
        hFOV default — untuk akurasi tinggi pakai from_json dengan
        kalibrasi terukur (lihat homography_path di Config).
        """
        return cls(
            width=config.image_width, height=config.image_height,
            height_m=config.mount_z_m,
            pitch_rad=math.radians(config.mount_pitch_deg),
            mount_x_m=config.mount_x_m, mount_y_m=config.mount_y_m,
            mount_yaw_rad=math.radians(config.mount_yaw_deg),
            mount_roll_rad=math.radians(config.mount_roll_deg))

    @classmethod
    def from_json(cls, path: str | Path) -> "CameraCalibration":
        """Muat kalibrasi terukur (dari tools/calibrate.py atau dataset.json).

        Format: {"width", "height", "height_m", "pitch_deg", "fx", "fy",
        "cx", "cy"} — fx/fy/cx/cy opsional (diturunkan dari hfov_default).
        Mount opsional: "mount_x_m", "mount_y_m", "mount_yaw_deg",
        "mount_roll_deg" (default 0 = kamera di titik referensi robot).
        """
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        try:
            return cls(
                width=int(data["width"]), height=int(data["height"]),
                height_m=float(data["height_m"]),
                pitch_rad=math.radians(float(data["pitch_deg"])),
                mount_x_m=float(data.get("mount_x_m", 0.0)),
                mount_y_m=float(data.get("mount_y_m", 0.0)),
                mount_yaw_rad=math.radians(
                    float(data.get("mount_yaw_deg", 0.0))),
                mount_roll_rad=math.radians(
                    float(data.get("mount_roll_deg", 0.0))),
                fx=float(data.get("fx", 0)), fy=float(data.get("fy", 0)),
                cx=float(data.get("cx", 0)), cy=float(data.get("cy", 0)))
        except KeyError as exc:
            raise ValueError(f"kalibrasi kurang field: {exc}") from exc
