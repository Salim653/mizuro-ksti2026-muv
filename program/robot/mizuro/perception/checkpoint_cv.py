"""Detektor checkpoint classical CV — nol training, nol model.

Alternatif dari SemanticDetector (YOLO ONNX). Dipilih via config
`checkpoint_mode: "cv"`.

Pendekatan: threshold warna HSV + morphology + filter kontur.
Cocok untuk checkpoint dengan warna khas konsisten (marka hijau di
lantai, gate merah, cone oranye). Tidak butuh dataset, tidak butuh
training, jalan ~5 ms/frame.

Keterbatasan jujur:
- Rapuh kalau pencahayaan berubah jauh (outdoor bayang vs matahari)
- Gagal kalau warna checkpoint mirip lantai/marka lain
- Tidak bisa bedakan "checkpoint" dari "benda warna sama" — hanya
  melaporkan "ada blob warna target di sini"

Kalau arena tidak konsisten warnanya, pakai `checkpoint_mode: "model"`
setelah ada dataset asli. Keduanya mengembalikan tipe `Detection` yang
sama, jadi pemanggil tidak perlu peduli backend mana yang aktif.
"""
from __future__ import annotations

import time

import cv2
import numpy as np

from .detector import Detection

# Preset warna umum (H 0-179 di OpenCV). Dipakai kalau config tidak
# menyebut rentang HSV eksplisit.
COLOR_PRESETS: dict[str, tuple[tuple[int, int, int], tuple[int, int, int]]] = {
    "green": ((35, 50, 50), (85, 255, 255)),
    "red": ((0, 70, 50), (10, 255, 255)),
    "orange": ((10, 80, 80), (25, 255, 255)),
    "yellow": ((20, 80, 80), (35, 255, 255)),
    "blue": ((95, 50, 50), (130, 255, 255)),
}


class CheckpointDetectorCV:
    """Blob warna -> Detection. Satu infer() = satu frame -> list[Detection]."""

    def __init__(self, *, hsv_low: tuple[int, int, int] | None = None,
                 hsv_high: tuple[int, int, int] | None = None,
                 color: str = "green",
                 min_area_px: int = 300,
                 max_area_px: int = 400_000,
                 min_solidity: float = 0.3,
                 label: str = "checkpoint") -> None:
        if hsv_low is None or hsv_high is None:
            preset = COLOR_PRESETS.get(color, COLOR_PRESETS["green"])
            hsv_low = hsv_low or preset[0]
            hsv_high = hsv_high or preset[1]
        self.hsv_low = tuple(int(v) for v in hsv_low)
        self.hsv_high = tuple(int(v) for v in hsv_high)
        self.min_area_px = int(min_area_px)
        self.max_area_px = int(max_area_px)
        self.min_solidity = float(min_solidity)
        self.label = label
        self.last_inference_ms = 0.0
        self._kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

    # -- utilitas ---------------------------------------------------------

    def mask(self, frame_bgr: np.ndarray) -> np.ndarray:
        """Mask biner piksel yang masuk rentang warna target."""
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        m = cv2.inRange(hsv, self.hsv_low, self.hsv_high)
        # Buka: buang noise titik-titik. Tutup: isi lubang kecil.
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, self._kernel)
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, self._kernel)
        return m

    # -- antarmuka (sama bentuknya dengan SemanticDetector) -----------------

    def infer(self, frame_bgr: np.ndarray) -> list[Detection]:
        """Frame -> daftar Detection. Tidak pernah melempar karena frame
        kosong/buruk; frame tanpa blob -> list kosong."""
        if frame_bgr is None or frame_bgr.size == 0:
            return []
        t0 = time.perf_counter()
        try:
            m = self.mask(frame_bgr)
            contours, _ = cv2.findContours(
                m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            results: list[Detection] = []
            for c in contours:
                area = float(cv2.contourArea(c))
                if not self.min_area_px <= area <= self.max_area_px:
                    continue
                x, y, w, h = cv2.boundingRect(c)
                bbox_area = float(w * h)
                if bbox_area <= 0:
                    continue
                # Solidity = seberapa "padat" blob. Bentuk geometris rapi
                # (marka, cone) > 0.5; noise acak biasanya rendah.
                solidity = area / bbox_area
                if solidity < self.min_solidity:
                    continue
                blob_mask = np.zeros(frame_bgr.shape[:2], dtype=np.uint8)
                cv2.drawContours(blob_mask, [c], -1, 1, -1)
                results.append(Detection(
                    class_id=0, label=self.label,
                    confidence=min(1.0, solidity),
                    bbox=(float(x), float(y), float(w), float(h)),
                    mask=blob_mask.astype(bool),
                    meta={"area_px": area, "solidity": solidity,
                          "backend": "cv"}))
            # Terbesar dulu: blob paling menonjol paling mungkin checkpoint.
            results.sort(key=lambda d: d.meta["area_px"], reverse=True)
            return results
        finally:
            self.last_inference_ms = (time.perf_counter() - t0) * 1000.0

    @classmethod
    def from_config(cls, config) -> CheckpointDetectorCV:
        """Bangun dari Config — rentang HSV & ambang dari satu tempat."""
        return cls(
            hsv_low=tuple(config.checkpoint_hsv_low)
            if config.checkpoint_hsv_low else None,
            hsv_high=tuple(config.checkpoint_hsv_high)
            if config.checkpoint_hsv_high else None,
            color=config.checkpoint_color,
            min_area_px=config.checkpoint_min_area_px,
            max_area_px=config.checkpoint_max_area_px,
            min_solidity=config.checkpoint_min_solidity,
            label=config.checkpoint_label)
