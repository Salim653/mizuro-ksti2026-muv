"""LaneTracker — persepsi lajur CV klasik.

Algoritma (sengaja sederhana; arena-3d membuktikan brightness threshold
saja sudah recover lajur sampai ~1 mm dari oracle label):

1. Grayscale + ROI (di bawah `lane_roi_top`, dan di bawah horizon —
   baris di atas horizon adalah dinding, bukan lantai).
2. Binarisasi OTSU global pada ROI. Otsu dipilih karena marka dan lantai
   adalah dua mode terpisah; adaptiveThreshold TIDAK dipakai karena ia
   hanya menandai TRANSISI (interior marka lebar = sama dengan mean lokal
   = tidak terdeteksi), yang merusak logika midpoint tepi.
   Polaritas diputuskan dari strip atas ROI (far field, didominasi
   lantai): "light_on_dark" (marka terang) atau "dark_on_light".
   Aturan lomba mengizinkan dua varian marka; "auto" jalan di keduanya.
3. Sampling per-stasiun: untuk jarak maju y, proyeksikan prediksi pusat
   lajur ke piksel, cari SEMUA run marka di baris itu dalam window sempit.
   Lintasan asli = dua garis tepi SOLID + dash tengah, jadi tiap stasiun
   diklasifikasi lebih dulu:
   - >= 3 run  -> run TENGAH adalah dash; pusat lajur = pusat dash
   - 2 run berjarak ~ lebar lajur (0.5 m) -> keduanya TEPI; pusat lajur
     = midpoint
   - lainnya (1 run, atau 2 run berjarak ~0.25 m = tepi + dash) -> AMBIGU:
     kandidat pusat (run itu sendiri, run +/- setengah lebar) diskor
     terhadap prediksi. Di pass 1 stasiun ambigu DILEWATI; tanpa itu
     stasiun pertama (prediksi = 0) bisa resolve ke interpretasi tepi
     yang salah dan bias ±0.25 m mengunci diri lewat pass 2.
4. Titik (u, v) -> tanah (x, y) lewat homography, fit kuadratik robust
   dengan bobot (stasiun dekat lebih reliabel per piksel).
5. Confidence = coverage x kualitas fit; di bawah `min_confidence`
   Observation dikembalikan dengan confidence 0 = "tidak ada estimasi".

Dua pass: pass 1 hanya stasiun unambiguous (prediksi = ekstrapolasi
stasiun sebelumnya), pass 2 prediksi dari fit pass 1 — stasiun ambigu
(dash hilang / satu tepi keluar frame) baru ditambahkan di sini.
"""
from __future__ import annotations

import math

import cv2
import numpy as np

from ..types import Observation
from .calibration import CameraCalibration
from .geometry import confidence_from_fit, fit_lane, lane_state

# Lebar lajur 0.50 m (guidebook) -> setengah 0.25 m; margin pencarian
# menambah ruang untuk offset robot + noise threshold.
LANE_HALF_WIDTH_M = 0.25
SEARCH_MARGIN_M = 0.15


class LaneTracker:
    """Satu frame -> satu Observation. Tidak menyimpan state antar-frame
    (kecuali debug terakhir), jadi aman dipanggil berulang dari loop."""

    def __init__(self, *, polarity: str = "auto", roi_top: float = 0.35,
                 min_confidence: float = 0.3, near_m: float = 0.5,
                 far_m: float = 3.0, n_stations: int = 16,
                 block_frac: int = 16, c_offset: int = 6) -> None:
        if polarity not in ("auto", "light_on_dark", "dark_on_light"):
            raise ValueError(f"polarity tidak dikenal: {polarity}")
        if not 0.0 <= roi_top < 1.0:
            raise ValueError("roi_top harus fraksi [0, 1)")
        if near_m <= 0 or far_m <= near_m:
            raise ValueError("near_m/far_m tidak valid")
        if n_stations < 3:
            raise ValueError("n_stations minimal 3 (fit kuadratik)")
        self.polarity = polarity
        self.roi_top = roi_top
        self.min_confidence = min_confidence
        self.near_m = near_m
        self.far_m = far_m
        self.n_stations = n_stations
        self.block_frac = block_frac   # disimpan untuk opsi adaptive masa depan
        self.c_offset = c_offset
        self.last_debug: dict = {}

    # --- API utama ---

    def estimate(self, frame_bgr: np.ndarray, calib: CameraCalibration,
                 timestamp: float, station_m: float = 0.6) -> Observation:
        """Satu frame -> Observation. Tidak pernah melempar karena frame
        buruk; frame tanpa lajur -> confidence 0."""
        if frame_bgr is None or frame_bgr.ndim != 3:
            return self._no_lane(timestamp, reason="frame kosong")
        h, w = frame_bgr.shape[:2]
        if (w, h) != (calib.width, calib.height):
            return self._no_lane(
                timestamp,
                reason=f"ukuran frame {w}x{h} != kalibrasi "
                       f"{calib.width}x{calib.height}")

        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        roi_row = int(self.roi_top * h)
        roi = gray[roi_row:, :]

        # Baris di atas horizon = dinding, bukan lantai: buang dari
        # binarisasi DAN dari keputusan polaritas. horizon_row_worst
        # (bukan horizon_row) supaya roll yang memiringkan horizon tidak
        # membocorkan piksel dinding di salah satu tepi frame.
        start = max(0, int(calib.horizon_row_worst + 2.0) - roi_row)
        body = roi[start:]
        if body.size == 0:
            return self._no_lane(timestamp, reason="ROI di atas horizon")

        polarity = self._decide_polarity(body)
        binary = self._binarize(body, polarity)

        # Stasiun hanya di rentang yang benar-benar terlihat.
        y_vis_near, y_vis_far = calib.forward_range(self.roi_top)
        y0 = max(self.near_m, y_vis_near)
        y1 = min(self.far_m, y_vis_far)
        if y1 - y0 < 0.3:
            return self._no_lane(timestamp, reason="rentang fit terlalu sempit")
        stations_y = np.linspace(y0, y1, self.n_stations)

        def collect(predictor) -> tuple[list, list, int]:
            """Sapu stasiun; predictor(y) -> prediksi lateral (meter)."""
            pts: list[tuple[float, float]] = []
            weights: list[float] = []
            found = 0
            x_pred = 0.0
            history: list[tuple[float, float]] = []   # (forward, right)
            for y in stations_y:
                if predictor is not None:
                    x_pred = float(predictor(float(y)))
                hit = self._station_center(binary, calib, float(y), x_pred,
                                           roi_row, start, w, h,
                                           allow_ambiguous=predictor
                                           is not None)
                if hit is None:
                    continue
                u_center, v_center, _n_runs = hit
                x_ground, y_ground = calib.pixel_to_ground(u_center, v_center)
                if not (math.isfinite(x_ground) and math.isfinite(y_ground)):
                    continue
                pts.append((float(x_ground), float(y_ground)))
                weights.append(1.0 / (1.0 + float(y) ** 2))
                found += 1
                history.append((float(x_ground), float(y_ground)))
                if predictor is None:
                    # Pass 1: ekstrapolasi linear dari dua stasiun
                    # terakhir (lengkung), atau posisi terakhir.
                    if len(history) >= 2:
                        (ya, xa), (yb, xb) = history[-2], history[-1]
                        dy = yb - ya
                        slope = (xb - xa) / dy if abs(dy) > 1e-6 else 0.0
                        x_pred = xb + slope * (float(y) - yb)
                    else:
                        x_pred = float(y_ground)
            return pts, weights, found

        # --- pass 1: prediksi dari ekstrapolasi ---
        pts, weights, found = collect(None)
        fit = fit_lane(np.array(pts) if pts else np.empty((0, 2)),
                       weights=np.array(weights) if weights else None)
        if fit is not None:
            # --- pass 2: prediksi dari fit pass 1 (stasiun pertama
            # sekarang punya prior yang benar) ---
            pts2, weights2, found2 = collect(fit.lateral_at)
            fit2 = fit_lane(np.array(pts2) if pts2 else np.empty((0, 2)),
                            weights=np.array(weights2) if weights2 else None)
            if fit2 is not None:
                fit, pts, weights, found = fit2, pts2, weights2, found2

        coverage = found / len(stations_y)
        confidence = confidence_from_fit(fit, coverage)

        self.last_debug = {
            "polarity": polarity, "stations_found": found,
            "stations_total": len(stations_y), "coverage": coverage,
            "fit_range_m": [float(y0), float(y1)],
            "rms_m": None if fit is None else fit.rms_m,
            "inliers": None if fit is None else fit.inliers,
        }

        if fit is None or confidence < self.min_confidence:
            return self._no_lane(timestamp, reason="confidence di bawah ambang")

        lateral, heading, curvature = lane_state(fit, station_m)
        return Observation(
            timestamp=timestamp,
            lane_error_m=lateral,
            heading_error_rad=heading,
            curvature_inv_m=curvature,
            confidence=confidence,
            detections=({"kind": "lane", **self.last_debug},))

    # --- internal ---

    def _no_lane(self, timestamp: float, reason: str) -> Observation:
        return Observation(timestamp=timestamp, confidence=0.0,
                           detections=({"kind": "lane", "reason": reason},))

    def _decide_polarity(self, body: np.ndarray) -> str:
        """Putusan polaritas dari STRIP ATAS body (far field).

        Di jauh, lajur hanya sebagian kecil lebar frame, jadi median =
        brightness lantai. Median seluruh ROI salah — di frame dekat
        marka bisa mendominasi.
        """
        if self.polarity != "auto":
            return self.polarity
        strip = body[:max(1, body.shape[0] // 4)]
        return "dark_on_light" if float(np.median(strip)) > 127.0 \
            else "light_on_dark"

    def _binarize(self, body: np.ndarray, polarity: str) -> np.ndarray:
        """Otsu global + opening 3x3 (buang speckle noise per-piksel).

        Otsu memisahkan dua mode (lantai vs marka) memakai level frame
        ini sendiri — robust terhadap brightness global berubah, tanpa
        kelemahan adaptiveThreshold yang hanya melihat transisi.
        """
        otsu_t, _ = cv2.threshold(body, 0, 255,
                                  cv2.THRESH_BINARY | cv2.THRESH_OTSU)
        if polarity == "light_on_dark":
            mask = np.where(body > otsu_t, 255, 0).astype(np.uint8)
        else:
            mask = np.where(body < otsu_t, 255, 0).astype(np.uint8)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        return cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    def _station_center(self, binary: np.ndarray, calib: CameraCalibration,
                        y_m: float, x_pred_m: float, roi_row: int,
                        row_offset: int, width: int, height: int,
                        allow_ambiguous: bool = True):
        """Pusat lajur di stasiun y_m: (u, v, n_runs) atau None.

        y_m = jarak maju stasiun (meter); x_pred_m = prediksi posisi
        lateral pusat lajur (meter, +kanan). `row_offset` = jumlah baris
        ROI yang dibuang di atas horizon (binary hanya memuat body).
        `allow_ambiguous` = False (pass 1): stasiun yang tidak bisa
        diputuskan tanpa prediksi dikembalikan None, bukan ditebak.
        """
        u_pred, v = calib.ground_to_pixel(y_m, x_pred_m)
        v = float(v)
        if not (roi_row <= v < height - 1):
            return None
        u_lo, _ = calib.ground_to_pixel(
            y_m, x_pred_m - (LANE_HALF_WIDTH_M + SEARCH_MARGIN_M))
        u_hi, _ = calib.ground_to_pixel(
            y_m, x_pred_m + (LANE_HALF_WIDTH_M + SEARCH_MARGIN_M))
        lo = max(0, int(math.floor(min(u_lo, u_hi))))
        hi = min(width - 1, int(math.ceil(max(u_lo, u_hi))))
        if hi <= lo:
            return None

        row_idx = int(round(v)) - roi_row - row_offset
        if not 0 <= row_idx < binary.shape[0]:
            return None
        row = binary[row_idx, lo:hi + 1] > 0
        if not row.any():
            return None
        edges = np.diff(np.concatenate(([0], row.view(np.int8), [0])))
        starts = np.flatnonzero(edges == 1)
        ends = np.flatnonzero(edges == -1)
        centers = (starts + ends - 1) / 2.0 + lo
        n_runs = len(centers)

        # Setengah lebar lajur dalam piksel di stasiun ini.
        u_half, _ = calib.ground_to_pixel(y_m, x_pred_m + LANE_HALF_WIDTH_M)
        half_px = abs(float(u_half) - float(u_pred))

        # --- stasiun UNAMBIGUOUS: tidak butuh prediksi sama sekali ---
        if n_runs >= 3:
            # Tepi kiri, dash, tepi kanan: run tengah = dash = pusat.
            # Median (bukan centers[1]) supaya run noise ekstra tidak
            # menggeser hasil.
            return float(np.median(centers)), v, n_runs
        if n_runs == 2:
            gap_px = abs(float(centers[1]) - float(centers[0]))
            if gap_px >= 1.5 * half_px:
                # Jarak ~ lebar lajur (0.5 m): kedua run adalah TEPI.
                # (Tepi + dash hanya berjarak ~0.25 m = 1.0 * half_px.)
                return float((centers[0] + centers[1]) / 2.0), v, n_runs

        # --- stasiun AMBIGU: butuh prediksi ---
        if not allow_ambiguous:
            return None

        window_px = abs(float(u_hi) - float(u_lo)) / 2.0
        # Kandidat pusat: run itu sendiri (dash), run +/- setengah lebar
        # (interpretasi tepi). Kandidat terdekat prediksi menang.
        candidates = [float(c) for c in centers]
        candidates += [float(c) + half_px for c in centers]
        candidates += [float(c) - half_px for c in centers]

        best = min(candidates, key=lambda c: abs(c - float(u_pred)))
        if abs(best - float(u_pred)) > window_px:
            return None  # tidak ada kandidat masuk akal di window
        return best, v, n_runs
