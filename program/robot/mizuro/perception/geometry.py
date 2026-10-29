"""Matematika polinomial lajur — murni numpy, tanpa cv2.

Laju dimodelkan sebagai kuadratik di frame tanah:

    x_center(y) = a*y^2 + b*y + c        (x maju, y kanan, meter)

Kenawa kuadratik dan bukan garis: radius minimum lintasan 1.5 m, dan dari
mount 0.16 m lajur terlihat sampai ~3 m — di rentang itu lengkungan sudah
terasa. Kuadratik cukup (radius konstan = lengkungan lingkaran) dan tidak
overfit seperti orde tinggi.

Konvensi tanda (dipakai Observation di types.py):
- lane_error_m positif  = pusat lajur di KANAN robot (robot harus belok kanan)
- heading_error_rad positif = arah lajur mengarah ke KANAN heading robot
- curvature_inv_m positif = lajur belok kanan (frame x-kanan/y-maju)

Titik tanah di seluruh modul ini: (forward_m, right_m) — kolom pertama
jarak maju, kolom kedua posisi lateral. Polinomial memetakan forward ->
lateral.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# Orde polinomial. 2 = kuadratik; cukup untuk radius >= ~1 m pada rentang
# fit ~3 m dan tidak goyah pada noise piksel.
DEFAULT_ORDER = 2

# Titik dengan residu lebih besar dari ini dibuang sebagai outlier lalu
# fit diulang (mis. frame saat lajur ambigu di tengah tikungan).
OUTLIER_FACTOR = 2.0
OUTLIER_FLOOR_M = 0.03


@dataclass(frozen=True)
class LaneFit:
    """Hasil fit: koefisien + diagnostik."""

    coeffs: np.ndarray          # orde menaik: [c, b, a] untuk a*y^2+b*y+c
    rms_m: float                # RMS residu lateral (meter)
    inliers: int
    n_points: int

    def lateral_at(self, forward_m: float) -> float:
        """Posisi lateral pusat lajur pada jarak maju (meter)."""
        return float(np.polyval(self.coeffs[::-1], forward_m))

    def slope_at(self, forward_m: float) -> float:
        """d(lateral)/d(forward) — tan(arah lajur relatif heading robot)."""
        dcoeffs = np.polyder(self.coeffs[::-1])
        return float(np.polyval(dcoeffs, forward_m))

    def curvature_inv_at(self, forward_m: float) -> float:
        """1/R bertanda (positif = belok kanan)."""
        dd = np.polyder(self.coeffs[::-1], 2)
        d1 = self.slope_at(forward_m)
        return float(np.polyval(dd, forward_m) / (1.0 + d1 * d1) ** 1.5)


def fit_lane(points_xy: np.ndarray, order: int = DEFAULT_ORDER,
             robust: bool = True,
             weights: np.ndarray | None = None) -> LaneFit | None:
    """Fit lateral(forward) least-squares dari titik (N, 2) = (maju, kanan).

    `weights` opsional (N,): stasiun dekat lebih reliabel per piksel
    (skala piksel->meter tumbuh dengan jarak), jadi fit bisa membobotnya
    lebih tinggi. RMS residu yang dilaporkan selalu tak berbobot.

    Mengembalikan None kalau titik tidak cukup (orde+1 setelah outlier
    rejection) — pemanggil wajib memperlakukannya sebagai "tidak ada lajur".
    """
    pts = np.asarray(points_xy, dtype=float)
    if pts.ndim != 2 or pts.shape[1] != 2:
        raise ValueError("points_xy harus berbentuk (N, 2)")
    if len(pts) < order + 1:
        return None
    if weights is not None:
        weights = np.asarray(weights, dtype=float)
        if weights.shape != (len(pts),) or np.any(weights < 0):
            raise ValueError("weights harus (N,) non-negatif")

    fwd, lat = pts[:, 0], pts[:, 1]   # kolom 0 = maju, kolom 1 = lateral
    if np.ptp(fwd) < 0.2:
        return None  # semua titik di baris yang sama: fit tak berarti

    def _fit(mask: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        w = None if weights is None else weights[mask]
        coeffs_desc = np.polyfit(fwd[mask], lat[mask], order, w=w)
        resid = lat - np.polyval(coeffs_desc, fwd)
        return coeffs_desc, resid

    mask = np.ones(len(pts), dtype=bool)
    coeffs_desc, resid = _fit(mask)
    rms = float(np.sqrt(np.mean(resid[mask] ** 2)))

    if robust and len(pts) >= order + 3:
        threshold = max(OUTLIER_FACTOR * rms, OUTLIER_FLOOR_M)
        candidate = np.abs(resid) <= threshold
        if candidate.sum() >= order + 1 and not candidate.all():
            mask = candidate
            coeffs_desc, resid = _fit(mask)
            rms = float(np.sqrt(np.mean(resid[mask] ** 2)))

    return LaneFit(coeffs=coeffs_desc[::-1], rms_m=rms,
                   inliers=int(mask.sum()), n_points=len(pts))


def lane_state(fit: LaneFit, station_m: float) -> tuple[float, float, float]:
    """(lane_error_m, heading_error_rad, curvature_inv_m) pada stasiun maju.

    heading = atan(d(lateral)/d(forward)): sudut arah lajur relatif heading
    robot, positif ke kanan. Dihitung dari TANGEN di stasiun, bukan dari
    koefisien c — c hanya posisi lateral di forward=0 (di belakang kamera).
    """
    lateral = fit.lateral_at(station_m)
    heading = float(np.arctan(fit.slope_at(station_m)))
    curvature = fit.curvature_inv_at(station_m)
    return lateral, heading, curvature


def confidence_from_fit(fit: LaneFit | None, coverage: float,
                        min_coverage: float = 0.5) -> float:
    """Skor [0, 1]: coverage (berapa stasiun ketemu) x kualitas fit.

    coverage rendah (banyak dash/gap) atau RMS residu besar -> confidence
    turun; controller memperlakukan confidence < lane_min_confidence sebagai
    "tidak ada estimasi" dan mengirim nol.
    """
    if fit is None or coverage < min_coverage:
        return 0.0
    quality = max(0.0, 1.0 - fit.rms_m / 0.10)
    return float(coverage * quality)
