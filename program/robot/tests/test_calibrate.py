"""Tes tools/calibrate.py — fungsi murni + kalibrasi sintetis.

Tidak ada kamera di sini: checkerboard dirender/diproyeksikan
sintetis dengan K & distorsi yang DIKETAHUI, lalu alat kalibrasi
dijalankan dan intrinsik-nya harus ter-recover.
"""
import importlib.util
from pathlib import Path

import cv2
import numpy as np
import pytest

_spec = importlib.util.spec_from_file_location(
    "calibrate_tool",
    Path(__file__).resolve().parent.parent / "tools" / "calibrate.py")
calibrate_tool = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(calibrate_tool)

PATTERN = (9, 6)
SQUARE_M = 0.025


def test_object_points_shape_and_spacing():
    objp = calibrate_tool.object_points(PATTERN, SQUARE_M)
    assert objp.shape == (54, 3)
    assert objp[:, 2].sum() == 0.0                    # planar, z = 0
    # Jarak antar sudut bertetangga = ukuran kotak.
    assert objp[1, 0] - objp[0, 0] == pytest.approx(SQUARE_M)
    assert objp[PATTERN[0], 1] - objp[0, 1] == pytest.approx(SQUARE_M)


def _render_checkerboard(shape=(480, 640), square_px=40, origin=(60, 100)):
    """Checkerboard standar: kotak GELAP di background TERANG.

    OpenCV findChessboardCorners gagal di pola terbalik (kotak terang
    di background gelap) — cetakan kalibrasi memang selalu hitam-di-
    atas-putih, jadi tes mengikuti itu.
    """
    img = np.full(shape, 255, dtype=np.uint8)
    rows, cols = PATTERN[1] + 1, PATTERN[0] + 1
    oy, ox = origin
    for i in range(rows):
        for j in range(cols):
            if (i + j) % 2 == 1:
                y, x = oy + i * square_px, ox + j * square_px
                img[y:y + square_px, x:x + square_px] = 0
    return img


def test_find_corners_detects_clean_checkerboard():
    img = _render_checkerboard()
    found, corners = calibrate_tool.find_corners(img, PATTERN)
    assert found
    assert corners.shape == (54, 1, 2)
    assert corners.dtype == np.float32


def test_find_corners_rejects_blank_image():
    blank = np.full((480, 640), 128, dtype=np.uint8)
    found, corners = calibrate_tool.find_corners(blank, PATTERN)
    assert not found
    assert corners is None


def _synthetic_views(n_views=12, seed=7):
    """Proyeksikan papan dengan K & distorsi DIKETAHUI dari berbagai sudut."""
    k_true = np.array([[550.0, 0, 320.0],
                       [0, 550.0, 240.0],
                       [0, 0, 1.0]])
    dist_true = np.array([-0.25, 0.08, 0.0, 0.0, 0.0])
    objp = calibrate_tool.object_points(PATTERN, SQUARE_M)
    rng = np.random.default_rng(seed)
    objpoints, imgpoints = [], []
    for _ in range(n_views):
        rvec = np.array([rng.uniform(-0.35, 0.35),
                         rng.uniform(-0.35, 0.35),
                         rng.uniform(-0.25, 0.25)])
        tvec = np.array([rng.uniform(-0.06, 0.06),
                         rng.uniform(-0.06, 0.06),
                         rng.uniform(0.45, 0.65)])
        pts, _ = cv2.projectPoints(objp, rvec, tvec, k_true, dist_true)
        objpoints.append(objp)
        imgpoints.append(pts.astype(np.float32))
    return objpoints, imgpoints, k_true, dist_true


def test_calibrate_recovers_known_intrinsics():
    objpoints, imgpoints, k_true, dist_true = _synthetic_views()
    k_rec, dist_rec, rpe = calibrate_tool.calibrate(
        objpoints, imgpoints, (480, 640))
    # Intrinsik harus recover dalam ~2% (data bersih, tanpa noise).
    assert k_rec[0, 0] == pytest.approx(k_true[0, 0], rel=0.02)
    assert k_rec[1, 1] == pytest.approx(k_true[1, 1], rel=0.02)
    assert k_rec[0, 2] == pytest.approx(k_true[0, 2], abs=3.0)
    assert k_rec[1, 2] == pytest.approx(k_true[1, 2], abs=3.0)
    # Distorsi barrel (k1) harus positif-terekover tanda & magnitudo.
    assert dist_rec[0, 0] == pytest.approx(dist_true[0], rel=0.15)
    # Data sintetis tanpa noise -> rpe harus kecil.
    assert rpe < 0.5


def test_calibrate_zero_tangent_by_design():
    """Alat pakai CALIB_ZERO_TANGENT_DIST: p1/p2 hasil = 0."""
    objpoints, imgpoints, _, _ = _synthetic_views()
    _, dist_rec, _ = calibrate_tool.calibrate(
        objpoints, imgpoints, (480, 640))
    assert dist_rec[0, 2] == pytest.approx(0.0)   # p1
    assert dist_rec[0, 3] == pytest.approx(0.0)   # p2


def test_coverage_hint_regions():
    corners = np.array([[[100.0, 100.0]]])        # pojok kiri-atas
    assert "kiri" in calibrate_tool.coverage_hint(corners, (480, 640))
    assert "atas" in calibrate_tool.coverage_hint(corners, (480, 640))
    corners = np.array([[[600.0, 400.0]]])        # pojok kanan-bawah
    assert "kanan" in calibrate_tool.coverage_hint(corners, (480, 640))
    assert "bawah" in calibrate_tool.coverage_hint(corners, (480, 640))
