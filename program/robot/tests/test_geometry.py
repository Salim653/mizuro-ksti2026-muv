"""Tes geometri persepsi: fit polinomial, state lajur, confidence.

Titik tanah di semua modul persepsi: (forward_m, right_m) — kolom 0 maju,
kolom 1 lateral. Polinomial memetakan forward -> lateral.
"""
import numpy as np
import pytest

from mizuro.perception.geometry import (
    DEFAULT_ORDER,
    confidence_from_fit,
    fit_lane,
    lane_state,
)


def _curve(a, b, c, ys):
    """Titik (forward, right) pada x = a*y^2 + b*y + c."""
    return np.array([(y, a * y * y + b * y + c) for y in ys])


def test_fit_recovers_exact_quadratic():
    ys = np.linspace(0.5, 2.0, 12)
    pts = _curve(0.03, -0.05, 0.11, ys)
    fit = fit_lane(pts)
    assert fit is not None
    assert fit.rms_m < 1e-9
    assert fit.n_points == 12
    np.testing.assert_allclose(fit.coeffs[::-1], [0.03, -0.05, 0.11], atol=1e-9)


def test_fit_rejects_degenerate_point_sets():
    # Terlalu sedikit titik untuk kuadratik.
    assert fit_lane(_curve(0.0, 0.0, 0.0, [1.0, 1.2])) is None
    # Semua titik di baris forward yang sama (span < 0.2 m).
    same_row = np.array([(1.0, 0.1), (1.05, 0.2), (1.1, 0.05)])
    assert fit_lane(same_row) is None
    # Bentuk salah.
    with pytest.raises(ValueError):
        fit_lane(np.zeros((5, 3)))


def test_fit_robust_drops_outlier():
    ys = np.linspace(0.5, 2.0, 12)
    pts = _curve(0.02, 0.01, 0.05, ys)
    pts[4] = (ys[4], 0.05 + 0.5)          # outlier 50 cm
    fit = fit_lane(pts, robust=True)
    assert fit is not None
    assert fit.inliers == 11
    assert fit.rms_m < 0.01
    # Tanpa robust, outlier merusak fit.
    fit_raw = fit_lane(pts, robust=False)
    assert fit_raw.rms_m > 0.1


def test_weights_change_the_fit():
    ys = np.linspace(0.5, 2.0, 12)
    pts = _curve(0.02, 0.01, 0.05, ys)
    pts[10:] += np.array([0.0, 0.04])     # stasiun jauh digeser 4 cm
    w_near = np.array([1.0 / (1.0 + y * y) for y in ys])
    fit_w = fit_lane(pts, weights=w_near)
    fit_flat = fit_lane(pts)
    # Bobot dekat harus menahan fit lebih dekat ke titik dekat.
    assert abs(fit_w.lateral_at(0.6) - 0.05 - 0.02 * 0.36 - 0.01 * 0.6) < 0.01
    assert fit_flat.lateral_at(0.6) != fit_w.lateral_at(0.6)


def test_lane_state_matches_analytic_truth():
    ys = np.linspace(0.5, 2.0, 12)
    a, b, c = 0.04, -0.06, 0.09
    fit = fit_lane(_curve(a, b, c, ys))
    for station in (0.6, 1.0, 1.5):
        lateral, heading, curvature = lane_state(fit, station)
        slope = 2.0 * a * station + b
        assert lateral == pytest.approx(a * station * station + b * station + c)
        assert heading == pytest.approx(np.arctan(slope))
        assert curvature == pytest.approx(
            2.0 * a / (1.0 + slope * slope) ** 1.5)


def test_lane_state_heading_uses_tangent_not_intercept():
    # c = 0.3 (lajur jauh di kanan di forward=0) tapi lurus: heading 0.
    ys = np.linspace(0.5, 2.0, 12)
    fit = fit_lane(_curve(0.0, 0.0, 0.3, ys))
    _, heading, curvature = lane_state(fit, 1.0)
    assert heading == pytest.approx(0.0)
    assert curvature == pytest.approx(0.0)


def test_confidence_zero_without_fit_or_coverage():
    assert confidence_from_fit(None, 1.0) == 0.0
    ys = np.linspace(0.5, 2.0, 12)
    fit = fit_lane(_curve(0.02, 0.01, 0.05, ys))
    assert confidence_from_fit(fit, 0.3) == 0.0      # coverage < 0.5
    assert confidence_from_fit(fit, 1.0) > 0.9       # fit bersih, coverage penuh


def test_confidence_penalises_noisy_fit():
    ys = np.linspace(0.5, 2.0, 12)
    clean = fit_lane(_curve(0.02, 0.01, 0.05, ys))
    noisy_pts = _curve(0.02, 0.01, 0.05, ys)
    rng = np.random.default_rng(3)
    noisy_pts[:, 1] += rng.normal(0, 0.03, len(ys))
    noisy = fit_lane(noisy_pts)
    assert confidence_from_fit(clean, 1.0) > confidence_from_fit(noisy, 1.0)


def test_default_order_is_quadratic():
    assert DEFAULT_ORDER == 2
