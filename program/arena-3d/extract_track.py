"""Extract the KSTI 2026 Mini Autonomous Vehicle track geometry from the rendered map.

Inputs : ../track_map_200dpi.png   (raster of the Drive PDF, 200 dpi)
Outputs: track_data.json     centre / outer / inner polylines in metres + width, radius, s
         track_floor.jpg     floor texture for the 3D scene
         sanity_overlay.png  visual check of every fitted curve

Pipeline
  1. white-pixel connected components; the two that span >50% of both axes are the edge
     lines (outer = larger convex hull), the small ones are the centre dashes.
  2. band = fill(outer) - fill(inner); its RETR_CCOMP contours give the two edge polylines
     in exact walking order (polar fitting smears across the corners, so it is not used).
  3. centreline seeded from the dash centroids (polar fit is fine for a single-valued
     dashed line), then refined by marching along the local normal and taking the midpoint
     of the two band edges -> true midline, and the march distance is the perpendicular width.
  4. scale: median perpendicular width = 0.50 m (guidebook III.C.1 "lebar lintasan 50 cm").
"""
import json
import math
import os

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, os.pardir, "track_map_200dpi.png")
OUT_JSON = os.path.join(HERE, "track_data.json")
OUT_TEX = os.path.join(HERE, "track_floor.jpg")
NOMINAL_WIDTH_M = 0.50
NS = 1440          # centreline samples
NB = 512           # polar bins


def smooth_circular(a, k):
    """Gaussian smooth a periodic 1-D array."""
    ker = np.exp(-0.5 * (np.arange(-3 * k, 3 * k + 1) / k) ** 2)
    ker /= ker.sum()
    pad = np.concatenate([a[-3 * k:], a, a[:3 * k]])
    return np.convolve(pad, ker, "same")[3 * k:-3 * k]


def polar_from_points(px, py, centre, k=20, agg=np.mean):
    v = np.stack([px - centre[0], py - centre[1]], 1)
    ang = np.arctan2(v[:, 1], v[:, 0])
    rad = np.hypot(v[:, 1], v[:, 0])
    b = ((ang + math.pi) / (2 * math.pi) * NB).astype(int) % NB
    rr = np.full(NB, np.nan)
    good = np.zeros(NB, bool)
    for i in range(NB):
        s = b == i
        if s.any():
            rr[i] = agg(rad[s])
            good[i] = True
    idx = np.arange(NB)
    rr = np.interp(idx, idx[good], rr[good])
    rr = smooth_circular(rr, k)
    th = (idx + 0.5) / NB * 2 * math.pi - math.pi
    return np.stack([centre[0] + rr * np.cos(th), centre[1] + rr * np.sin(th)], 1)


def closed_resample(P, n):
    """Uniform arc-length resample of a closed polyline."""
    Q = np.vstack([P, P[:1]])
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(Q, axis=0), axis=1))])
    sq = np.linspace(0, s[-1], n, endpoint=False)
    return np.stack([np.interp(sq, s, Q[:, 0]), np.interp(sq, s, Q[:, 1])], 1), s[-1]


def tangent(P, k=8):
    tx = smooth_circular(np.gradient(P[:, 0]), k)
    ty = smooth_circular(np.gradient(P[:, 1]), k)
    nn = np.hypot(tx, ty)
    return np.stack([tx / nn, ty / nn], 1)


def main():
    img = cv2.imread(SRC, cv2.IMREAD_COLOR)
    assert img is not None, SRC
    H, W = img.shape[:2]
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    mask = (gray > 190).astype(np.uint8)
    nlab, lab, stats, cents = cv2.connectedComponentsWithStats(mask, 8)
    n = int(nlab)

    loops = [i for i in range(1, n)
             if stats[i, 4] > 0.002 * W * H and stats[i, 2] > 0.5 * W and stats[i, 3] > 0.5 * H]
    assert len(loops) >= 2, f"expected 2 edge loops, got {loops}"

    def hull_area(i):
        c = cv2.findContours((lab == i).astype(np.uint8), cv2.RETR_EXTERNAL,
                             cv2.CHAIN_APPROX_SIMPLE)[0][0]
        return cv2.contourArea(cv2.convexHull(c))

    outer_id, inner_id = sorted(loops, key=lambda i: -hull_area(i))[:2]

    def filled(i):
        c = cv2.findContours((lab == i).astype(np.uint8), cv2.RETR_EXTERNAL,
                             cv2.CHAIN_APPROX_NONE)[0][0]
        f = np.zeros((H, W), np.uint8)
        cv2.drawContours(f, [c], -1, 1, cv2.FILLED)
        return f

    band = (filled(outer_id) & (1 - filled(inner_id))).astype(np.uint8)
    cnts, hier = cv2.findContours(band * 255, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    hier = hier[0]
    ext = [i for i in range(len(cnts)) if hier[i][3] < 0]
    oi = max(ext, key=lambda i: len(cnts[i]))
    hole = max((i for i in range(len(cnts)) if hier[i][3] == oi), key=lambda i: len(cnts[i]))
    outer_px = cnts[oi].reshape(-1, 2).astype(float)
    inner_px = cnts[hole].reshape(-1, 2).astype(float)
    mo = cv2.moments(band)
    centre = np.array([mo["m10"] / mo["m00"], mo["m01"] / mo["m00"]])
    print(f"image {W}x{H}  outer#{outer_id} inner#{inner_id}  "
          f"band={band.sum()}px  edges {len(outer_px)}/{len(inner_px)} pts")

    # ---- seed centreline from dash centroids --------------------------------
    band_d = cv2.distanceTransform(1 - band, cv2.DIST_L2, 3)
    cand = [(float(cents[i][0]), float(cents[i][1]))
            for i in range(1, n) if 15 < stats[i, 4] < 0.0008 * W * H]
    D = np.array([c for c in cand if band_d[int(round(c[1])), int(round(c[0]))] <= 40])
    print(f"dashes kept {len(D)}/{len(cand)}")
    seed = closed_resample(polar_from_points(D[:, 0], D[:, 1], centre, k=22), NS)[0]

    # ---- refine to the true midline by normal marching ----------------------
    band_u = band.astype(bool)

    def march(p, nvec, limit=500.0, step=2.0):
        d = 0.0
        while d < limit:
            d += step
            x = int(round(p[0] + nvec[0] * d))
            y = int(round(p[1] + nvec[1] * d))
            if not (0 <= x < W and 0 <= y < H) or not band_u[y, x]:
                return max(0.0, d - step)
        return limit

    C = seed.copy()
    for it in range(4):
        T = tangent(C, k=6)
        newC = C.copy()
        ws = np.zeros(len(C))
        for i in range(len(C)):
            nx, ny = -T[i, 1], T[i, 0]
            a = march(C[i], (nx, ny))
            b = march(C[i], (-nx, -ny))
            if a + b < 20:                      # fell off the band: pull toward centre
                v = centre - C[i]
                C[i] = C[i] + v / np.hypot(*v) * 6
                continue
            newC[i] = C[i] + np.array((nx, ny)) * ((a - b) / 2.0)
            ws[i] = a + b
        C = newC.copy()
        C[:, 0] = smooth_circular(newC[:, 0], 3)
        C[:, 1] = smooth_circular(newC[:, 1], 3)

    # cusp cleanup: wide smoothing then re-snap to the midline (a real 1:10 track has no
    # direction reversals at 1.4 cm sample spacing)
    for _ in range(4):
        C[:, 0] = smooth_circular(C[:, 0], 22)
        C[:, 1] = smooth_circular(C[:, 1], 22)
        Tt = tangent(C, k=6)
        for i in range(len(C)):
            nx, ny = -Tt[i, 1], Tt[i, 0]
            a = march(C[i], (nx, ny))
            b = march(C[i], (-nx, -ny))
            if a + b > 20:
                C[i] = C[i] + np.array((nx, ny)) * ((a - b) / 2.0)

    C, per_px = closed_resample(C, NS)
    T = tangent(C, k=6)
    band_w_px = np.array([march(C[i], (-T[i, 1], T[i, 0])) +
                          march(C[i], (T[i, 1], -T[i, 0])) for i in range(len(C))])
    band_w_px = smooth_circular(band_w_px, 10)

    # the band runs from the OUTER edge of the outer line to the OUTER edge of the inner
    # line, so the drivable gap is band width minus one outer-line thickness.
    oc = cv2.findContours((lab == outer_id).astype(np.uint8), cv2.RETR_EXTERNAL,
                          cv2.CHAIN_APPROX_NONE)[0][0]
    t_line = stats[outer_id, 4] / cv2.arcLength(oc, True)
    width_px = band_w_px - t_line
    ppm = float(np.median(width_px)) / NOMINAL_WIDTH_M
    print(f"line thickness {t_line:.1f}px  band {np.median(band_w_px):.1f}px  "
          f"gap {np.median(width_px):.1f}px -> {ppm:.2f}px/m")
    print(f"loop {per_px/ppm:.2f} m  drivable width {width_px.min()/ppm:.2f}-{width_px.max()/ppm:.2f} m")

    # curvature in 1/m from metre-space derivatives
    def curvature(P):
        gx, gy = np.gradient(P[:, 0]) / ppm, np.gradient(P[:, 1]) / ppm
        hxx, hyy = np.gradient(gx), np.gradient(gy)
        return (gx * hyy - gy * hxx) / np.power(gx ** 2 + gy ** 2, 1.5)

    # racing line = periodic low-pass of the midline: keeps the loop shape, drops the
    # sub-centimetre wobble inherited from the painted dashes
    def lowpass(P, harm):
        cols = []
        for c in (0, 1):
            F = np.fft.rfft(P[:, c])
            F[harm:] = 0
            cols.append(np.fft.irfft(F, len(P)))
        return np.stack(cols, 1)

    R = lowpass(C, 30)
    R, _ = closed_resample(R, NS)
    radius_m = [round(float(1.0 / abs(c)) if abs(c) > 1e-3 else 999.0, 3)
                for c in curvature(R)]
    print(f"racing-line radius: min {min(radius_m):.2f} m  "
          f"p5 {np.percentile([r for r in radius_m if r < 900], 5):.2f} m  "
          f"straight {sum(1 for r in radius_m if r > 20)} / {NS} samples")

    outer_r, _ = closed_resample(outer_px, NS)
    inner_r, _ = closed_resample(inner_px, NS)

    def to_m(P):
        return [[round(float(p[0] - centre[0]) / ppm, 4),
                 round(float(p[1] - centre[1]) / ppm, 4)] for p in P]

    centre_m, outer_m, inner_m = to_m(C), to_m(outer_r), to_m(inner_r)
    path_m = to_m(R)
    s_m = [round(float(i) * per_px / NS / ppm, 4) for i in range(NS)]
    tex = cv2.resize(img, (int(W * 1600 / H), 1600), interpolation=cv2.INTER_AREA)
    cv2.imwrite(OUT_TEX, tex, [cv2.IMWRITE_JPEG_QUALITY, 88])

    ov = img.copy()
    for P, col, r in ((outer_r, (0, 255, 0), 5), (inner_r, (0, 128, 255), 5), (C, (255, 0, 255), 4)):
        for p in P:
            cv2.circle(ov, (int(p[0]), int(p[1])), r, col, -1)
    for x, y in D:
        cv2.circle(ov, (int(x), int(y)), 3, (0, 220, 220), -1)
    cv2.imwrite(os.path.join(HERE, "sanity_overlay.png"),
               cv2.resize(ov, (int(W * 1300 / H), 1300)))

    data = dict(
        meta=dict(
            source="KSTI 2026 Mini AV lintasan (Drive 1wFD-KYt6aSOmciczAfKIlE6q_Z4m5Lsh)",
            px_per_m=round(ppm, 4), nominal_width_m=NOMINAL_WIDTH_M,
            loop_length_m=round(per_px / ppm, 3), samples=NS,
            width_min_m=round(float(width_px.min() / ppm), 3),
            width_max_m=round(float(width_px.max() / ppm), 3),
            min_radius_m=round(min(r for r in radius_m if r < 900), 3),
            tex_wh=[int(tex.shape[1]), int(tex.shape[0])], image_wh=[W, H],
            centre_px=[round(float(centre[0]), 2), round(float(centre[1]), 2)],
            note="metres; origin = loop centre; img x->world x, img y->world z (y up)",
        ),
        centre=centre_m, path=path_m, outer=outer_m, inner=inner_m,
        width=[round(float(w / ppm), 4) for w in width_px],
        radius=radius_m, s=s_m,
    )
    with open(OUT_JSON, "w", encoding="utf-8") as f:
        json.dump(data, f, separators=(",", ":"))
    print("wrote", os.path.basename(OUT_JSON), os.path.getsize(OUT_JSON), "bytes")


if __name__ == "__main__":
    main()
