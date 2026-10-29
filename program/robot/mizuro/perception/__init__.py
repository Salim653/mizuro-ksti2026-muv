"""Persepsi lajur klasik (Fase 1).

Modul:
- calibration : model kamera -> homography ke bidang tanah (konvensi sim arena-3d)
- geometry    : matematika polinomial murni (fit, error, curvatur) — tanpa cv2
- classical   : LaneTracker (threshold adaptif + sampling per-stasiun)
- synth       : generator frame sintetis dengan ground truth eksak

Koordinat tanah (car frame, mengikuti annotations.jsonl arena-3d):
    x = maju (forward), y = kanan (right), satuan meter.
Laju dimodelkan x_center(y) = a*y^2 + b*y + c.
"""
