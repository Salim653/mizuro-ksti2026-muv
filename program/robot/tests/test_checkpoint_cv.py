"""Tes detektor checkpoint classical CV — frame sintetis, tanpa hardware.

Frame dibuat dengan numpy: blob warna solid di atas background gelap,
plus bentuk-bentuk yang HARUS ditolak (kecil, terlalu besar, tidak
padat). Tidak ada file eksternal, tidak ada model.
"""
import numpy as np
import pytest

from mizuro.config import Config
from mizuro.perception.checkpoint_cv import (COLOR_PRESETS,
                                             CheckpointDetectorCV)
from mizuro.perception.detector import Detection

H, W = 480, 640


def _blank():
    return np.zeros((H, W, 3), dtype=np.uint8)


def _bgr_from_hsv(h, s=255, v=255):
    """Warna BGR solid dari nilai HSV (H 0-179 seperti OpenCV)."""
    px = np.uint8([[[h, s, v]]])
    return tuple(int(c) for c in cv2_cvt(px)[0, 0])


def cv2_cvt(px):
    import cv2
    return cv2.cvtColor(px, cv2.COLOR_HSV2BGR)


def _rect(frame, x, y, w, h, color_bgr):
    frame[y:y + h, x:x + w] = color_bgr
    return frame


@pytest.fixture
def det():
    return CheckpointDetectorCV(color="green")


# --- deteksi dasar -----------------------------------------------------------

def test_detects_solid_green_blob(det):
    frame = _rect(_blank(), 300, 200, 40, 80, _bgr_from_hsv(60))
    result = det.infer(frame)
    assert len(result) == 1
    d = result[0]
    assert isinstance(d, Detection)
    assert d.label == "checkpoint"
    assert d.class_id == 0
    assert d.confidence > 0.9            # persegi solid: solidity ~1.0
    x, y, w, h = d.bbox
    assert (x, y, w, h) == (300, 200, 40, 80)
    assert d.mask is not None
    assert d.mask.shape == (H, W)
    # Morphology membulatkan sudut: mask ~ area blob, bukan persis.
    assert abs(int(d.mask.sum()) - 40 * 80) <= 64


def test_empty_frame_returns_empty_list(det):
    assert det.infer(_blank()) == []


def test_none_frame_returns_empty_list(det):
    assert det.infer(None) == []


def test_wrong_color_not_detected(det):
    """Blob biru tidak lolos detektor hijau."""
    frame = _rect(_blank(), 300, 200, 40, 80, _bgr_from_hsv(110))
    assert det.infer(frame) == []


def test_multiple_blobs_sorted_by_area_desc(det):
    frame = _blank()
    _rect(frame, 50, 50, 20, 20, _bgr_from_hsv(60))     # 400 px
    _rect(frame, 300, 200, 40, 80, _bgr_from_hsv(60))   # 3200 px
    result = det.infer(frame)
    assert len(result) == 2
    assert result[0].meta["area_px"] > result[1].meta["area_px"]


# --- filter area ---------------------------------------------------------------

def test_small_blob_rejected(det):
    """5x5 px < min_area 300 — dan morphology open menghabisinya."""
    frame = _rect(_blank(), 300, 200, 5, 5, _bgr_from_hsv(60))
    assert det.infer(frame) == []


def test_blob_below_custom_min_area_rejected():
    d = CheckpointDetectorCV(color="green", min_area_px=10000)
    frame = _rect(_blank(), 300, 200, 40, 80, _bgr_from_hsv(60))  # 3200 px
    assert d.infer(frame) == []


def test_blob_above_max_area_rejected():
    d = CheckpointDetectorCV(color="green", max_area_px=1000)
    frame = _rect(_blank(), 300, 200, 40, 80, _bgr_from_hsv(60))
    assert d.infer(frame) == []


# --- filter solidity -------------------------------------------------------------

def test_thin_diagonal_rejected_by_solidity(det):
    """Garis diagonal tebal: area cukup, tapi solidity sangat rendah."""
    import cv2
    frame = _blank()
    cv2.line(frame, (100, 100), (400, 400), _bgr_from_hsv(60), thickness=8)
    result = det.infer(frame)
    assert result == []


def test_solidity_threshold_configurable():
    """Ambiguitas bentuk: batas 0.5 menolak L-shape (solidity ~0.55?).
    Dipakai untuk memastikan threshold benar-benar diterapkan."""
    import cv2
    frame = _blank()
    # Plus/cross: solidity ~0.36, area > 300.
    cv2.rectangle(frame, (270, 180), (310, 300), _bgr_from_hsv(60), -1)
    cv2.rectangle(frame, (220, 230), (360, 250), _bgr_from_hsv(60), -1)
    loose = CheckpointDetectorCV(color="green", min_solidity=0.2)
    strict = CheckpointDetectorCV(color="green", min_solidity=0.8)
    assert len(loose.infer(frame)) == 1
    assert strict.infer(frame) == []


# --- preset warna -----------------------------------------------------------------

@pytest.mark.parametrize("color,hsv_h", [
    ("green", 60), ("red", 0), ("orange", 15),
    ("yellow", 30), ("blue", 110),
])
def test_color_presets(color, hsv_h):
    d = CheckpointDetectorCV(color=color)
    frame = _rect(_blank(), 300, 200, 40, 80, _bgr_from_hsv(hsv_h))
    result = d.infer(frame)
    assert len(result) == 1
    assert result[0].label == "checkpoint"


def test_unknown_color_falls_back_to_green():
    d = CheckpointDetectorCV(color="ungu-aneh")
    frame = _rect(_blank(), 300, 200, 40, 80, _bgr_from_hsv(60))
    assert len(d.infer(frame)) == 1


# --- HSV override -------------------------------------------------------------------

def test_explicit_hsv_range_overrides_preset():
    """Rentang sempit hanya menangkap satu nilai H."""
    d = CheckpointDetectorCV(hsv_low=(55, 50, 50), hsv_high=(65, 255, 255))
    frame = _blank()
    _rect(frame, 300, 200, 40, 80, _bgr_from_hsv(60))    # lolos
    _rect(frame, 50, 50, 40, 80, _bgr_from_hsv(90))      # ditolak
    result = d.infer(frame)
    assert len(result) == 1
    assert result[0].bbox[0] == 300


def test_mask_returns_binary_image(det):
    frame = _rect(_blank(), 300, 200, 40, 80, _bgr_from_hsv(60))
    m = det.mask(frame)
    assert m.dtype == np.uint8
    assert m.shape == (H, W)
    assert set(np.unique(m)) <= {0, 255}
    # Sudut membulat oleh morphology: toleransi kecil.
    assert abs(m.sum() // 255 - 40 * 80) <= 64


# --- from_config ---------------------------------------------------------------------

def test_from_config_uses_preset():
    cfg = Config(checkpoint_mode="cv", checkpoint_color="red",
                 checkpoint_min_area_px=500)
    d = CheckpointDetectorCV.from_config(cfg)
    assert d.hsv_low == COLOR_PRESETS["red"][0]
    assert d.hsv_high == COLOR_PRESETS["red"][1]
    assert d.min_area_px == 500


def test_from_config_uses_explicit_hsv():
    cfg = Config(checkpoint_mode="cv",
                 checkpoint_hsv_low=(40, 60, 60),
                 checkpoint_hsv_high=(70, 255, 255))
    d = CheckpointDetectorCV.from_config(cfg)
    assert d.hsv_low == (40, 60, 60)
    assert d.hsv_high == (70, 255, 255)


def test_from_config_label_and_solidity():
    cfg = Config(checkpoint_label="gate-merah",
                 checkpoint_min_solidity=0.7)
    d = CheckpointDetectorCV.from_config(cfg)
    assert d.label == "gate-merah"
    assert d.min_solidity == 0.7
    frame = _rect(_blank(), 300, 200, 40, 80, _bgr_from_hsv(60))
    assert d.infer(frame)[0].label == "gate-merah"


# --- kinerja --------------------------------------------------------------------------

def test_inference_is_fast(det):
    """CV klasik harus jauh lebih cepat dari YOLO (~270 ms)."""
    import time
    frame = _rect(_blank(), 300, 200, 40, 80, _bgr_from_hsv(60))
    t0 = time.perf_counter()
    for _ in range(20):
        det.infer(frame)
    avg_ms = (time.perf_counter() - t0) / 20 * 1000.0
    assert avg_ms < 50.0        # longgar: laptop apa pun harus lolos
    assert det.last_inference_ms >= 0.0
