"""Tes detektor semantik: decode murni + NMS + mask mapping.

Decode diuji dengan array sintetis (layout YOLO11-seg asli) — tidak
perlu file model. Integrasi cv2.dnn di-skip kalau ONNX tidak ada.
"""
from pathlib import Path

import numpy as np
import pytest

from mizuro.perception.detector import (
    COCO_LABELS,
    Detection,
    decode_yolo_seg,
    letterbox,
    mask_to_original,
    nms_per_class,
)

N_ANCHORS = 8400
N_PROTO = 32


def _synth_outputs(objects, seed=0):
    """Bangun (out0, out1) sintetis layout YOLO11-seg.

    objects: list (class_id, conf, cx, cy, w, h) di ruang 640x640.
    Sisanya noise skor rendah.
    """
    rng = np.random.default_rng(seed)
    out0 = np.zeros((116, N_ANCHORS), dtype=np.float32)
    out0[4:84, :] = rng.uniform(0.0, 0.05, (80, N_ANCHORS))  # noise kelas
    out0[84:116, :] = rng.uniform(-1.0, 1.0, (32, N_ANCHORS))
    for i, (cls, conf, cx, cy, w, h) in enumerate(objects):
        out0[0, i] = cx
        out0[1, i] = cy
        out0[2, i] = w
        out0[3, i] = h
        out0[4 + cls, i] = conf
    out1 = rng.uniform(-2.0, 2.0, (N_PROTO, 160, 160)).astype(np.float32)
    return out0, out1


META = {"scale": 1.0, "pad_x": 0.0, "pad_y": 0.0,
        "orig_h": 480, "orig_w": 640, "size": 640}


def test_letterbox_preserves_aspect_and_meta():
    frame = np.zeros((480, 640, 3), dtype=np.uint8)   # 4:3 -> pad kiri/kanan
    blob, meta = letterbox(frame, 640)
    assert blob.shape == (1, 3, 640, 640)
    assert meta["scale"] == pytest.approx(1.0)
    assert meta["orig_h"] == 480 and meta["orig_w"] == 640
    # Frame portrait: scale ditentukan tinggi, pad horizontal.
    tall = np.zeros((800, 400, 3), dtype=np.uint8)
    _, meta2 = letterbox(tall, 640)
    assert meta2["scale"] == pytest.approx(0.8)
    assert meta2["pad_x"] > meta2["pad_y"]


def test_decode_finds_high_confidence_objects():
    out0, out1 = _synth_outputs([
        (0, 0.9, 100.0, 100.0, 50.0, 50.0),      # person
        (39, 0.8, 300.0, 200.0, 40.0, 80.0),     # bottle
    ])
    dets = decode_yolo_seg(out0, out1, META, conf_thresh=0.5)
    assert len(dets) == 2
    by_cls = {d["class_id"]: d for d in dets}
    assert by_cls[0]["confidence"] == pytest.approx(0.9, abs=1e-6)
    assert by_cls[0]["bbox"] == pytest.approx((100.0, 100.0, 50.0, 50.0))
    assert by_cls[39]["confidence"] == pytest.approx(0.8, abs=1e-6)
    # Mask proto ikut ter-decode.
    assert by_cls[0]["mask"].shape == (160, 160)
    assert by_cls[0]["mask"].dtype == bool


def test_decode_respects_confidence_threshold():
    out0, out1 = _synth_outputs([(0, 0.4, 100.0, 100.0, 50.0, 50.0)])
    assert decode_yolo_seg(out0, out1, META, conf_thresh=0.5) == []
    assert len(decode_yolo_seg(out0, out1, META, conf_thresh=0.3)) == 1


def test_decode_class_filter():
    out0, out1 = _synth_outputs([
        (0, 0.9, 100.0, 100.0, 50.0, 50.0),
        (39, 0.8, 300.0, 200.0, 40.0, 80.0),
    ])
    dets = decode_yolo_seg(out0, out1, META, conf_thresh=0.5,
                           class_filter={39})
    assert len(dets) == 1
    assert dets[0]["class_id"] == 39


def test_decode_rejects_bad_layout():
    # 1-D dan layout terlalu sempit harus ditolak, bukan crash aneh.
    with pytest.raises(ValueError, match="layout"):
        decode_yolo_seg(np.zeros(10, dtype=np.float32),
                        np.zeros((32, 160, 160), dtype=np.float32), META)
    with pytest.raises(ValueError, match="layout"):
        decode_yolo_seg(np.zeros((3, 100), dtype=np.float32),
                        np.zeros((32, 160, 160), dtype=np.float32), META)


def test_nms_suppresses_overlap_same_class_only():
    boxes = np.array([[100.0, 100.0, 50.0, 50.0],
                      [102.0, 101.0, 48.0, 52.0],     # overlap kelas sama
                      [400.0, 300.0, 60.0, 60.0]])    # jauh, kelas sama
    scores = np.array([0.9, 0.8, 0.7])
    classes = np.array([0, 0, 0])
    keep = nms_per_class(boxes, scores, classes, iou_thresh=0.45)
    assert keep == [0, 2]                            # yang overlap tersapu

    # Kelas beda di posisi sama: keduanya lolos.
    classes2 = np.array([0, 1, 0])
    keep2 = nms_per_class(boxes, scores, classes2, iou_thresh=0.45)
    assert set(keep2) == {0, 1, 2}


def test_nms_keeps_highest_score_first():
    boxes = np.array([[100.0, 100.0, 50.0, 50.0],
                      [101.0, 99.0, 50.0, 50.0]])
    scores = np.array([0.5, 0.95])
    keep = nms_per_class(boxes, scores, np.array([0, 0]), 0.45)
    assert keep == [1]                               # skor tinggi menang


def test_mask_to_original_identity_meta():
    # scale 1, pad 0: mask harus mendarat di koordinat sama.
    # Putih di proto [40:80, 40:80] -> ruang 640 = [160:320, 160:320],
    # jadi bbox harus MENUTUP region itu (cx=cy=240, w=h=160).
    mask160 = np.zeros((160, 160), dtype=np.float32)
    mask160[40:80, 40:80] = 1.0
    bbox = (240.0, 240.0, 160.0, 160.0)
    out = mask_to_original(mask160, bbox, META)
    assert out.shape == (480, 640)
    assert out.dtype == bool
    assert out[240, 240]                    # dalam bbox & wilayah putih
    assert not out[10, 10]                  # luar bbox = False
    assert not out[100, 100]                # dalam bbox tapi wilayah hitam


def test_mask_to_original_scales_with_letterbox():
    # scale 0.5, pad 0, orig 1280x1280: letterbox 640 -> asli 1280.
    # Putih proto [80:160, 80:160] -> ruang 640 = [320:640, 320:640].
    # bbox (480, 480, 320, 320) -> crop [320:640, 320:640] = persis putih.
    meta = {"scale": 0.5, "pad_x": 0.0, "pad_y": 0.0,
            "orig_h": 1280, "orig_w": 1280, "size": 640}
    mask160 = np.zeros((160, 160), dtype=np.float32)
    mask160[80:160, 80:160] = 1.0
    bbox = (480.0, 480.0, 320.0, 320.0)
    out = mask_to_original(mask160, bbox, meta)
    assert out.shape == (1280, 1280)
    # bbox asli: (320/0.5, 640/0.5) = (640, 1280) -> putih penuh di situ.
    assert out[900, 900]
    assert not out[100, 100]
    assert not out[1279, 100]              # luar bbox di salah satu sumbu


def test_coco_labels_count():
    assert len(COCO_LABELS) == 80
    assert COCO_LABELS[0] == "person"
    assert COCO_LABELS[39] == "bottle"


# --- integrasi model asli (skip kalau ONNX tidak ada) ---

_ONNX = Path(__file__).resolve().parents[3] / "yolo11n-seg.onnx"


@pytest.mark.skipif(not _ONNX.is_file(), reason="yolo11n-seg.onnx tidak ada")
class TestDetectorIntegration:
    def test_loads_and_runs_on_synthetic_frame(self):
        from mizuro.perception.detector import SemanticDetector
        det = SemanticDetector(str(_ONNX), threshold=0.5)
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        result = det.infer(frame)
        assert isinstance(result, list)
        assert det.last_inference_ms >= 0.0

    def test_detects_bottle_in_bench_frame(self):
        """Frame bench (meja + kaleng + botol) harus menghasilkan
        minimal satu deteksi kelas COCO nyata."""
        from mizuro.perception.detector import SemanticDetector
        import cv2
        frames = sorted(Path("runs/bench-frames").glob("frame-*.jpg"))
        if not frames:
            pytest.skip("tidak ada frame bench tersimpan")
        det = SemanticDetector(str(_ONNX), threshold=0.4)
        frame = cv2.imread(str(frames[-1]))
        assert frame is not None
        result = det.infer(frame)
        assert len(result) >= 1
        for d in result:
            assert isinstance(d, Detection)
            assert d.label in COCO_LABELS
            assert 0.0 < d.confidence <= 1.0
            x, y, w, h = d.bbox
            assert 0 <= x <= 640 and 0 <= y <= 480
            if d.mask is not None:
                assert d.mask.shape == frame.shape[:2]
