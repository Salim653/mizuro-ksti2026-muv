"""Detektor semantik ONNX (YOLO11-seg) lewat cv2.dnn.

Kenapa cv2.dnn, bukan onnxruntime: runtime robot WAJIB
/usr/bin/python3 (JetPack) dan di sana tidak ada onnxruntime; cv2
ada di kedua interpreter. Trade-off: sedikit lebih lambat dari
onnxruntime-gpu, tapi nol dependency tambahan.

Layout output YOLO11-seg export (ultralytics -> onnx, simplify):
    output0: (1, 116, 8400) = 4 bbox (xywh, piksel letterbox)
                            + 80 skor kelas (sudah sigmoid)
                            + 32 koefisien mask
    output1: (1, 32, 160, 160) = prototipe mask

Semua fungsi decode di sini murni numpy (tanpa cv2) supaya bisa
diuji dengan array sintetis — tidak perlu file model.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

# COCO-80 (model pretrained ultralytics). Model hasil training arena
# nanti ganti lewat file labels (argumen `labels`).
COCO_LABELS = (
    "person", "bicycle", "car", "motorcycle", "airplane", "bus",
    "train", "truck", "boat", "traffic light", "fire hydrant",
    "stop sign", "parking meter", "bench", "bird", "cat", "dog",
    "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe",
    "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat",
    "baseball glove", "skateboard", "surfboard", "tennis racket",
    "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl",
    "banana", "apple", "sandwich", "orange", "broccoli", "carrot",
    "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop",
    "mouse", "remote", "keyboard", "cell phone", "microwave", "oven",
    "toaster", "sink", "refrigerator", "book", "clock", "vase",
    "scissors", "teddy bear", "hair drier", "toothbrush",
)

MASK_THRESHOLD = 0.5


@dataclass
class Detection:
    """Satu deteksi. bbox & mask di koordinat frame ASLI."""

    class_id: int
    label: str
    confidence: float
    bbox: tuple[float, float, float, float]     # (x, y, w, h) piksel
    mask: np.ndarray | None = None              # bool (h, w) frame asli
    meta: dict = field(default_factory=dict)


def letterbox(frame: np.ndarray, size: int = 640,
              pad_value: int = 114) -> tuple[np.ndarray, dict]:
    """Resize preserve aspect ratio + pad. Return (blob siap-infer, meta).

    meta memuat apa yang dibutuhkan buat memetakan balik ke frame asli:
    scale, pad_x, pad_y, orig_h, orig_w.
    """
    import cv2
    h, w = frame.shape[:2]
    scale = min(size / w, size / h)
    new_w, new_h = int(round(w * scale)), int(round(h * scale))
    resized = cv2.resize(frame, (new_w, new_h),
                         interpolation=cv2.INTER_LINEAR)
    pad_x = (size - new_w) / 2.0
    pad_y = (size - new_h) / 2.0
    top, bottom = int(round(pad_y - 0.1)), int(round(pad_y + 0.1))
    left, right = int(round(pad_x - 0.1)), int(round(pad_x + 0.1))
    canvas = np.full((size + top + bottom, size + left + right, 3),
                     pad_value, dtype=np.uint8)
    canvas[top:top + new_h, left:left + new_w] = resized
    # cv2 butuh ukuran TEPAT; crop/pad supaya pas `size`.
    canvas = canvas[:size, :size]
    blob = cv2.dnn.blobFromImage(canvas, scalefactor=1 / 255.0,
                                 swapRB=True)
    meta = {"scale": scale, "pad_x": pad_x, "pad_y": pad_y,
            "orig_h": h, "orig_w": w, "size": size}
    return blob, meta


def _iou_matrix(boxes: np.ndarray) -> np.ndarray:
    """IoU antar semua pasang bbox xywh. Return (N, N)."""
    if len(boxes) == 0:
        return np.zeros((0, 0))
    x1 = boxes[:, 0] - boxes[:, 2] / 2
    y1 = boxes[:, 1] - boxes[:, 3] / 2
    x2 = boxes[:, 0] + boxes[:, 2] / 2
    y2 = boxes[:, 1] + boxes[:, 3] / 2
    area = np.maximum(x2 - x1, 0) * np.maximum(y2 - y1, 0)
    iou = np.zeros((len(boxes), len(boxes)))
    for i in range(len(boxes)):
        xx1 = np.maximum(x1[i], x1)
        yy1 = np.maximum(y1[i], y1)
        xx2 = np.minimum(x2[i], x2)
        yy2 = np.minimum(y2[i], y2)
        inter = np.maximum(xx2 - xx1, 0) * np.maximum(yy2 - yy1, 0)
        union = area[i] + area - inter
        iou[i] = np.where(union > 0, inter / np.maximum(union, 1e-9), 0.0)
    return iou


def nms_per_class(boxes: np.ndarray, scores: np.ndarray,
                  class_ids: np.ndarray, iou_thresh: float
                  ) -> list[int]:
    """NMS greedy per-kelas. Return index yang lolos (terurut skor desc).

    Per-kelas wajib: dua objek kelas beda yang overlap tidak boleh
    saling menekan (misal orang di depan mobil).
    """
    if len(boxes) == 0:
        return []
    iou_all = _iou_matrix(boxes)          # (N, N) sekali, bukan per-box
    order = np.argsort(-scores)
    keep: list[int] = []
    suppressed = np.zeros(len(boxes), dtype=bool)
    for idx in order:
        if suppressed[idx]:
            continue
        keep.append(int(idx))
        same_class = (class_ids == class_ids[idx])
        suppress = same_class & (iou_all[idx] > iou_thresh)
        suppress[idx] = False
        suppressed |= suppress
    return keep


def decode_yolo_seg(out0: np.ndarray, out1: np.ndarray, meta: dict, *,
                    conf_thresh: float = 0.5, iou_thresh: float = 0.45,
                    class_filter: set[int] | None = None,
                    max_det: int = 50) -> list[dict]:
    """Decode output YOLO11-seg. Return list dict (belum ke frame asli).

    Tiap dict: class_id, confidence, bbox (xywh di ruang letterbox),
    mask (bool 160x160, sudah di-crop ke bbox di ruang proto).
    Fungsi murni numpy — bisa diuji tanpa model.
    """
    pred = np.asarray(out0, dtype=float).squeeze()      # (116, 8400)
    if pred.ndim != 2 or pred.shape[0] < 5:
        raise ValueError(f"layout output0 tak dikenali: {pred.shape}")
    pred = pred.T                                      # (8400, 116)
    boxes_xywh = pred[:, :4]
    class_scores = pred[:, 4:]
    n_coeff = pred.shape[1] - 4
    # 80 kelas + 32 koefisien = 112; kalau lebih, ada margin.
    n_classes = n_coeff - 32 if n_coeff > 32 else n_coeff
    if n_classes <= 0:
        raise ValueError(f"tidak ada skor kelas di output0: {pred.shape}")

    scores = class_scores[:, :n_classes]
    class_ids = np.argmax(scores, axis=1)
    confidences = scores[np.arange(len(scores)), class_ids]

    mask_coeffs = pred[:, n_classes:n_classes + 32] \
        if pred.shape[1] >= n_classes + 32 else None

    alive = confidences >= conf_thresh
    if class_filter is not None:
        alive &= np.isin(class_ids, list(class_filter))
    idx = np.where(alive)[0]
    if len(idx) == 0:
        return []

    keep = nms_per_class(boxes_xywh[idx], confidences[idx],
                         class_ids[idx], iou_thresh)[:max_det]

    protos = np.asarray(out1, dtype=float).squeeze()    # (32, 160, 160)
    proto_h, proto_w = protos.shape[1], protos.shape[2]
    proto_flat = protos.reshape(protos.shape[0], -1)    # (32, 25600)

    size = meta["size"]
    results = []
    for i in keep:
        j = idx[i]
        entry = {
            "class_id": int(class_ids[j]),
            "confidence": float(confidences[j]),
            "bbox": tuple(float(v) for v in boxes_xywh[j]),
            "mask": None,
        }
        if mask_coeffs is not None:
            logits = mask_coeffs[j] @ proto_flat        # (25600,)
            mask160 = 1.0 / (1.0 + np.exp(-logits))
            mask160 = mask160.reshape(proto_h, proto_w)
            entry["mask"] = mask160 > MASK_THRESHOLD
            entry["mask160"] = mask160
        results.append(entry)
    return results


def mask_to_original(mask160: np.ndarray, bbox_xywh: tuple, meta: dict,
                     thresh: float = MASK_THRESHOLD) -> np.ndarray:
    """Mask proto (160x160) -> bool di frame ASLI, di-crop ke bbox.

    Urutan: proto -> ruang letterbox (resize ke `size`) -> crop bbox
    (nol di luar) -> peta balik lewat skala/pad letterbox -> frame asli.
    """
    import cv2
    size = meta["size"]
    orig_h, orig_w = meta["orig_h"], meta["orig_w"]
    scale, pad_x, pad_y = meta["scale"], meta["pad_x"], meta["pad_y"]

    mask_lb = cv2.resize(mask160.astype(np.float32), (size, size),
                         interpolation=cv2.INTER_LINEAR)
    # Crop ke bbox di ruang letterbox.
    cx, cy, bw, bh = bbox_xywh
    x0 = max(0, int(cx - bw / 2))
    y0 = max(0, int(cy - bh / 2))
    x1 = min(size, int(math.ceil(cx + bw / 2)))
    y1 = min(size, int(math.ceil(cy + bh / 2)))
    cropped = np.zeros((size, size), dtype=np.float32)
    if x1 > x0 and y1 > y0:
        cropped[y0:y1, x0:x1] = mask_lb[y0:y1, x0:x1]

    # Peta balik letterbox -> frame asli.
    ox0 = int(round((x0 - pad_x) / scale))
    oy0 = int(round((y0 - pad_y) / scale))
    ox1 = int(round((x1 - pad_x) / scale))
    oy1 = int(round((y1 - pad_y) / scale))
    ox0, oy0 = max(0, ox0), max(0, oy0)
    ox1, oy1 = min(orig_w, ox1), min(orig_h, oy1)
    out = np.zeros((orig_h, orig_w), dtype=bool)
    if ox1 > ox0 and oy1 > oy0:
        region = cv2.resize(cropped[y0:y1, x0:x1], (ox1 - ox0, oy1 - oy0),
                            interpolation=cv2.INTER_LINEAR)
        out[oy0:oy1, ox0:ox1] = region > thresh
    return out


class SemanticDetector:
    """cv2.dnn wrapper: frame BGR -> list[Detection] di frame asli."""

    def __init__(self, model_path: str, threshold: float = 0.5, *,
                 input_size: int = 640, nms_iou: float = 0.45,
                 labels: tuple[str, ...] | None = None,
                 class_filter: set[int] | None = None) -> None:
        import cv2
        self.cv2 = cv2
        self.net = cv2.dnn.readNetFromONNX(model_path)
        self.threshold = threshold
        self.input_size = input_size
        self.nms_iou = nms_iou
        self.labels = labels or COCO_LABELS
        self.class_filter = class_filter
        self.last_inference_ms = 0.0

    def _label(self, class_id: int) -> str:
        if 0 <= class_id < len(self.labels):
            return self.labels[class_id]
        return f"class{class_id}"

    def infer(self, frame: np.ndarray) -> list[Detection]:
        """Satu frame BGR -> deteksi (bbox + mask di frame asli)."""
        import time
        blob, meta = letterbox(frame, self.input_size)
        self.net.setInput(blob)
        t0 = time.monotonic()
        outs = self.net.forward(self.net.getUnconnectedOutLayersNames())
        self.last_inference_ms = (time.monotonic() - t0) * 1000.0
        out0, out1 = outs[0], outs[1]
        raw = decode_yolo_seg(out0, out1, meta,
                              conf_thresh=self.threshold,
                              iou_thresh=self.nms_iou,
                              class_filter=self.class_filter)
        detections = []
        for entry in raw:
            mask = None
            if entry.get("mask") is not None:
                mask = mask_to_original(entry["mask"], entry["bbox"], meta)
            detections.append(Detection(
                class_id=entry["class_id"],
                label=self._label(entry["class_id"]),
                confidence=entry["confidence"],
                bbox=entry["bbox"],
                mask=mask))
        return detections
