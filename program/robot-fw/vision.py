from __future__ import annotations

from typing import Any

from core import Observation


class OnnxDetector:
    def __init__(self, model_path: str, threshold: float) -> None:
        import cv2
        self.cv2 = cv2
        self.net = cv2.dnn.readNetFromONNX(model_path)
        self.threshold = threshold

    def infer(self, frame) -> list[dict[str, Any]]:
        h, w = frame.shape[:2]
        blob = self.cv2.dnn.blobFromImage(frame, 1 / 255.0, (640, 640), swapRB=True)
        self.net.setInput(blob)
        raw = self.net.forward()
        # shortcut: model-specific output decoding is not implemented; add after fixing ONNX export layout.
        return [{"raw_shape": list(raw.shape), "image_size": [w, h], "threshold": self.threshold}]


class LaneEstimator:
    def estimate(self, frame, detections: list[dict[str, Any]], now: float) -> Observation:
        # shortcut: lane model and ground projection require camera calibration and detector output semantics.
        return Observation(timestamp=now, detections=detections)
