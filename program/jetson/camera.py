from __future__ import annotations

import sys


def has_gstreamer(cv2) -> bool:
    for line in cv2.getBuildInformation().splitlines():
        if "GStreamer:" in line:
            return line.split("GStreamer:")[1].strip().upper().startswith("YES")
    return False


def nvargus_pipeline(width: int, height: int, fps: int = 30, sensor: int = 0) -> str:
    """Jetson CSI pipeline ending in an appsink that yields BGR frames."""
    return (
        f"nvarguscamerasrc sensor-id={sensor} ! "
        f"video/x-raw(memory:NVMM),width={width},height={height},"
        f"framerate={fps}/1,format=NV12 ! "
        "nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! "
        "video/x-raw,format=BGR ! appsink drop=1 max-buffers=1 sync=false"
    )


class Camera:
    def __init__(self, pipeline: str, width: int, height: int) -> None:
        import cv2
        self.cv2 = cv2
        if not has_gstreamer(cv2):
            raise RuntimeError(
                f"cv2 {cv2.__version__} at {sys.executable} has no GStreamer backend; "
                "run this with /usr/bin/python3 (JetPack OpenCV), not the repo .venv")
        self.pipeline = pipeline or nvargus_pipeline(width, height)
        self.cap = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            raise RuntimeError(f"GStreamer camera pipeline failed to open: {self.pipeline}")
        self.width, self.height = width, height

    def read(self):
        ok, frame = self.cap.read()
        if not ok or frame is None:
            raise RuntimeError("camera frame read failed")
        if frame.shape[1] != self.width or frame.shape[0] != self.height:
            raise RuntimeError(f"unexpected frame shape: {frame.shape}")
        return frame

    def close(self) -> None:
        self.cap.release()
