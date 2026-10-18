class Camera:
    def __init__(self, pipeline: str, width: int, height: int) -> None:
        import cv2
        self.cv2 = cv2
        self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            raise RuntimeError("GStreamer camera pipeline failed to open")
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
