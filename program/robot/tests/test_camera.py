"""Tes kamera: penjagaan GStreamer, validasi ukuran, opener suntikan,
dan sumber file video (bench tanpa hardware).

Tidak ada hardware di sini. `opener` disuntikkan sebagai pengganti
cv2.VideoCapture; penjagaan build-info diuji lewat monkeypatch.
"""
import numpy as np
import pytest

from mizuro.camera import (
    Camera,
    EndOfVideo,
    has_gstreamer,
    is_gstreamer_pipeline,
    nvargus_pipeline,
)
from mizuro.config import Config

# Pipeline GStreamer (ada '!') vs path file video (tanpa '!').
GST_PIPE = "nvarguscamerasrc ! videoconvert ! appsink"
FILE_PIPE = "recordings/video-uji.mp4"


class FakeCapture:
    """cv2.VideoCapture tiruan: bisa diprogram gagal/beda ukuran/loop."""

    def __init__(self, pipeline, frames=None, fail_open=False,
                 shape=(480, 640, 3), fail_read_after=None):
        self.pipeline = pipeline
        self._initial = list(frames or [])
        self.frames = list(self._initial)
        self.fail_open = fail_open
        self.shape = shape
        self.released = False
        self.reads = 0
        self.seeks = 0
        # None = read() selalu sukses (zeros kalau frames habis);
        # N = gagal setelah N pembacaan (mirror video selesai).
        self.fail_read_after = fail_read_after

    def isOpened(self):
        return not self.fail_open

    def read(self):
        self.reads += 1
        if self.fail_read_after is not None and self.reads > self.fail_read_after:
            return False, None
        if self.frames:
            return True, self.frames.pop(0)
        return True, np.zeros(self.shape, dtype=np.uint8)

    def set(self, prop, value):
        self.seeks += 1
        if value == 0:                      # CAP_PROP_POS_FRAMES = 0
            self.frames = list(self._initial)
            self.reads = 0                  # penghitung kegagalan ikut reset
        return True

    def get(self, prop):
        return 60.0

    def release(self):
        self.released = True


def test_pipeline_contains_csi_elements():
    p = nvargus_pipeline(1280, 720, fps=30, sensor=0)
    assert "nvarguscamerasrc sensor-id=0" in p
    assert "width=1280,height=720" in p
    assert "framerate=30/1" in p
    assert "format=BGR ! appsink" in p


def test_has_gstreamer_returns_bool():
    import cv2
    assert isinstance(has_gstreamer(cv2), bool)


def test_rejects_build_without_gstreamer(monkeypatch):
    monkeypatch.setattr("mizuro.camera.has_gstreamer", lambda cv2: False)
    with pytest.raises(RuntimeError, match="/usr/bin/python3"):
        Camera("", 640, 480, opener=lambda p: FakeCapture(p))


def test_open_failure_message_mentions_other_process(monkeypatch):
    monkeypatch.setattr("mizuro.camera.has_gstreamer", lambda cv2: True)
    with pytest.raises(RuntimeError, match="kamera gagal dibuka"):
        Camera(GST_PIPE, 640, 480,
               opener=lambda p: FakeCapture(p, fail_open=True))


def test_read_returns_frame(monkeypatch):
    monkeypatch.setattr("mizuro.camera.has_gstreamer", lambda cv2: True)
    cam = Camera("pipa-uji", 640, 480,
                 opener=lambda p: FakeCapture(p, shape=(480, 640, 3)))
    frame = cam.read()
    assert frame.shape == (480, 640, 3)
    assert cam.pipeline == "pipa-uji"


def test_read_rejects_wrong_size(monkeypatch):
    """Sumber GStreamer: ukuran beda dari kalibrasi DITOLAK (bug homography)."""
    monkeypatch.setattr("mizuro.camera.has_gstreamer", lambda cv2: True)
    cam = Camera(GST_PIPE, 640, 480,
                 opener=lambda p: FakeCapture(p, shape=(240, 320, 3)))
    with pytest.raises(RuntimeError, match="ukuran frame tak terduga"):
        cam.read()


def test_read_failure_raises(monkeypatch):
    monkeypatch.setattr("mizuro.camera.has_gstreamer", lambda cv2: True)

    class DeadCapture(FakeCapture):
        def read(self):
            return False, None

    cam = Camera(GST_PIPE, 640, 480, opener=lambda p: DeadCapture(p))
    with pytest.raises(RuntimeError, match="pembacaan frame"):
        cam.read()


def test_close_releases_and_context_manager(monkeypatch):
    monkeypatch.setattr("mizuro.camera.has_gstreamer", lambda cv2: True)
    holder = {}

    def opener(p):
        holder["cap"] = FakeCapture(p)
        return holder["cap"]

    with Camera("pipa-uji", 640, 480, opener=opener) as cam:
        cam.read()
    assert holder["cap"].released is True
    assert cam.cap is None


def test_default_pipeline_used_when_empty(monkeypatch):
    monkeypatch.setattr("mizuro.camera.has_gstreamer", lambda cv2: True)
    seen = {}

    def opener(p):
        seen["p"] = p
        return FakeCapture(p)

    cam = Camera("", 1280, 720, opener=opener)
    assert "nvarguscamerasrc" in seen["p"]
    assert "width=1280,height=720" in seen["p"]


def test_from_config(monkeypatch):
    monkeypatch.setattr("mizuro.camera.has_gstreamer", lambda cv2: True)
    cfg = Config(camera_pipeline="pipa-cfg", image_width=640, image_height=480)
    cam = Camera.from_config(cfg, opener=lambda p: FakeCapture(p))
    assert cam.pipeline == "pipa-cfg"
    assert (cam.width, cam.height) == (640, 480)


def test_undistort_changes_frame_but_not_size(monkeypatch):
    """distorsi nyata (barrel) -> isi frame berubah, ukuran sama."""
    import cv2
    monkeypatch.setattr("mizuro.camera.has_gstreamer", lambda cv2: True)
    # Frame dengan gradien: distorsi barrel pasti menggeser piksel.
    base = np.tile(np.arange(640, dtype=np.uint8), (480, 1))
    frame = cv2.merge([base, base, base])
    k = np.array([[500.0, 0, 320.0], [0, 500.0, 240.0], [0, 0, 1.0]])
    dist = np.array([-0.3, 0.1, 0.0, 0.0, 0.0])   # barrel kuat
    cam = Camera("pipa-uji", 640, 480,
                 opener=lambda p: FakeCapture(p, frames=[frame.copy()]),
                 undistort=(k, dist))
    out = cam.read()
    assert out.shape == (480, 640, 3)
    assert not np.array_equal(out, frame)      # benar-benar dikoreksi


def test_no_undistort_passes_frame_through(monkeypatch):
    """Tanpa undistort, frame lewat apa adanya (perilaku lama)."""
    monkeypatch.setattr("mizuro.camera.has_gstreamer", lambda cv2: True)
    frame = np.full((480, 640, 3), 77, dtype=np.uint8)
    cam = Camera("pipa-uji", 640, 480,
                 opener=lambda p: FakeCapture(p, frames=[frame.copy()]))
    assert np.array_equal(cam.read(), frame)


def test_from_config_passes_undistort(monkeypatch):
    monkeypatch.setattr("mizuro.camera.has_gstreamer", lambda cv2: True)
    cfg = Config(camera_pipeline="pipa-cfg", image_width=640, image_height=480)
    k = np.eye(3)
    dist = np.zeros(5)
    cam = Camera.from_config(cfg, opener=lambda p: FakeCapture(p),
                             undistort=(k, dist))
    assert cam._undistort is not None


# --- sumber file video (bench tanpa hardware) -------------------------------

def test_is_gstreamer_pipeline():
    assert is_gstreamer_pipeline(GST_PIPE)
    assert is_gstreamer_pipeline(nvargus_pipeline(640, 480))
    assert not is_gstreamer_pipeline("")
    assert not is_gstreamer_pipeline(FILE_PIPE)
    assert not is_gstreamer_pipeline("/home/kazero/laps/capture_001.mp4")


def test_file_source_does_not_require_gstreamer(monkeypatch):
    """Video file boleh jalan dengan cv2 pip .venv (tanpa GStreamer)."""
    monkeypatch.setattr("mizuro.camera.has_gstreamer", lambda cv2: False)
    cam = Camera(FILE_PIPE, 640, 480, opener=lambda p: FakeCapture(p))
    assert cam.is_file is True
    assert cam.pipeline == FILE_PIPE


def test_file_open_failure_message_mentions_file(monkeypatch):
    monkeypatch.setattr("mizuro.camera.has_gstreamer", lambda cv2: False)
    with pytest.raises(RuntimeError, match="video gagal dibuka"):
        Camera(FILE_PIPE, 640, 480,
               opener=lambda p: FakeCapture(p, fail_open=True))


def test_file_read_failure_raises_end_of_video(monkeypatch):
    """Video habis = EndOfVideo (bukan RuntimeError kamera)."""
    monkeypatch.setattr("mizuro.camera.has_gstreamer", lambda cv2: False)
    cam = Camera(FILE_PIPE, 640, 480,
                 opener=lambda p: FakeCapture(p, fail_read_after=0))
    with pytest.raises(EndOfVideo, match="video selesai"):
        cam.read()


def test_file_video_loop_restarts(monkeypatch):
    """video_loop=True: habis -> seek ke 0 -> frame pertama lagi."""
    monkeypatch.setattr("mizuro.camera.has_gstreamer", lambda cv2: False)
    frames = [np.full((480, 640, 3), i, dtype=np.uint8) for i in (10, 20)]
    holder = {}

    def opener(p):
        holder["cap"] = FakeCapture(p, frames=frames, fail_read_after=2)
        return holder["cap"]

    cam = Camera(FILE_PIPE, 640, 480, opener=opener, video_loop=True)
    assert cam.read()[0, 0, 0] == 10
    assert cam.read()[0, 0, 0] == 20
    assert cam.read()[0, 0, 0] == 10        # loop: frame pertama lagi
    assert holder["cap"].seeks == 1


def test_file_read_resizes_to_config_size(monkeypatch):
    """Video rekaman jarang persis resolusi config -> di-resize."""
    monkeypatch.setattr("mizuro.camera.has_gstreamer", lambda cv2: False)
    frame = np.full((240, 320, 3), 99, dtype=np.uint8)
    cam = Camera(FILE_PIPE, 640, 480,
                 opener=lambda p: FakeCapture(p, frames=[frame]))
    out = cam.read()
    assert out.shape == (480, 640, 3)


def test_file_source_fps_from_capture(monkeypatch):
    monkeypatch.setattr("mizuro.camera.has_gstreamer", lambda cv2: False)
    cam = Camera(FILE_PIPE, 640, 480, opener=lambda p: FakeCapture(p))
    assert cam.source_fps == 60.0
