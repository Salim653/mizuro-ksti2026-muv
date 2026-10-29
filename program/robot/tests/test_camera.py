"""Tes kamera: penjagaan GStreamer, validasi ukuran, opener suntikan.

Tidak ada hardware di sini. `opener` disuntikkan sebagai pengganti
cv2.VideoCapture; penjagaan build-info diuji lewat monkeypatch.
"""
import numpy as np
import pytest

from mizuro.camera import Camera, has_gstreamer, nvargus_pipeline
from mizuro.config import Config


class FakeCapture:
    """cv2.VideoCapture tiruan: bisa diprogram gagal/beda ukuran."""

    def __init__(self, pipeline, frames=None, fail_open=False,
                 shape=(480, 640, 3)):
        self.pipeline = pipeline
        self.frames = list(frames or [])
        self.fail_open = fail_open
        self.shape = shape
        self.released = False
        self.reads = 0

    def isOpened(self):
        return not self.fail_open

    def read(self):
        self.reads += 1
        if self.frames:
            return True, self.frames.pop(0)
        return True, np.zeros(self.shape, dtype=np.uint8)

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
        Camera("pipa-uji", 640, 480,
               opener=lambda p: FakeCapture(p, fail_open=True))


def test_read_returns_frame(monkeypatch):
    monkeypatch.setattr("mizuro.camera.has_gstreamer", lambda cv2: True)
    cam = Camera("pipa-uji", 640, 480,
                 opener=lambda p: FakeCapture(p, shape=(480, 640, 3)))
    frame = cam.read()
    assert frame.shape == (480, 640, 3)
    assert cam.pipeline == "pipa-uji"


def test_read_rejects_wrong_size(monkeypatch):
    monkeypatch.setattr("mizuro.camera.has_gstreamer", lambda cv2: True)
    cam = Camera("pipa-uji", 640, 480,
                 opener=lambda p: FakeCapture(p, shape=(240, 320, 3)))
    with pytest.raises(RuntimeError, match="ukuran frame tak terduga"):
        cam.read()


def test_read_failure_raises(monkeypatch):
    monkeypatch.setattr("mizuro.camera.has_gstreamer", lambda cv2: True)

    class DeadCapture(FakeCapture):
        def read(self):
            return False, None

    cam = Camera("pipa-uji", 640, 480, opener=lambda p: DeadCapture(p))
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
