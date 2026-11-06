"""Tes bench server: kontrol sumber kamera runtime + status hidup.

Tidak ada hardware di sini. `opener` disuntikkan sebagai pengganti
cv2.VideoCapture (pola sama seperti tests/test_camera.py), tracker
diganti stub supaya yang diuji adalah KONTROL (tukar sumber, status,
thread), bukan akurasi persepsi — itu sudah diuji test_classical.py.

Yang dikunci di sini:

- tukar sumber CSI <-> video bekerja dan tidak restart proses,
- sumber gagal dibuka -> sumber LAMA tetap aktif (tidak ada keadaan
  setengah tukar),
- nama video dari body divalidasi ketat (traversal / ekstensi /
  di luar direktori ditolak),
- video habis / kamera mati / tracker error TIDAK membunuh loop,
- stop() benar-benar menghentikan thread dan melepas kamera.
"""
import threading
import time

import numpy as np
import pytest

from mizuro.bench_server import VIDEO_SUFFIXES, BenchControl
from mizuro.camera import CameraSource, EndOfVideo
from mizuro.config import Config
from mizuro.perception.calibration import CameraCalibration
from mizuro.types import Observation

WIDTH, HEIGHT = 64, 48
GST_PIPE = "nvarguscamerasrc ! videoconvert ! appsink"


class FakeCapture:
    """cv2.VideoCapture tiruan yang bisa diprogram per-pipeline."""

    def __init__(self, pipeline, frames=None, fail_open=False,
                 shape=(HEIGHT, WIDTH, 3), fail_read_after=None):
        self.pipeline = pipeline
        self._initial = list(frames or [])
        self.frames = list(self._initial)
        self.fail_open = fail_open
        self.shape = shape
        self.released = False
        self.reads = 0
        self.seeks = 0
        self.fail_read_after = fail_read_after

    def isOpened(self):
        return not self.fail_open

    def read(self):
        self.reads += 1
        if self.fail_read_after is not None and self.reads > self.fail_read_after:
            return False, None
        if self.frames:
            return True, self.frames.pop(0)
        return True, np.full(self.shape, 7, dtype=np.uint8)

    def set(self, prop, value):
        self.seeks += 1
        if value == 0:
            self.frames = list(self._initial)
            self.reads = 0
        return True

    def get(self, prop):
        return 30.0

    def release(self):
        self.released = True


class StubTracker:
    """Tracker tiruan: selalu hasilkan lajur + debug stasiun."""

    def __init__(self, fail=False):
        self.fail = fail
        self.calls = 0
        self.last_debug = {"stations_found": 4, "stations_total": 4,
                           "polarity": "dark_on_light"}

    def estimate(self, frame, calib, timestamp):
        self.calls += 1
        if self.fail:
            raise RuntimeError("tracker meledak")
        return Observation(timestamp, lane_error_m=0.01,
                           heading_error_rad=0.02, curvature_inv_m=0.1,
                           confidence=0.9)


class StubHub:
    """FrameHub tiruan: cukup catat pemanggilan publish."""

    def __init__(self):
        self.published = 0

    def publish(self, frame):
        self.published += 1


def make_control(tmp_path, *, videos=None, tracker=None, hub=None, initial="",
                 opener=None, video_loop=True, fail_open=()):
    """BenchControl dengan opener yang memetakan pipeline -> FakeCapture."""
    opened = {}

    def factory(pipeline):
        cap = FakeCapture(
            pipeline,
            fail_open=pipeline in fail_open,
            fail_read_after=(2 if pipeline.endswith("habis.mp4") else None))
        opened[pipeline] = cap
        return cap

    config = Config(image_width=WIDTH, image_height=HEIGHT,
                    output_dir=str(tmp_path / "runs"))
    control = BenchControl(
        config,
        videos_dir=tmp_path / "videos",
        calib=CameraCalibration.from_config(config),
        tracker=tracker or StubTracker(),
        hub=hub or StubHub(),
        opener=opener or factory,
        initial=initial,
        video_loop=video_loop)
    return control, opened


def _wait_for(predicate, timeout=3.0):
    """Tunggu sampai predicate() True (loop kamera jalan di thread)."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return predicate()


@pytest.fixture()
def videos_dir(tmp_path):
    directory = tmp_path / "videos"
    directory.mkdir()
    for name in ("b-laps.mp4", "a-laps.mp4"):
        (directory / name).write_bytes(b"x" * 2048)
    (directory / "catatan.txt").write_text("bukan video", encoding="utf-8")
    (directory / "sub").mkdir()
    return directory


@pytest.fixture()
def gstreamer_ok(monkeypatch):
    """Pura-pura cv2 punya backend GStreamer (CSI bisa dibuka di tes)."""
    import mizuro.camera as camera_module
    monkeypatch.setattr(camera_module, "has_gstreamer", lambda cv2: True)


# --- daftar sumber ---------------------------------------------------------------

def test_sources_lists_videos_sorted_and_filtered(videos_dir, tmp_path):
    control, _ = make_control(tmp_path)
    sources = control.sources()
    assert [item["name"] for item in sources["videos"]] == [
        "a-laps.mp4", "b-laps.mp4"]
    assert sources["videos"][0]["size_bytes"] == 2048
    assert sources["videos_dir"] == str(videos_dir)
    assert sources["current"] == "none"
    assert sources["current_name"] == ""
    assert sources["video_loop"] is True


def test_sources_empty_dir(tmp_path):
    (tmp_path / "videos").mkdir()
    control, _ = make_control(tmp_path)
    assert control.sources()["videos"] == []


def test_sources_missing_dir(tmp_path):
    control, _ = make_control(tmp_path)
    sources = control.sources()
    assert sources["videos"] == []
    assert sources["videos_dir"].endswith("videos")


def test_sources_csi_availability_follows_gstreamer(gstreamer_ok, monkeypatch,
                                                    tmp_path):
    import mizuro.camera as camera_module
    (tmp_path / "videos").mkdir()
    control, _ = make_control(tmp_path)
    assert control.sources()["csi"]["available"] is True
    assert control.sources()["csi"]["reason"] == ""
    # sekarang matikan GStreamer
    monkeypatch.setattr(camera_module, "has_gstreamer", lambda cv2: False)
    csi = control.sources()["csi"]
    assert csi["available"] is False
    assert "/usr/bin/python3" in csi["reason"]


# --- select -----------------------------------------------------------------------

def test_select_video_switches_source(gstreamer_ok, videos_dir, tmp_path):
    control, opened = make_control(tmp_path, initial=GST_PIPE)
    with control:
        assert control.status()["source"] == "csi"
        result = control.select({"source": "video", "name": "a-laps.mp4"})
        assert result["source"] == "video"
        assert result["pipeline"].endswith("a-laps.mp4")
        assert result["is_file"] is True
        # 2 = sumber awal (CSI) + tukar eksplisit ke video
        assert result["switches"] == 2
        assert control.source.is_file is True
        # kamera CSI lama dilepas
        assert opened[GST_PIPE].released is True
        assert opened[str(videos_dir / "a-laps.mp4")].released is False


def test_select_csi_uses_default_pipeline(gstreamer_ok, videos_dir, tmp_path):
    control, opened = make_control(tmp_path)
    with control:
        control.select({"source": "csi"})
        assert control.status()["source"] == "csi"
        assert "nvarguscamerasrc" in control.source.pipeline
        assert f"width={WIDTH},height={HEIGHT}" in control.source.pipeline


def test_select_rejects_bad_names(videos_dir, tmp_path):
    control, _ = make_control(tmp_path)
    bad_names = [
        "", None, 5,
        "../rahasia.mp4", "sub/a.mp4", "/etc/passwd",
        "a-laps.txt", "a-laps.mp4.txt", ".",
        "x" * 200 + ".mp4",
    ]
    for name in bad_names:
        with pytest.raises(ValueError):
            control.select({"source": "video", "name": name})


def test_select_rejects_unknown_kind(videos_dir, tmp_path):
    control, _ = make_control(tmp_path)
    for payload in ({"source": "sihir"}, {}, {"source": None},
                    "csi", None, ["csi"]):
        with pytest.raises(ValueError):
            control.select(payload)


def test_select_rejects_symlink_escape(videos_dir, tmp_path):
    outside = tmp_path / "luar.mp4"
    outside.write_bytes(b"x")
    link = videos_dir / "link.mp4"
    try:
        link.symlink_to(outside)
    except OSError:
        pytest.skip("symlink tidak didukung filesystem ini")
    control, _ = make_control(tmp_path)
    with pytest.raises(ValueError, match="direktori daftar"):
        control.select({"source": "video", "name": "link.mp4"})


def test_failed_open_keeps_previous_source(gstreamer_ok, videos_dir, tmp_path):
    control, _ = make_control(
        tmp_path, initial=GST_PIPE,
        fail_open=(str(videos_dir / "a-laps.mp4"),))
    with control:
        before = control.status()
        with pytest.raises(RuntimeError, match="video gagal dibuka"):
            control.select({"source": "video", "name": "a-laps.mp4"})
        after = control.status()
        # sumber lama utuh: pipeline sama, tidak ada error status
        assert after["pipeline"] == before["pipeline"]
        assert after["source"] == "csi"
        assert after["error"] == ""
        assert control.source.pipeline == GST_PIPE


def test_select_resets_counters(gstreamer_ok, videos_dir, tmp_path):
    control, _ = make_control(tmp_path, initial=GST_PIPE)
    with control:
        assert _wait_for(lambda: control.status()["frames"] >= 3)
        control.select({"source": "video", "name": "a-laps.mp4"})
        status = control.status()
        assert status["frames"] == 0
        assert status["fps"] == 0.0
        # 2 = sumber awal (CSI) + tukar eksplisit ke video
        assert status["switches"] == 2
        assert _wait_for(lambda: control.status()["frames"] >= 3)


# --- status & loop ------------------------------------------------------------------

def test_status_shape_before_start(videos_dir, tmp_path):
    control, _ = make_control(tmp_path)
    status = control.status()
    for key in ("available", "ended", "error", "tracker_error", "frames",
                "fps", "lane_error_m", "heading_error_rad",
                "curvature_inv_m", "confidence", "stations", "polarity",
                "source", "pipeline", "is_file", "source_fps"):
        assert key in status, key
    assert status["available"] is False
    assert status["frames"] == 0
    assert status["lane_error_m"] is None


def test_loop_publishes_and_tracks(videos_dir, tmp_path):
    hub = StubHub()
    tracker = StubTracker()
    control, _ = make_control(tmp_path, tracker=tracker, hub=hub,
                              initial=str(videos_dir / "a-laps.mp4"))
    with control:
        assert _wait_for(lambda: control.status()["frames"] >= 5)
        status = control.status()
        assert status["available"] is True
        assert status["error"] == ""
        assert status["fps"] > 0
        assert status["lane_error_m"] == pytest.approx(0.01)
        assert status["confidence"] == pytest.approx(0.9)
        assert status["stations"] == "4/4"
        assert status["polarity"] == "dark_on_light"
        assert hub.published >= 5
        assert tracker.calls >= 5


def test_loop_keeps_running_when_tracker_fails(videos_dir, tmp_path):
    control, _ = make_control(tmp_path, tracker=StubTracker(fail=True),
                              initial=str(videos_dir / "a-laps.mp4"))
    with control:
        assert _wait_for(lambda: control.status()["frames"] >= 5)
        status = control.status()
        # kamera tetap hidup; error tracker dilaporkan terpisah
        assert status["available"] is True
        assert status["error"] == ""
        assert "tracker meledak" in status["tracker_error"]
        assert status["lane_error_m"] is None


def test_end_of_video_marks_ended_and_keeps_thread(videos_dir, tmp_path):
    (videos_dir / "habis.mp4").write_bytes(b"x")
    control, _ = make_control(tmp_path, video_loop=False,
                              initial=str(videos_dir / "habis.mp4"))
    with control:
        assert _wait_for(lambda: control.status()["ended"])
        status = control.status()
        assert status["ended"] is True
        assert status["available"] is False
        assert status["error"] == ""
        # kamera TIDAK dilepas: frame terakhir tetap ada di MJPEG buat
        # dilihat operator saat memilih video pengganti
        assert control.source.camera is not None
        # thread masih hidup -> operator bisa pilih video lain
        assert control._thread is not None and control._thread.is_alive()
        control.select({"source": "video", "name": "a-laps.mp4"})
        assert _wait_for(lambda: control.status()["frames"] >= 3)
        assert control.status()["ended"] is False


def test_video_loop_restarts_automatically(videos_dir, tmp_path):
    (videos_dir / "habis.mp4").write_bytes(b"x")
    control, opened = make_control(
        tmp_path, initial=str(videos_dir / "habis.mp4"), video_loop=True)
    with control:
        # fail_read_after=2 + loop: frame harus terus bertambah melewati
        # batas "video selesai" tanpa flag ended
        assert _wait_for(lambda: control.status()["frames"] >= 8)
        assert control.status()["ended"] is False
        assert control.status()["available"] is True
        cap = opened[str(videos_dir / "habis.mp4")]
        assert cap.seeks >= 1


def test_initial_failure_recorded_and_recoverable(monkeypatch, videos_dir,
                                                  tmp_path):
    import mizuro.camera as camera_module
    # CSI tanpa GStreamer: sumber awal gagal, thread TETAP jalan
    monkeypatch.setattr(camera_module, "has_gstreamer", lambda cv2: False)
    control, _ = make_control(tmp_path, initial="")
    with control:
        assert _wait_for(lambda: control.status()["error"])
        assert "GStreamer" in control.status()["error"]
        assert control.status()["available"] is False
        assert control._thread.is_alive()
        # operator memilih video dari dashboard -> pulih
        control.select({"source": "video", "name": "a-laps.mp4"})
        assert _wait_for(lambda: control.status()["frames"] >= 3)
        assert control.status()["error"] == ""


def test_camera_death_reported_not_fatal(videos_dir, tmp_path):
    control, _ = make_control(
        tmp_path, initial=str(videos_dir / "a-laps.mp4"),
        fail_open=(str(videos_dir / "a-laps.mp4"),))
    with control:
        # kamera gagal dibuka sejak awal: error tercatat, thread hidup
        assert _wait_for(lambda: control.status()["error"])
        assert control._thread.is_alive()


# --- siklus hidup ---------------------------------------------------------------------

def test_stop_joins_thread_and_releases_camera(gstreamer_ok, videos_dir,
                                               tmp_path):
    control, opened = make_control(tmp_path, initial=GST_PIPE)
    control.start()
    assert _wait_for(lambda: control.status()["frames"] >= 2)
    thread = control._thread
    control.stop()
    assert thread is not None and not thread.is_alive()
    assert opened[GST_PIPE].released is True
    assert control.source.camera is None


def test_context_manager_starts_and_stops(gstreamer_ok, videos_dir, tmp_path):
    control, opened = make_control(tmp_path, initial=GST_PIPE)
    with control:
        assert _wait_for(lambda: control.status()["frames"] >= 2)
    assert control._thread is None or not control._thread.is_alive()
    assert opened[GST_PIPE].released is True


def test_wait_returns_true_on_stop(gstreamer_ok, videos_dir, tmp_path):
    control, _ = make_control(tmp_path, initial=GST_PIPE)
    control.start()
    try:
        assert control.wait(0.05) is False
        threading.Thread(target=control.stop, daemon=True).start()
        assert control.wait(2.0) is True
    finally:
        control.stop()


def test_double_stop_is_safe(gstreamer_ok, videos_dir, tmp_path):
    control, _ = make_control(tmp_path, initial=GST_PIPE)
    control.start()
    control.stop()
    control.stop()      # tidak boleh melempar


# --- CameraSource (dipakai BenchControl) ----------------------------------------------

def test_camera_source_read_without_selection_raises():
    source = CameraSource(WIDTH, HEIGHT)
    with pytest.raises(RuntimeError, match="belum ada sumber"):
        source.read()


def test_camera_source_describe_none():
    source = CameraSource(WIDTH, HEIGHT)
    assert source.describe() == {"source": "none", "pipeline": "",
                                 "is_file": False, "source_fps": 0.0}


def test_camera_source_select_swaps_and_counts(monkeypatch):
    import mizuro.camera as camera_module
    monkeypatch.setattr(camera_module, "has_gstreamer", lambda cv2: True)
    opened = {}

    def factory(pipeline):
        cap = FakeCapture(pipeline)
        opened[pipeline] = cap
        return cap

    source = CameraSource(WIDTH, HEIGHT, opener=factory)
    source.select(GST_PIPE)
    assert source.switches == 1
    assert source.pipeline == GST_PIPE
    assert source.is_file is False
    assert source.source_fps == 30.0
    source.select("video.mp4")
    assert source.switches == 2
    assert source.is_file is True
    assert opened[GST_PIPE].released is True
    assert opened["video.mp4"].released is False
    source.close()
    assert opened["video.mp4"].released is True
    assert source.camera is None


def test_camera_source_failed_select_keeps_current(monkeypatch):
    import mizuro.camera as camera_module
    monkeypatch.setattr(camera_module, "has_gstreamer", lambda cv2: True)

    def factory(pipeline):
        return FakeCapture(pipeline, fail_open=pipeline == "rusak.mp4")

    source = CameraSource(WIDTH, HEIGHT, opener=factory)
    source.select(GST_PIPE)
    with pytest.raises(RuntimeError):
        source.select("rusak.mp4")
    assert source.pipeline == GST_PIPE
    assert source.switches == 1


def test_camera_source_read_delegates(monkeypatch):
    import mizuro.camera as camera_module
    monkeypatch.setattr(camera_module, "has_gstreamer", lambda cv2: True)
    source = CameraSource(WIDTH, HEIGHT,
                          opener=lambda p: FakeCapture(p))
    source.select(GST_PIPE)
    frame = source.read()
    assert frame.shape == (HEIGHT, WIDTH, 3)


def test_video_suffixes_are_lowercase():
    assert all(suffix == suffix.lower() for suffix in VIDEO_SUFFIXES)
    assert ".mp4" in VIDEO_SUFFIXES
