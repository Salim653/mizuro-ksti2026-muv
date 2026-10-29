"""Tes stream: FrameHub (encode sekali, throttle, snapshot) + server HTTP.

Server diuji di port ephemeral (port=0) di localhost; tidak ada jaringan
eksternal dan tidak ada kamera.
"""
import threading
import time
import urllib.request

import numpy as np
import pytest

from mizuro.stream import FrameHub, MjpegServer


def _frame(seed=0):
    rng = np.random.default_rng(seed)
    return rng.integers(0, 255, (48, 64, 3), dtype=np.uint8)


# --- FrameHub ---

def test_publish_skipped_without_clients():
    hub = FrameHub(fps=60.0)
    hub.publish(_frame())
    assert hub.encoded_frames == 0
    assert hub.latest() is None


def test_publish_encodes_with_client():
    hub = FrameHub(fps=60.0)
    hub.attach()
    hub.publish(_frame())
    assert hub.encoded_frames == 1
    jpeg = hub.latest()
    assert jpeg is not None and jpeg[:2] == b"\xff\xd8"   # magic JPEG


def test_publish_throttled_to_fps():
    hub = FrameHub(fps=5.0)
    hub.attach()
    for i in range(10):
        hub.publish(_frame(i))
    assert hub.encoded_frames == 1     # 10 frame dalam < 1/5 s -> 1 encode


def test_snapshot_arms_pending_and_gets_frame():
    hub = FrameHub(fps=60.0)
    hub.publish(_frame())              # tanpa client: tidak di-encode
    assert hub.latest() is None

    result = {}

    def grab():
        result["jpeg"] = hub.snapshot(timeout=2.0)

    t = threading.Thread(target=grab)
    t.start()
    time.sleep(0.05)
    hub.publish(_frame(1))             # sekarang ada _pending -> encode
    t.join(timeout=3.0)
    assert result["jpeg"] is not None
    assert hub.encoded_frames == 1


def test_snapshot_timeout_returns_none():
    hub = FrameHub(fps=60.0)
    assert hub.snapshot(timeout=0.1) is None


def test_publish_survives_garbage_frame():
    hub = FrameHub(fps=60.0)
    hub.attach()
    hub.publish(None)                  # imencode melempar -> ditangkap
    assert hub.encoded_frames == 0
    # _pending tidak nyangkut: snapshot masih bisa meminta frame.
    hub.publish(_frame())
    assert hub.encoded_frames == 1


def test_resize_applied_before_encode():
    hub = FrameHub(fps=60.0, width=32, height=16)
    hub.attach()
    hub.publish(_frame())
    jpeg = hub.latest()
    assert jpeg is not None
    import cv2
    img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    assert img.shape == (16, 32, 3)


def test_clients_count_exact_across_attach_detach():
    hub = FrameHub()
    assert hub.clients == 0
    hub.attach(); hub.attach()
    assert hub.clients == 2
    hub.detach()
    assert hub.clients == 1
    hub.detach(); hub.detach()         # detach berlebih tidak jadi negatif
    assert hub.clients == 0


def test_invalid_construction_rejected():
    with pytest.raises(ValueError):
        FrameHub(quality=0)
    with pytest.raises(ValueError):
        FrameHub(fps=0.0)


# --- MjpegServer ---

@pytest.fixture
def server():
    hub = FrameHub(fps=60.0, width=64, height=48)
    srv = MjpegServer(hub, "127.0.0.1", 0).start()
    yield srv, hub
    srv.stop()


def _get(url, timeout=5.0):
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return r.status, r.headers.get("Content-Type"), r.read()


def test_index_page(server):
    srv, _ = server
    status, ctype, body = _get(srv.url)
    assert status == 200
    assert "text/html" in ctype
    assert b"/stream" in body


def test_snapshot_before_any_frame_is_503(server):
    srv, _ = server
    try:
        _get(srv.url + "snapshot")
        raise AssertionError("seharusnya 503")
    except urllib.error.HTTPError as e:
        assert e.code == 503


def test_snapshot_after_publish(server):
    srv, hub = server
    hub.attach()                       # client nyata (server /stream nanti)
    hub.publish(_frame(2))
    status, ctype, body = _get(srv.url + "snapshot")
    assert status == 200
    assert ctype == "image/jpeg"
    assert body[:2] == b"\xff\xd8"


def test_unknown_path_404(server):
    srv, _ = server
    try:
        _get(srv.url + "tidak-ada")
        raise AssertionError("seharusnya 404")
    except urllib.error.HTTPError as e:
        assert e.code == 404


def test_stream_serves_multipart_and_detaches(server):
    srv, hub = server

    def publisher():
        for i in range(5):
            hub.publish(_frame(i))
            time.sleep(0.02)

    threading.Thread(target=publisher, daemon=True).start()
    with urllib.request.urlopen(srv.url + "stream", timeout=5.0) as r:
        assert r.status == 200
        assert "multipart/x-mixed-replace" in r.headers["Content-Type"]
        time.sleep(0.2)
        assert hub.clients == 1
        chunk = r.read(4096)           # baca sebagian multipart
        assert b"--frame" in chunk
    # Setelah koneksi ditutup, client harus lepas (tidak bocor). Handler
    # baru sadar saat publish berikutnya membuat write() gagal — sama
    # seperti di robot nyata (loop kamera terus jalan).
    for i in range(5, 10):
        hub.publish(_frame(i))
        time.sleep(0.02)
    deadline = time.monotonic() + 3.0
    while hub.clients > 0 and time.monotonic() < deadline:
        time.sleep(0.05)
    assert hub.clients == 0
