"""Tes main.py: validasi flag dashboard dan helper serve_dashboard.

Mode yang menyentuh kamera/serial TIDAK diuji di sini (butuh
hardware) — yang diuji jalur yang bisa gagal sebelum hardware:
validasi flag dan start/stop server dashboard.
"""
import socket
import sys
import threading
import time
from urllib.request import Request, urlopen

import pytest

import main
from mizuro.config import Config


def _free_port() -> int:
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def test_dashboard_requires_a_mode(monkeypatch):
    """--dashboard sendirian = tidak ada log run yang ditulis."""
    monkeypatch.setattr(sys, "argv", ["main.py", "--dashboard"])
    with pytest.raises(SystemExit):
        main.main()


def test_dashboard_port_out_of_range(monkeypatch):
    monkeypatch.setattr(sys, "argv",
                        ["main.py", "--dashboard", "--stream-only",
                         "--dashboard-port", "70000"])
    with pytest.raises(SystemExit):
        main.main()


def test_dashboard_port_zero_rejected(monkeypatch):
    monkeypatch.setattr(sys, "argv",
                        ["main.py", "--dashboard", "--closed-loop",
                         "--dashboard-port", "0"])
    with pytest.raises(SystemExit):
        main.main()


def test_serve_dashboard_starts_serves_and_stops(tmp_path, capsys):
    """Server jalan di thread daemon, halaman ter-inject URL stream,
    dan bisa di-shutdown bersih (tidak ada thread nyangkut)."""
    config = Config(output_dir=str(tmp_path))
    server = main.serve_dashboard(
        config, 0, feed_url="http://127.0.0.1:8080/stream")
    try:
        assert server.server_port > 0
        with urlopen(f"http://127.0.0.1:{server.server_port}/") as response:
            page = response.read()
        assert b"http://127.0.0.1:8080/stream" in page
        assert b"__FEED_URL__" not in page
        out = capsys.readouterr().out
        assert "dashboard :" in out and "PERINGATAN" in out
    finally:
        server.shutdown()
        server.server_close()


def test_serve_dashboard_reads_live_run(tmp_path):
    """Dashboard yang distart main.py melihat run yang sedang ditulis:
    RunLog baru -> langsung jadi run terbaru di /api/status."""
    from mizuro.states import State, StateMachine
    from mizuro.telemetry import RunLog
    from mizuro.types import Command

    config = Config(output_dir=str(tmp_path))
    server = main.serve_dashboard(config, 0)
    try:
        with RunLog(str(tmp_path)) as log:
            machine = StateMachine()
            machine.transition(State.READY, "checks")
            machine.transition(State.RUNNING, "tes dashboard")
            log.row(machine, None, Command(0.25, 0.05))
            with urlopen(f"http://127.0.0.1:{server.server_port}"
                         "/api/status") as response:
                import json
                status = json.load(response)
            assert status["run"] == log.path.name
            assert status["state"] == "RUNNING"
            assert status["speed_mps"] == 0.25
            assert status["fresh"] is True
    finally:
        server.shutdown()
        server.server_close()


# --- mode --bench -------------------------------------------------------------------

class _FakeCapture:
    """cv2.VideoCapture tiruan buat run_bench (tanpa hardware)."""

    def __init__(self, pipeline):
        self.pipeline = pipeline
        self.released = False
        self.reads = 0

    def isOpened(self):
        return True

    def read(self):
        import numpy as np
        self.reads += 1
        return True, np.zeros((48, 64, 3), dtype=np.uint8)

    def set(self, prop, value):
        return True

    def get(self, prop):
        return 30.0

    def release(self):
        self.released = True


def _bench_config(tmp_path, videos, port):
    return Config(image_width=64, image_height=48,
                  output_dir=str(tmp_path / "runs"),
                  stream_host="127.0.0.1", stream_port=port,
                  camera_pipeline=str(videos / "a.mp4"))


def test_bench_flag_conflicts_with_dashboard(monkeypatch):
    """--bench sudah menyertakan dashboard; --dashboard redundan."""
    monkeypatch.setattr(sys, "argv",
                        ["main.py", "--bench", "--dashboard"])
    with pytest.raises(SystemExit):
        main.main()


def test_bench_duration_must_be_positive(monkeypatch):
    monkeypatch.setattr(sys, "argv",
                        ["main.py", "--bench", "--bench-duration", "0"])
    with pytest.raises(SystemExit):
        main.main()


def test_bench_video_must_exist(monkeypatch, tmp_path):
    monkeypatch.setattr(sys, "argv",
                        ["main.py", "--bench", "--video",
                         str(tmp_path / "tidak-ada.mp4")])
    with pytest.raises(SystemExit):
        main.main()


def test_run_bench_serves_control_endpoints(tmp_path):
    """run_bench: MJPEG + dashboard dengan kontrol sumber kamera hidup,
    tukar sumber lewat POST bekerja, dan semuanya berhenti bersih."""
    import json

    videos = tmp_path / "videos"
    videos.mkdir()
    (videos / "a.mp4").write_bytes(b"x")
    (videos / "b.mp4").write_bytes(b"x")
    stream_port, dash_port = _free_port(), _free_port()
    config = _bench_config(tmp_path, videos, stream_port)
    opened = {}

    def opener(pipeline):
        cap = _FakeCapture(pipeline)
        opened[pipeline] = cap
        return cap

    base = f"http://127.0.0.1:{dash_port}"
    thread = threading.Thread(
        target=main.run_bench,
        args=(config, dash_port, videos),
        kwargs={"video": str(videos / "a.mp4"), "duration_s": 3.0,
                "opener": opener},
        daemon=True)
    thread.start()
    try:
        # tunggu server siap (bench start butuh beberapa ms)
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            try:
                with urlopen(f"{base}/api/sources", timeout=1.0) as response:
                    sources = json.load(response)
                break
            except Exception:
                time.sleep(0.05)
        else:
            raise AssertionError("dashboard bench tidak pernah siap")

        assert sources["current"] == "video"
        assert [item["name"] for item in sources["videos"]] == [
            "a.mp4", "b.mp4"]

        # kamera mengalir: frame bertambah di status bench
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            with urlopen(f"{base}/api/status", timeout=1.0) as response:
                status = json.load(response)
            if status["bench"]["frames"] > 0:
                break
            time.sleep(0.05)
        assert status["bench"]["frames"] > 0
        assert status["bench"]["available"] is True

        # tukar sumber lewat POST — proses TIDAK restart
        request = Request(f"{base}/api/source",
                          data=b'{"source": "video", "name": "b.mp4"}',
                          headers={"Content-Type": "application/json"},
                          method="POST")
        with urlopen(request, timeout=5.0) as response:
            switched = json.load(response)
        assert switched["source"] == "video"
        assert switched["pipeline"].endswith("b.mp4")
        assert opened[str(videos / "a.mp4")].released is True

        # nama ngawur ditolak 400, sumber lama tetap aktif
        request = Request(f"{base}/api/source",
                          data=b'{"source": "video", "name": "../x.mp4"}',
                          headers={"Content-Type": "application/json"},
                          method="POST")
        with pytest.raises(Exception) as excinfo:
            urlopen(request, timeout=5.0)
        assert getattr(excinfo.value, "code", None) == 400
        with urlopen(f"{base}/api/status", timeout=1.0) as response:
            assert json.load(response)["bench"]["pipeline"].endswith("b.mp4")
    finally:
        thread.join(timeout=15.0)
    assert not thread.is_alive()
    # semua kamera dilepas saat bench berhenti
    assert all(cap.released for cap in opened.values())


def test_run_bench_forces_dry_run(tmp_path):
    """Config live (dry_run=false) tidak boleh bocor ke mode bench:
    run_bench memaksa dry_run dan tidak pernah membuka serial."""
    import dataclasses

    videos = tmp_path / "videos"
    videos.mkdir()
    (videos / "a.mp4").write_bytes(b"x")
    config = dataclasses.replace(
        _bench_config(tmp_path, videos, _free_port()), dry_run=False)
    seen = {}
    original = main.serve_dashboard

    def spy(cfg, port, feed_url="", control=None):
        seen["dry_run"] = cfg.dry_run
        seen["control"] = control
        return original(cfg, port, feed_url, control)

    main.serve_dashboard = spy
    try:
        # durasi pendek: bench berhenti sendiri, tidak ada Ctrl+C tiruan
        main.run_bench(config, 0, videos, video=str(videos / "a.mp4"),
                       duration_s=0.05,
                       opener=lambda pipeline: _FakeCapture(pipeline))
    finally:
        main.serve_dashboard = original
    assert seen["dry_run"] is True
    assert seen["control"] is not None
