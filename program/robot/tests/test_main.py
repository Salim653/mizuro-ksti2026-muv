"""Tes main.py: validasi flag dashboard dan helper serve_dashboard.

Mode yang menyentuh kamera/serial TIDAK diuji di sini (butuh
hardware) — yang diuji jalur yang bisa gagal sebelum hardware:
validasi flag dan start/stop server dashboard.
"""
import sys
from urllib.request import urlopen

import pytest

import main
from mizuro.config import Config


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
