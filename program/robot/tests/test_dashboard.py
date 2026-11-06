"""Tes dashboard: pembacaan log, endpoint HTTP, dan keamanan query.

Dashboard read-only — tes memastikan tidak ada kebocoran detail event
(config/secret), nama run dari query divalidasi (path traversal
ditolak), dan deret waktu grafik ter-downsample dengan benar.
"""
import csv
import json
import math
import threading
from urllib.request import urlopen

import pytest

import dashboard
from mizuro.states import State, StateMachine
from mizuro.telemetry import FIELDS, RunLog
from mizuro.types import Command


def _write_run(root, name, rows, events=None, mtime=None):
    """Buat direktori run sintetis; return path telemetry.csv."""
    run = root / name
    run.mkdir(parents=True)
    path = run / "telemetry.csv"
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(FIELDS))
        writer.writeheader()
        for row in rows:
            writer.writerow(row)
    if events is not None:
        (run / "events.jsonl").write_text(
            "".join(json.dumps(event) + "\n" for event in events),
            encoding="utf-8")
    if mtime is not None:
        import os
        os.utime(path, (mtime, mtime))
    return path


def _row(t, **kw):
    base = {"t_monotonic": t, "state": "RUNNING", "reason": "jalur aman",
            "command_reason": "lane ok", "lane_error_m": 0.01,
            "heading_error_rad": 0.0, "curvature_inv_m": 0.2,
            "confidence": 0.9, "speed_mps": 0.3, "steering_rad": 0.0}
    base.update(kw)
    return base


# --- number ---------------------------------------------------------------------

def test_number_parses_and_rejects_junk():
    assert dashboard.number("0.5") == 0.5
    assert dashboard.number("") is None
    assert dashboard.number(None) is None
    assert dashboard.number("nan") is None
    assert dashboard.number("inf") is None
    assert dashboard.number("bukan-angka") is None


# --- latest_status ----------------------------------------------------------------

def test_latest_status_reads_last_row(tmp_path):
    _write_run(tmp_path, "run-20260101-120000",
               [_row(1.0, speed_mps=0.3), _row(2.0, speed_mps=0.42,
                                               lane_error_m=-0.02)],
               events=[{"event": "ready", "config": {"serial_device": "rahasia"}},
                       {"event": "fault", "error": "detail pribadi"},
                       {"event": "tidak_whitelist"}])
    status = dashboard.latest_status(tmp_path)
    assert status["available"] is True
    assert status["run"] == "run-20260101-120000"
    assert status["state"] == "RUNNING"
    assert status["speed_mps"] == 0.42
    assert status["lane_error_m"] == -0.02
    assert status["reason"] == "jalur aman"
    assert status["command_reason"] == "lane ok"
    # hanya nama event yang lolos; detail TIDAK pernah ikut
    assert status["events"] == [{"event": "ready"}, {"event": "fault"}]
    dumped = json.dumps(status)
    assert "rahasia" not in dumped and "detail pribadi" not in dumped
    assert "tidak_whitelist" not in dumped


def test_latest_status_freshness(tmp_path):
    import os
    path = _write_run(tmp_path, "run-20260101-120000", [_row(1.0)])
    os.utime(path, (1, 1))   # 1970 -> pasti basi
    assert dashboard.latest_status(tmp_path)["fresh"] is False
    # RunLog asli: mtime = sekarang -> fresh
    with RunLog(str(tmp_path)) as log:
        machine = StateMachine()
        machine.transition(State.READY, "checks")
        machine.transition(State.RUNNING, "tes")
        log.row(machine, None, Command(0.3, 0.0))
        live = dashboard.latest_status(tmp_path)
        assert live["run"] == log.path.name
        assert live["fresh"] is True
        assert live["state"] == "RUNNING"


def test_latest_status_picks_newest_run(tmp_path):
    _write_run(tmp_path, "run-20260101-120000", [_row(1.0, speed_mps=0.1)])
    _write_run(tmp_path, "run-20260102-120000", [_row(1.0, speed_mps=0.2)])
    assert dashboard.latest_status(tmp_path)["speed_mps"] == 0.2


def test_latest_status_explicit_run(tmp_path):
    _write_run(tmp_path, "run-20260101-120000", [_row(1.0, speed_mps=0.1)])
    _write_run(tmp_path, "run-20260102-120000", [_row(1.0, speed_mps=0.2)])
    status = dashboard.latest_status(tmp_path, "run-20260101-120000")
    assert status["run"] == "run-20260101-120000"
    assert status["speed_mps"] == 0.1


def test_latest_status_rejects_traversal(tmp_path):
    _write_run(tmp_path, "run-20260101-120000", [_row(1.0)])
    for bad in ("../../etc", "run-../..", "/etc/passwd", "run-a/b"):
        with pytest.raises(ValueError):
            dashboard.latest_status(tmp_path, bad)


def test_latest_status_missing_run_or_dir(tmp_path):
    assert dashboard.latest_status(tmp_path / "kosong")["available"] is False
    _write_run(tmp_path, "run-20260101-120000", [_row(1.0)])
    missing = dashboard.latest_status(tmp_path, "run-tidak-ada")
    assert missing["available"] is False
    assert missing["run"] == "run-tidak-ada"


def test_latest_status_header_only_run(tmp_path):
    _write_run(tmp_path, "run-20260101-120000", [])
    status = dashboard.latest_status(tmp_path)
    assert status["available"] is False
    assert status["run"] == "run-20260101-120000"


def test_latest_status_fault_reason(tmp_path):
    _write_run(tmp_path, "run-20260101-120000",
               [_row(1.0, state="FAULT", reason="")])
    assert dashboard.latest_status(tmp_path)["reason"] == "Runtime fault"


# --- list_runs ---------------------------------------------------------------------

def test_list_runs_sorted_with_latest_flag(tmp_path):
    _write_run(tmp_path, "run-20260101-120000", [_row(1.0)])
    _write_run(tmp_path, "run-20260103-120000", [_row(1.0), _row(2.0)])
    _write_run(tmp_path, "run-20260102-120000", [_row(1.0)])
    runs = dashboard.list_runs(tmp_path)
    assert [item["run"] for item in runs] == [
        "run-20260103-120000", "run-20260102-120000", "run-20260101-120000"]
    assert runs[0]["latest"] is True
    assert runs[0]["rows"] == 2
    assert "latest" not in runs[1]


def test_list_runs_missing_dir(tmp_path):
    assert dashboard.list_runs(tmp_path / "kosong") == []


# --- run_history -------------------------------------------------------------------

def test_run_history_downsamples(tmp_path):
    rows = [_row(float(i), speed_mps=0.1 + i * 0.01) for i in range(100)]
    _write_run(tmp_path, "run-20260101-120000", rows)
    history = dashboard.run_history(tmp_path, None, 20)
    assert history["available"] is True
    assert history["rows"] == 100
    assert len(history["t"]) == 20
    assert len(history["speed_mps"]) == 20
    assert history["t"][0] == 0.0            # relatif terhadap titik pertama
    assert history["t"][-1] == pytest.approx(99.0)  # titik terakhir = baris terakhir
    assert history["speed_mps"][-1] == pytest.approx(0.1 + 99 * 0.01)
    assert history["lane_error_mm"][-1] == pytest.approx(10.0, abs=0.05)
    assert all(math.isfinite(value) for value in history["t"])


def test_run_history_points_clamped(tmp_path):
    rows = [_row(float(i)) for i in range(5)]
    _write_run(tmp_path, "run-20260101-120000", rows)
    assert len(dashboard.run_history(tmp_path, None, 1)["t"]) == 5
    assert len(dashboard.run_history(tmp_path, None, 9999)["t"]) == 5


def test_run_history_empty_run(tmp_path):
    _write_run(tmp_path, "run-20260101-120000", [])
    history = dashboard.run_history(tmp_path)
    assert history["available"] is False
    assert history["t"] == []


def test_run_history_rejects_traversal(tmp_path):
    _write_run(tmp_path, "run-20260101-120000", [_row(1.0)])
    with pytest.raises(ValueError):
        dashboard.run_history(tmp_path, "../../etc")


# --- demo ---------------------------------------------------------------------------

def test_demo_status_shape():
    demo = dashboard.demo_status()
    assert demo["demo"] is True and demo["available"] is True
    assert demo["state"] == "RUNNING"
    assert demo["speed_mps"] == 0.42
    history = demo["history"]
    assert history["available"] is True
    assert len(history["t"]) == len(history["speed_mps"]) == 120
    assert all(0 <= value <= 1 for value in history["confidence"])


# --- server HTTP ----------------------------------------------------------------------

@pytest.fixture()
def server(tmp_path):
    _write_run(tmp_path, "run-20260101-120000",
               [_row(1.0), _row(2.0, speed_mps=0.5)],
               events=[{"event": "ready", "config": {"serial_device": "rahasia"}}])
    httpd = dashboard.make_server("127.0.0.1", 0, tmp_path)
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    try:
        yield f"http://127.0.0.1:{httpd.server_port}"
    finally:
        httpd.shutdown()
        httpd.server_close()
        thread.join()


def test_server_serves_page(server):
    with urlopen(f"{server}/") as response:
        page = response.read()
        assert response.status == 200
        assert response.headers.get_content_type() == "text/html"
        text = page.decode("utf-8")
        assert "Dashboard Run" in text and "aria-live" in text
        assert "Tidak ada log run" in text and "Mode simulasi" in text
        assert 'id="feedUrl"' in text and 'id="chart"' in text
        assert "read-only" in text


def test_server_status_and_runs(server):
    with urlopen(f"{server}/api/status") as response:
        status = json.load(response)
        assert status["available"] is True
        assert status["speed_mps"] == 0.5
        assert status["events"] == [{"event": "ready"}]
    with urlopen(f"{server}/api/runs") as response:
        runs = json.load(response)["runs"]
        assert runs[0]["run"] == "run-20260101-120000"
        assert runs[0]["rows"] == 2


def test_server_history_and_demo(server):
    with urlopen(f"{server}/api/history?points=1") as response:
        history = json.load(response)
        assert history["available"] is True
        assert len(history["t"]) == len(history["speed_mps"]) == 2
    with urlopen(f"{server}/api/demo") as response:
        demo = json.load(response)
        assert demo["demo"] is True
        assert len(demo["history"]["t"]) == 120


def test_server_rejects_bad_run_query(server):
    with pytest.raises(Exception) as excinfo:
        urlopen(f"{server}/api/status?run=../etc")
    assert getattr(excinfo.value, "code", None) == 400


def test_server_404_unknown_path(server):
    with pytest.raises(Exception) as excinfo:
        urlopen(f"{server}/tidak-ada")
    assert getattr(excinfo.value, "code", None) == 404


def test_server_security_headers(server):
    with urlopen(f"{server}/api/status") as response:
        headers = response.headers
        assert headers.get("Cache-Control") == "no-store"
        assert headers.get("X-Content-Type-Options") == "nosniff"
        assert "default-src 'none'" in headers.get("Content-Security-Policy", "")


# --- self-check -----------------------------------------------------------------------

def test_self_check_passes():
    dashboard.self_check()


def test_page_file_exists_next_to_module():
    assert dashboard.PAGE.is_file()
    assert dashboard.PAGE.name == "dashboard.html"


# --- injeksi URL stream (--dashboard) ----------------------------------------------------

def test_js_string_escapes_for_inline_script():
    """Payload yang bisa bocor dari literal JS harus di-escape:
    kutip (tutup string), backslash, dan `<` (cegah </script>)."""
    escaped = dashboard._js_string('a"b\\c<d>\'e')
    assert escaped == 'a\\"b\\\\c\\x3cd>\\\'e'
    assert "<" not in escaped
    assert '"' not in escaped.replace('\\"', "")


def test_make_server_injects_feed_url(tmp_path):
    _write_run(tmp_path, "run-20260101-120000", [_row(1.0)])
    payload = 'http://h:8080/s" onload="alert(1)'
    server = dashboard.make_server("127.0.0.1", 0, tmp_path,
                                   feed_url=payload)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        with urlopen(f"http://127.0.0.1:{server.server_port}/") as response:
            page = response.read()
        assert b"__FEED_URL__" not in page
        assert b"http://h:8080/s" in page           # URL asli ada
        assert b'\\x3c' in page or b"alert" in page  # tapi ter-escape
        assert b'" onload="alert(1)' not in page    # payload mentah TIDAK ada
    finally:
        server.shutdown()
        server.server_close()
        thread.join()


def test_make_server_without_feed_url_clears_token(tmp_path):
    _write_run(tmp_path, "run-20260101-120000", [_row(1.0)])
    server = dashboard.make_server("127.0.0.1", 0, tmp_path)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        with urlopen(f"http://127.0.0.1:{server.server_port}/") as response:
            page = response.read()
        assert b"__FEED_URL__" not in page
        assert b'const DEFAULT_FEED = "";' in page
    finally:
        server.shutdown()
        server.server_close()
        thread.join()
