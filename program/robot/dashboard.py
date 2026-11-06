#!/usr/bin/env python3
"""Dashboard run MIZURO — pembaca log lokal (bench/pit saja, read-only).

Membaca `runs/run-*/telemetry.csv` + `events.jsonl` yang ditulis runtime
(`main.py --closed-loop`) dan menampilkannya di browser: kecepatan,
kemudi, error lajur, confidence, state machine, event runtime, dan
grafik deret waktu. Tanpa dependensi luar — stdlib Python + canvas di
browser, jadi jalan dengan `/usr/bin/python3` di Jetson apa adanya.

Aturan lomba: TIDAK ADA komunikasi selama percobaan. Dashboard ini alat
pit/bench — matikan sebelum setiap percobaan. Semua endpoint read-only;
tidak ada yang menulis, menggerakkan motor, atau menyentuh serial.

Pakai:
    /usr/bin/python3 dashboard.py --runs runs --port 18765
    /usr/bin/python3 dashboard.py --self-check

HTML terpisah (`dashboard.html`) supaya bisa diedit anggota tim tanpa
menyentuh Python. Endpoint JSON:

    /api/status    status run terbaru (atau ?run=run-YYYYmmdd-HHMMSS)
    /api/runs      daftar run yang ada di direktori log
    /api/history   deret waktu buat grafik (?run=&points=)
    /api/demo      data sintetis buat preview UI tanpa robot
"""
from __future__ import annotations

import argparse
import csv
import io
import json
import math
import os
import re
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, urlparse
from urllib.request import urlopen

from mizuro.telemetry import FIELDS

PAGE = Path(__file__).with_name("dashboard.html")
# Nama run divalidasi ketat: cegah path traversal lewat query ?run=..
RUN_NAME_RE = re.compile(r"run-[\w.-]+")
# Event yang boleh tampil. Detail event (bisa berisi config/secret)
# TIDAK pernah dikirim — hanya nama.
EVENT_NAMES = {
    "ready", "running", "dry_run", "stream", "lane_lost", "lane_recovered",
    "loop_overrun", "run_timeout", "detector_error", "keyboard_interrupt",
    "shutdown", "fault",
}
FRESH_S = 3.0          # log dianggap hidup kalau mtime < 3 detik
HISTORY_POINTS = 240   # default titik grafik (downsample)


def number(value: Any) -> float | None:
    """Parse float; None/NaN/inf -> None (JSON aman, JS render 'N/A')."""
    try:
        parsed = float(value) if value not in (None, "") else None
    except (TypeError, ValueError):
        return None
    return parsed if parsed is None or math.isfinite(parsed) else None


def tail_lines(path: Path, count: int, skip_first: bool = False) -> list[str]:
    """`count` baris terakhir file tanpa membaca seluruh file.

    File telemetry bisa puluhan ribu baris; baca mundur per blok 4 KB
    sampai cukup baris terkumpul.
    """
    with path.open("rb") as stream:
        stream.seek(0, 2)
        start = stream.tell()
        data = b""
        while start and data.count(b"\n") <= count:
            size = min(4096, start)
            start -= size
            stream.seek(start)
            data = stream.read(size) + data
    lines = data.decode("utf-8", errors="replace").splitlines()
    return (lines[1:] if skip_first and lines else lines)[-count:]


def _resolve_run(runs_dir: Path, run: str | None) -> Path | None:
    """Path telemetry.csv untuk `run`; None = run terbaru.

    Nama run dari query divalidasi regex dulu — `../` atau path absolut
    ditolak (ValueError -> HTTP 400), tidak bisa baca file di luar runs.
    """
    if run:
        if not RUN_NAME_RE.fullmatch(run):
            raise ValueError(f"nama run tidak valid: {run}")
        path = runs_dir / run / "telemetry.csv"
        return path if path.is_file() else None
    if not runs_dir.is_dir():
        return None
    files = list(runs_dir.glob("run-*/telemetry.csv"))
    return max(files, key=lambda path: path.parent.name) if files else None


def latest_status(runs_dir: Path, run: str | None = None) -> dict[str, Any]:
    """Baris telemetry terakhir + event runtime terakhir.

    Hanya field whitelist yang dikirim. `reason` state machine ikut
    (berguna buat diagnosa DEGRADED/FAULT); detail event tidak.
    """
    path = _resolve_run(runs_dir, run)
    if path is None:
        return {"available": False, "run": run or ""}

    header = path.open(encoding="utf-8").readline().rstrip()
    rows = list(csv.DictReader(
        io.StringIO("\n".join([header, *tail_lines(path, 2, skip_first=True)]))))
    last = next((row for row in reversed(rows) if row.get("t_monotonic")), None)
    if last is None:
        return {"available": False, "run": path.parent.name}

    age_s = max(0.0, time.time() - path.stat().st_mtime)
    values: dict[str, Any] = {
        "available": True, "run": path.parent.name,
        "age_s": round(age_s, 1), "fresh": age_s < FRESH_S,
    }
    values["state"] = last.get("state", "")
    values["reason"] = ("Runtime fault" if last.get("state") == "FAULT"
                        else last.get("reason", ""))
    values["command_reason"] = last.get("command_reason", "")
    for field in ("lane_error_m", "heading_error_rad", "curvature_inv_m",
                  "confidence", "speed_mps", "steering_rad"):
        values[field] = number(last.get(field))

    events: list[dict[str, str]] = []
    event_file = path.parent / "events.jsonl"
    if event_file.is_file():
        for line in tail_lines(event_file, 64):
            try:
                name = json.loads(line).get("event")
            except (json.JSONDecodeError, AttributeError):
                continue
            if isinstance(name, str) and name in EVENT_NAMES:
                events.append({"event": name})
    values["events"] = events[-12:]
    return values


def list_runs(runs_dir: Path) -> list[dict[str, Any]]:
    """Semua run di direktori log, terbaru dulu (nama = timestamp)."""
    if not runs_dir.is_dir():
        return []
    runs: list[dict[str, Any]] = []
    for path in runs_dir.glob("run-*/telemetry.csv"):
        try:
            with path.open("rb") as stream:
                rows = max(0, sum(1 for _ in stream) - 1)
            age_s = max(0.0, time.time() - path.stat().st_mtime)
        except OSError:
            continue
        runs.append({"run": path.parent.name, "rows": rows,
                     "age_s": round(age_s, 1), "fresh": age_s < FRESH_S})
    runs.sort(key=lambda item: item["run"], reverse=True)
    if runs:
        runs[0]["latest"] = True
    return runs


def run_history(runs_dir: Path, run: str | None = None,
                points: int = HISTORY_POINTS) -> dict[str, Any]:
    """Deret waktu buat grafik: t (relatif), speed, lane error (mm), conf.

    CSV dibaca penuh lalu di-downsample dengan stride — grafik 240 titik
    dari 18.000 baris tetap cukup detail dan hemat bandwidth.
    """
    points = max(20, min(600, int(points)))
    path = _resolve_run(runs_dir, run)
    if path is None:
        return {"available": False, "run": run or "", "t": [],
                "speed_mps": [], "lane_error_mm": [], "confidence": []}

    with path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.reader(stream)
        header = next(reader, [])
        index = {name: pos for pos, name in enumerate(header)}
        needed = ("t_monotonic", "speed_mps", "lane_error_m", "confidence")
        if not all(name in index for name in needed):
            return {"available": False, "run": path.parent.name, "t": [],
                    "speed_mps": [], "lane_error_mm": [], "confidence": []}
        rows = [row for row in reader if row and row[0]]

    if not rows:
        return {"available": False, "run": path.parent.name, "t": [],
                "speed_mps": [], "lane_error_mm": [], "confidence": []}

    # Indeks rata: selalu `points` titik (atau kurang), pertama & terakhir
    # selalu termasuk — grafik tidak melompat di ujung.
    count = min(points, len(rows))
    if count > 1:
        last = len(rows) - 1
        picked = sorted({round(i * last / (count - 1)) for i in range(count)})
    else:
        picked = [0]

    def cell(row: list[str], name: str) -> Any:
        pos = index[name]
        return row[pos] if pos < len(row) else ""

    t0 = number(cell(rows[picked[0]], "t_monotonic")) or 0.0
    series: dict[str, list] = {"t": [], "speed_mps": [],
                               "lane_error_mm": [], "confidence": []}
    for i in picked:
        row = rows[i]
        t = number(cell(row, "t_monotonic"))
        speed = number(cell(row, "speed_mps"))
        error = number(cell(row, "lane_error_m"))
        conf = number(cell(row, "confidence"))
        series["t"].append(None if t is None else round(t - t0, 2))
        series["speed_mps"].append(speed)
        series["lane_error_mm"].append(
            None if error is None else round(error * 1000.0, 1))
        series["confidence"].append(conf)
    return {"available": True, "run": path.parent.name,
            "rows": len(rows), **series}

def demo_status() -> dict[str, Any]:
    """Data sintetis (status + deret waktu) buat preview UI tanpa robot."""
    n = 120
    t: list[float] = []
    speed: list[float] = []
    error: list[float] = []
    conf: list[float] = []
    for i in range(n):
        x = i / (n - 1) * 24.0
        t.append(round(x, 2))
        speed.append(round(0.35 + 0.15 * math.sin(x / 3.0), 3))
        error.append(round(30.0 * math.sin(x / 2.0 + 1.0), 1))
        conf.append(round(0.75 + 0.20 * math.cos(x / 4.0), 2))
    return {
        "available": True, "demo": True, "run": "SIMULASI · DEMO ONLY",
        "state": "RUNNING", "reason": "data sintetis — bukan run asli",
        "command_reason": "demo lane tracking", "age_s": 0.2, "fresh": True,
        "lane_error_m": 0.031, "heading_error_rad": -0.028,
        "curvature_inv_m": 0.18, "confidence": 0.91,
        "speed_mps": 0.42, "steering_rad": 0.08,
        "events": [{"event": "ready"}, {"event": "running"},
                   {"event": "demo_data"}],
        "history": {"available": True, "run": "SIMULASI · DEMO ONLY",
                    "rows": n, "t": t, "speed_mps": speed,
                    "lane_error_mm": error, "confidence": conf},
    }


def _js_string(value: str) -> str:
    """Escape string buat diselipkan sebagai literal JS di halaman.

    URL stream berasal dari config/server sendiri, tapi halaman ini
    punya CSP longgar (`script-src 'unsafe-inline'`) — jadi escapé
    backslash, kutip, dan `<` (cegah `</script>` bocor) sekali lagi
    di sini. Defense in depth, bukan satu-satunya pagar.
    """
    return (value.replace("\\", "\\\\").replace('"', '\\"')
            .replace("'", "\\'").replace("<", "\\x3c")
            .replace("\n", "\\n").replace("\r", "\\r"))


def make_server(host: str, port: int, runs_dir: Path,
                feed_url: str = "") -> ThreadingHTTPServer:
    """Server dashboard. `feed_url` (opsional) di-inject ke halaman:

    kalau main.py dijalankan dengan `--dashboard`, URL MJPEG dari mode
    tersebut dikirim ke sini sehingga panel kamera langsung terisi —
    operator tidak perlu ketik manual. Kosong = placeholder dibiarkan
    kosong, operator isi sendiri.
    """
    page = PAGE.read_bytes()
    token = b"__FEED_URL__"
    if feed_url:
        page = page.replace(token, _js_string(feed_url).encode("utf-8"))
    else:
        page = page.replace(token, b"")

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:
            parsed = urlparse(self.path)
            query = parse_qs(parsed.query)
            try:
                if parsed.path in ("/", "/index.html"):
                    self._reply(200, "text/html; charset=utf-8", page)
                elif parsed.path == "/api/status":
                    run = query.get("run", [None])[0]
                    self._json(latest_status(runs_dir, run))
                elif parsed.path == "/api/runs":
                    self._json({"runs": list_runs(runs_dir)})
                elif parsed.path == "/api/history":
                    run = query.get("run", [None])[0]
                    points = query.get("points", [HISTORY_POINTS])[0]
                    self._json(run_history(runs_dir, run, points))
                elif parsed.path == "/api/demo":
                    self._json(demo_status())
                else:
                    self._reply(404, "text/plain; charset=utf-8",
                                b"tidak ditemukan")
            except ValueError as exc:
                # nama run dari query tidak lolos validasi
                self._json({"error": str(exc)}, status=400)
            except (OSError, csv.Error, UnicodeError) as exc:
                self._json({"error": f"tidak bisa membaca log: "
                                     f"{type(exc).__name__}"}, status=500)

        def _json(self, payload: dict[str, Any], status: int = 200) -> None:
            body = json.dumps(payload).encode("utf-8")
            self._reply(status, "application/json; charset=utf-8", body)

        def _reply(self, code: int, content_type: str, body: bytes) -> None:
            self.send_response(code)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("Content-Security-Policy",
                             "default-src 'none'; style-src 'unsafe-inline'; "
                             "script-src 'unsafe-inline'; img-src 'self' "
                             "http: https: data:; connect-src 'self'; "
                             "base-uri 'none'; frame-ancestors 'none'")
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, format: str, *args: Any) -> None:
            return

    server = ThreadingHTTPServer((host, port), Handler)
    server.daemon_threads = True
    return server


def self_check() -> None:
    """Verifikasi tanpa hardware: baca log sintetis + serve HTTP beneran.

    Stdlib only (tanpa pytest) — jalan dengan /usr/bin/python3 di Jetson.
    """
    with tempfile.TemporaryDirectory() as temp:
        root = Path(temp)

        # run lama (harus kalah dari run baru saat memilih "terbaru")
        old = root / "run-19990101-000000"
        old.mkdir()
        (old / "telemetry.csv").write_text("t_monotonic,state\n1,STOPPED\n",
                                           encoding="utf-8")
        os.utime(old / "telemetry.csv", (1, 1))

        # run dengan data lengkap + event berisi "secret"
        run = root / "run-20000101-000000"
        run.mkdir()
        with (run / "telemetry.csv").open("w", newline="",
                                          encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(FIELDS))
            writer.writeheader()
            writer.writerow({"t_monotonic": 12.5, "state": "RUNNING",
                             "reason": "jalur aman", "command_reason": "lane ok",
                             "lane_error_m": "0.012", "heading_error_rad": "-0.04",
                             "curvature_inv_m": "0.3", "confidence": "0.92",
                             "speed_mps": "0.4", "steering_rad": "-0.1"})
            writer.writerow({"t_monotonic": 12.6, "state": "RUNNING",
                             "reason": "jalur aman", "command_reason": "lane ok",
                             "lane_error_m": "-0.02", "heading_error_rad": "0.01",
                             "curvature_inv_m": "0.25", "confidence": "0.88",
                             "speed_mps": "0.45", "steering_rad": "0.12"})
        (run / "events.jsonl").write_text(
            '{"event":"ready","config":{"serial_device":"rahasia"}}\n'
            '{"event":"fault","error":"detail pribadi"}\n'
            '{"event":"bukan_whitelist","x":1}\n', encoding="utf-8")
        os.utime(run / "telemetry.csv", (1, 1))

        status = latest_status(root)
        assert status["state"] == "RUNNING" and status["speed_mps"] == 0.45
        assert status["lane_error_m"] == -0.02 and status["fresh"] is False
        assert status["events"] == [{"event": "ready"}, {"event": "fault"}]
        assert status["reason"] == "jalur aman"
        dumped = json.dumps(status)
        assert "rahasia" not in dumped and "detail pribadi" not in dumped
        assert "bukan_whitelist" not in dumped
        assert latest_status(root)["run"] == "run-20000101-000000"

        # run spesifik lewat query + nama tidak valid ditolak
        picked = latest_status(root, "run-19990101-000000")
        assert picked["run"] == "run-19990101-000000"
        assert picked["available"] is True and picked["state"] == "STOPPED"
        try:
            latest_status(root, "../../etc")
        except ValueError:
            pass
        else:
            raise AssertionError("nama run traversal diterima")
        assert latest_status(root, "run-tidak-ada")["available"] is False

        # run hidup: RunLog asli dari paket mizuro -> harus terpilih + fresh
        from mizuro.states import State, StateMachine
        from mizuro.telemetry import RunLog
        from mizuro.types import Command

        with RunLog(str(root)) as log:
            machine = StateMachine()
            machine.transition(State.READY, "checks")
            machine.transition(State.RUNNING, "self-check")
            log.event("ready", loop_hz=60.0)
            for i in range(5):
                log.row(machine, None, Command(0.3 + i * 0.01, 0.02))
            live = latest_status(root)
            assert live["run"] == log.path.name and live["fresh"] is True
            assert live["state"] == "RUNNING"

        runs = list_runs(root)
        assert [item["run"] for item in runs] == sorted(
            [item["run"] for item in runs], reverse=True)
        assert runs[0]["latest"] is True and runs[0]["rows"] == 5
        assert list_runs(root / "tidak-ada") == []

        history = run_history(root, None, 3)
        assert history["available"] is True and history["rows"] == 5
        assert len(history["t"]) == len(history["speed_mps"]) == 5
        assert history["t"][0] == 0.0
        assert history["speed_mps"][-1] > 0.3
        empty = run_history(root, "run-19990101-000000")
        assert empty["available"] is False and empty["t"] == []

        server = make_server("127.0.0.1", 0, root,
                             feed_url="http://127.0.0.1:8080/stream")
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            with urlopen(f"http://127.0.0.1:{server.server_port}/") as response:
                page = response.read()
                assert response.status == 200
                assert response.headers.get_content_type() == "text/html"
                assert b"Dashboard Run" in page and b"aria-live" in page
                assert b"Tidak ada log run" in page
                assert b"Mode simulasi" in page and b"feedUrl" in page
                # URL stream dari --dashboard harus ter-inject, token bersih
                assert b"http://127.0.0.1:8080/stream" in page
                assert b"__FEED_URL__" not in page
            with urlopen(f"http://127.0.0.1:{server.server_port}/api/status"
                         ) as response:
                served = json.load(response)
                assert served["run"] == log.path.name
            with urlopen(f"http://127.0.0.1:{server.server_port}/api/runs"
                         ) as response:
                assert len(json.load(response)["runs"]) == 3
            with urlopen(f"http://127.0.0.1:{server.server_port}"
                         "/api/history?points=2") as response:
                served = json.load(response)
                # points=2 naik ke clamp minimum 20; run hidup punya 5
                # baris -> 5 titik (semua baris), bukan 2.
                assert served["available"] is True
                assert len(served["t"]) == len(served["speed_mps"]) == 5
            with urlopen(f"http://127.0.0.1:{server.server_port}/api/demo"
                         ) as response:
                demo = json.load(response)
                assert demo["demo"] is True and demo["speed_mps"] == 0.42
                assert len(demo["history"]["t"]) == 120
            try:
                urlopen(f"http://127.0.0.1:{server.server_port}"
                        "/api/status?run=../etc")
            except Exception as exc:
                assert getattr(exc, "code", None) == 400
            else:
                raise AssertionError("query run traversal dilayani")
            try:
                urlopen(f"http://127.0.0.1:{server.server_port}/tidak-ada")
            except Exception as exc:
                assert getattr(exc, "code", None) == 404
            else:
                raise AssertionError("path tidak dikenal dilayani")
        finally:
            server.shutdown()
            server.server_close()
            thread.join()
    print("dashboard self-check passed")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Dashboard run MIZURO (read-only, bench/pit saja)")
    parser.add_argument("--runs", type=Path, default=Path("runs"),
                        help="direktori log runtime (default: ./runs "
                             "relatif terhadap cwd, sama seperti main.py)")
    parser.add_argument("--host", default="127.0.0.1",
                        help="alamat bind; 0.0.0.0 hanya untuk jaringan "
                             "pit tepercaya (tidak ada autentikasi)")
    parser.add_argument("--port", type=int, default=18765)
    parser.add_argument("--self-check", action="store_true",
                        help="verifikasi tanpa hardware (stdlib only)")
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        raise SystemExit(f"port harus dalam [1, 65535]: {args.port}")
    if args.self_check:
        self_check()
        return
    server = make_server(args.host, args.port, args.runs)
    print(f"Dashboard: http://{args.host}:{server.server_port}/ "
          f"(log: {args.runs})")
    print("Bench/pit saja — matikan sebelum percobaan lomba. Ctrl+C stop.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
