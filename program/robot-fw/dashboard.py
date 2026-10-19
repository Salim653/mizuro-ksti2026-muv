from __future__ import annotations

import argparse
import csv
import io
import json
import math
import os
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from core import Command, State, StateMachine
from telemetry import RunLog
from typing import Any
from urllib.request import urlopen


PAGE = Path(__file__).with_name("dashboard.html")
FIELDS = (
    "state", "command_reason", "lane_error_m", "heading_error_rad",
    "curvature_inv_m", "confidence", "speed_mps", "steering_rad",
)
EVENT_NAMES = {"ready", "dry_run", "fault", "shutdown", "loop_overrun", "frame_save_failed"}


def number(value: str | None) -> float | None:
    try:
        parsed = float(value) if value not in (None, "") else None
        return parsed if parsed is None or math.isfinite(parsed) else None
    except ValueError:
        return None


def tail_lines(path: Path, count: int, skip_first: bool = False) -> list[str]:
    with path.open("rb") as stream:
        stream.seek(0, 2)
        end = stream.tell()
        start = end
        data = b""
        while start and data.count(b"\n") <= count:
            size = min(4096, start)
            start -= size
            stream.seek(start)
            data = stream.read(size) + data
    lines = data.decode("utf-8", errors="replace").splitlines()
    return (lines[1:] if skip_first and lines else lines)[-count:]


def latest_status(runs_dir: Path) -> dict[str, Any]:
    files = [path for path in runs_dir.glob("run-*/telemetry.csv")
             if not (path.parent / ".dashboard-test").exists()] if runs_dir.is_dir() else []
    if not files:
        return {"available": False}

    telemetry = max(files, key=lambda path: path.parent.name)
    with telemetry.open(encoding="utf-8") as stream:
        header = stream.readline().rstrip()
    rows = list(csv.DictReader(io.StringIO("\n".join([header, *tail_lines(telemetry, 2, skip_first=True)]))))
    last = next((row for row in reversed(rows) if row.get("t_monotonic")), None)
    if last is None:
        return {"available": False, "run": telemetry.parent.name}

    age_s = max(0.0, time.time() - telemetry.stat().st_mtime)
    values: dict[str, Any] = {"available": True, "run": telemetry.parent.name,
                              "age_s": age_s, "fresh": age_s < 3.0}
    values.update({field: last.get(field, "") for field in FIELDS if field in ("state", "command_reason")})
    values["reason"] = "Runtime fault" if last.get("state") == "FAULT" else last.get("reason", "")
    values.update({field: number(last.get(field)) for field in FIELDS if field not in ("state", "command_reason")})

    events: list[dict[str, str]] = []
    event_file = telemetry.parent / "events.jsonl"
    if event_file.is_file():
        for line in tail_lines(event_file, 64):
            try:
                name = json.loads(line).get("event")
            except (json.JSONDecodeError, AttributeError):
                continue
            if isinstance(name, str) and name in EVENT_NAMES:
                events.append({"event": name})
        values["events"] = events[-8:]
    else:
        values["events"] = []
    return values


def make_server(host: str, port: int, runs_dir: Path) -> ThreadingHTTPServer:
    page = PAGE.read_bytes()

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:
            if self.path in ("/", "/index.html"):
                body, content_type = page, "text/html; charset=utf-8"
                status = 200
            elif self.path == "/api/status":
                try:
                    body = json.dumps(latest_status(runs_dir)).encode("utf-8")
                    content_type, status = "application/json; charset=utf-8", 200
                except (OSError, csv.Error, UnicodeError) as exc:
                    body = json.dumps({"error": f"Could not read run logs: {type(exc).__name__}"}).encode("utf-8")
                    content_type, status = "application/json; charset=utf-8", 500
            elif self.path == "/api/demo":
                body = json.dumps({"available": True, "demo": True, "run": "SIMULATED · DEMO ONLY",
                                   "state": "RUNNING", "reason": "Synthetic dashboard preview",
                                   "command_reason": "simulated lane tracking", "speed_mps": 0.42,
                                   "steering_rad": 0.08, "lane_error_m": 0.06,
                                   "heading_error_rad": -0.03, "curvature_inv_m": 0.18,
                                   "confidence": 0.91, "age_s": 0.2, "fresh": True,
                                   "events": [{"event": "demo_data"}, {"event": "simulated_lane_tracking"}]}).encode("utf-8")
                content_type, status = "application/json; charset=utf-8", 200
            else:
                body, content_type, status = b"Not found", "text/plain; charset=utf-8", 404
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("Content-Security-Policy", "default-src 'none'; style-src 'unsafe-inline'; script-src 'unsafe-inline'; img-src 'self' http: https:; connect-src 'self'; base-uri 'none'; frame-ancestors 'none'")
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, format: str, *args: Any) -> None:
            return

    return ThreadingHTTPServer((host, port), Handler)


def self_check() -> None:
    with tempfile.TemporaryDirectory() as temp:
        root = Path(temp)
        run = root / "run-20000101-000000"
        run.mkdir()
        telemetry_path = run / "telemetry.csv"
        with telemetry_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=("t_monotonic", *FIELDS, "reason"))
            writer.writeheader()
            writer.writerow({"t_monotonic": 12.5, "state": "RUNNING", "command_reason": "lane ok",
                             "lane_error_m": "0.12", "heading_error_rad": "-0.04",
                             "curvature_inv_m": "0.3", "confidence": "0.92",
                             "speed_mps": "0.4", "steering_rad": "-0.1",
                             "reason": "must not be exposed"})
        (run / "events.jsonl").write_text(
            '{"event":"ready","config":{"serial_device":"secret"}}\n'
            '{"event":"fault","error":"private detail"}\n', encoding="utf-8")
        os.utime(telemetry_path, (1, 1))
        status = latest_status(root)
        assert status["state"] == "RUNNING" and status["speed_mps"] == 0.4
        assert status["lane_error_m"] == 0.12 and status["events"] == [
            {"event": "ready"}, {"event": "fault"}]
        assert status["reason"] == "must not be exposed"
        assert "config" not in json.dumps(status) and "private detail" not in json.dumps(status)
        old_log = root / "run-19990101-000000"
        old_log.mkdir()
        (old_log / "telemetry.csv").write_text("t_monotonic,state\n1,STOPPED\n", encoding="utf-8")
        os.utime(old_log / "telemetry.csv", (1, 1))
        assert latest_status(root)["run"] == "run-20000101-000000"
        active = RunLog(str(root))
        try:
            machine = StateMachine()
            machine.transition(State.READY, "startup checks passed")
            machine.transition(State.RUNNING, "temporary self-check")
            active.row(machine, None, Command())
            live = latest_status(root)
            assert live["run"] == active.path.name and live["state"] == "RUNNING" and live["fresh"]
        finally:
            active.close()
        assert latest_status(root / "empty")["available"] is False

        server = make_server("127.0.0.1", 0, root)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            with urlopen(f"http://127.0.0.1:{server.server_port}/api/status") as response:
                served = json.load(response)
                assert response.status == 200 and served["run"] == active.path.name
            with urlopen(f"http://127.0.0.1:{server.server_port}/") as response:
                page = response.read()
                assert response.headers.get_content_type() == "text/html"
                assert b"Field readout" in page and b"aria-live" in page
                assert b"No run log found" in page and b"cannot read the run log" in page
                assert b"SIMULATED DATA" in page and b"feedUrl" in page
            with urlopen(f"http://127.0.0.1:{server.server_port}/api/demo") as response:
                demo = json.load(response)
                assert response.status == 200 and demo["demo"] is True
                assert demo["run"] == "SIMULATED \u00b7 DEMO ONLY" and demo["speed_mps"] == 0.42
            try:
                urlopen(f"http://127.0.0.1:{server.server_port}/unknown")
            except Exception as exc:
                assert getattr(exc, "code", None) == 404
            else:
                raise AssertionError("unknown dashboard path was served")
        finally:
            server.shutdown()
            server.server_close()
            thread.join()
    print("dashboard self-check passed")


def main() -> None:
    parser = argparse.ArgumentParser(description="Read-only dashboard for robot run logs")
    parser.add_argument("--runs", type=Path, default=Path(__file__).with_name("runs"),
                        help="runtime output directory (default: robot-fw/runs)")
    parser.add_argument("--host", default="127.0.0.1",
                        help="bind address; use 0.0.0.0 only on a trusted bench network")
    parser.add_argument("--port", type=int, default=18765)
    parser.add_argument("--self-check", action="store_true")
    args = parser.parse_args()
    if args.self_check:
        self_check()
        return
    server = make_server(args.host, args.port, args.runs)
    print(f"Dashboard: http://{args.host}:{server.server_port}/ (logs: {args.runs})")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
