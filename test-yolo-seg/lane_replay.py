#!/usr/bin/env python3
"""Ideal-lap video replay -> lane segmentation -> ESP32 motors -> web.

The video is a recorded ideal lap from the arena-3d sim (POV, 640x480).
For every frame the lane model is called on the local inference server, the
returned outer/inner/centre polylines are unprojected and fitted, and a
pure-pursuit steering command is mapped to differential motor speeds:

    M <left> <right>\\n          (same protocol as test_botol_seg.py)

The page on port 8092 shows the replay with the segmentation overlay, the
fitted lane, the lateral/heading terms and the motor values.  Motors are gated
OFF in the page until you press "Enable motors"; lift the wheels first.

Run on the Jetson:
    cd ~/mizuro-ksti2026-muv/test-yolo-seg
    ../.venv/bin/python lane_replay.py

Needs the lane inference server running in another terminal:
    cd ~/mizuro-ksti2026-muv/program/sim-viz
    KSTI_BACKEND=cuda ../../.venv/bin/python server_viz.py
"""
from __future__ import annotations

import argparse
import glob
import json
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.request import Request, urlopen

import cv2
import numpy as np

try:
    import serial
except ImportError:                      # test the page without pyserial
    serial = None

# ---- lane geometry of the recorded POV (sim mount) ----
W, H = 640, 480
FOV_DEG = 62.0
PITCH = math.radians(12.0)
CAM_H = 0.16
CAM_FWD = 0.10
FX = (W / 2.0) / math.tan(math.radians(FOV_DEG) / 2.0)
FY = FX
CX = (W - 1) / 2.0
CY = (H - 1) / 2.0

# ---- controller (same numbers as the sim model driver) ----
XREF = 0.45
LOOKAHEAD = 0.575
WHEELBASE = 0.25
LANE_W = 0.50
V_MAX = 1.60
A_LAT = 6.0
KH = 0.5
MAX_STEER = math.radians(31.0)
V_MIN = 0.45

# ---- playback / motors ----
PLAY_FPS = 15.0
JPEG_Q = 82
BASE_SPEED = 200
BASE_SPAN = 0
TURN_GAIN = 90
PORT = 8092
PREDICT_URL = "http://127.0.0.1:8766/predict"
SERIAL_PORT = "/dev/ttyUSB0"
BAUD = 115200

COLORS = {"outer": (60, 220, 90), "inner": (255, 150, 40), "centre": (60, 160, 255)}
INK = (238, 234, 231)
MUTED = (160, 168, 176)
ACCENT = (75, 176, 232)      # BGR amber
BAD = (110, 110, 250)


class Controller:
    def __init__(self) -> None:
        self.outer_side = 0
        self.steer = 0.0

    @staticmethod
    def _fit(uv):
        xs, ys = [], []
        cp, sp = math.cos(PITCH), math.sin(PITCH)
        for u, v in uv:
            b = (v - CY) / FY
            down = sp + b * cp
            if down <= 1e-3:
                continue
            lam = CAM_H / down
            xs.append(lam * (cp - b * sp) + CAM_FWD)
            ys.append(-lam * (u - CX) / FX)
        return np.asarray(xs, dtype=np.float64), np.asarray(ys, dtype=np.float64)

    @staticmethod
    def _ev(c, xs, x):
        x = float(np.clip(x, xs.min(), xs.max()))
        return float(np.polyval(c, x))

    def update(self, out):
        cls = {}
        rows = 0
        for name in ("outer", "inner", "centre"):
            xs, ys = self._fit(out.get(name) or [])
            rows += len(xs)
            if len(xs) < 4:
                continue
            deg = 2 if len(xs) >= 12 else 1
            try:
                c = np.polyfit(xs, ys, deg)
            except Exception:
                continue
            x_far = min(XREF + 0.30, max(XREF + 0.05, float(xs.max())))
            cls[name] = {
                "y0": self._ev(c, xs, XREF),
                "y1": self._ev(c, xs, x_far),
                "xF": x_far,
                "n": len(xs),
                "c": c,
                "curv": float(2.0 * c[0]) if deg == 2 else 0.0,
            }
        O, I, C = cls.get("outer"), cls.get("inner"), cls.get("centre")
        est = None
        if O and I and 0.75 * LANE_W < abs(O["y0"] - I["y0"]) < 1.35 * LANE_W:
            self.outer_side = 1 if O["y0"] > I["y0"] else -1
            xF = min(O["xF"], I["xF"])
            c0 = 0.5 * (O["y0"] + I["y0"])
            c1 = 0.5 * (O["y1"] + I["y1"])
            slope = (c1 - c0) / max(0.05, xF - XREF)
            est = {"src": "pair", "c0": c0, "c1": c1, "xF": xF,
                   "curv": 0.5 * (O["curv"] + I["curv"]),
                   "c": np.array([slope, c0 - slope * XREF])}
        elif (O or I) and self.outer_side:
            E = O if (O and (not I or O["n"] >= I["n"])) else I
            toward = (-self.outer_side if E is O else self.outer_side) * 0.5 * LANE_W
            est = {"src": "outer" if E is O else "inner",
                   "c0": E["y0"] + toward, "c1": E["y1"] + toward,
                   "xF": E["xF"], "curv": E["curv"], "c": E["c"]}
        elif C and C["n"] >= 8:
            est = {"src": "centre", "c0": C["y0"], "c1": C["y1"],
                   "xF": C["xF"], "curv": C["curv"], "c": C["c"]}

        if est is None:
            self.steer *= 0.6
            return {"valid": False, "src": "none", "rows": rows, "lat": 0.0,
                    "head": 0.0, "curv": 0.0, "target": 0.0,
                    "steer": 0.0, "xF": 0.0, "fit": None}
        lat = est["c0"]
        denom = max(0.15, est["xF"] - XREF)
        head = math.atan2(est["c1"] - est["c0"], denom)
        curv_r = 1.0 / abs(est["curv"]) if abs(est["curv"]) > 1e-3 else 50.0
        target = min(V_MAX, math.sqrt(A_LAT * max(0.15, min(curv_r, 50.0))))
        target *= 1.0 / (1.0 + 2.5 * abs(lat) + 1.5 * abs(head))
        target = max(V_MIN, target)
        err = math.atan(2.0 * WHEELBASE * lat / (LOOKAHEAD ** 2 + lat * lat)) + KH * head
        err = max(-MAX_STEER, min(MAX_STEER, err))
        self.steer = 0.65 * self.steer + 0.35 * err
        return {"valid": True, "src": est["src"], "rows": rows, "lat": lat,
                "head": head, "curv": est["curv"], "target": target,
                "steer": self.steer, "xF": est["xF"], "fit": est.get("c")}


def motor_values(state, ok):
    if not (ok and state["valid"]) or state["target"] <= 0.0:
        return 0, 0
    base = BASE_SPEED + int(round(BASE_SPAN * min(1.0, state["target"] / V_MAX)))
    turn = int(round(TURN_GAIN * min(1.0, abs(state["steer"]) / MAX_STEER)))
    if abs(state["steer"]) < math.radians(1.5):
        turn = 0
    if state["steer"] > 0:                       # + = turn left
        left, right = base - turn, base + turn
    else:
        left, right = base + turn, base - turn
    clamp = lambda v: max(0, min(255, int(v)))
    return clamp(left), clamp(right)


class Motors:
    def __init__(self, port: str, enabled: bool = True) -> None:
        self.ser = None
        self.ok = False
        self.port = port
        self.error = ""
        if not enabled:
            self.error = "disabled with --no-motor"
            return
        if serial is None:
            self.error = "pyserial not installed"
            return
        try:
            self.ser = serial.Serial(port, BAUD, timeout=0)
            time.sleep(2)                         # ESP32 resets when the port opens
            self.ok = True
            self.send(0, 0)
        except Exception as exc:
            self.error = f"{type(exc).__name__}: {exc}"
            self.ser = None

    def send(self, left: int, right: int) -> None:
        if self.ser is None:
            return
        try:
            self.ser.write(f"M {left} {right}\n".encode())
            self.ser.reset_input_buffer()
        except Exception as exc:
            self.ok = False
            self.error = f"write: {exc}"
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    def close(self) -> None:
        if self.ser is not None:
            self.send(0, 0)
            try:
                self.ser.close()
            except Exception:
                pass


class Hub:
    def __init__(self) -> None:
        self._cond = threading.Condition()
        self._jpeg = None
        self._seq = 0
        self._status = {}
        self.clients = 0

    def publish(self, jpeg: bytes, status: dict) -> None:
        with self._cond:
            self._jpeg = jpeg
            self._seq += 1
            self._status = status
            self._cond.notify_all()

    def set_status(self, status: dict) -> None:
        with self._cond:
            self._status = status
            self._cond.notify_all()

    @property
    def status(self) -> dict:
        with self._cond:
            return dict(self._status)

    def attach(self) -> None:
        with self._cond:
            self.clients += 1
            self._cond.notify_all()

    def detach(self) -> None:
        with self._cond:
            self.clients = max(0, self.clients - 1)
            self._cond.notify_all()

    def wait(self, seq: int, timeout: float = 5.0):
        with self._cond:
            self._cond.wait_for(lambda: self._seq != seq or self.clients == 0, timeout)
            return self._jpeg, self._seq


class Runtime:
    def __init__(self) -> None:
        self.paused = False
        self.motor_on = False
        self.restart = False


class _Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.0"
    hub: Hub
    runtime: Runtime
    page: bytes
    motors: Motors

    def log_message(self, *args) -> None:
        pass

    def _reply(self, code: int, ctype: str, body: bytes) -> None:
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:
        path, _, query = self.path.partition("?")
        q = dict(p.split("=", 1) for p in query.split("&") if "=" in p)
        if path in ("/", "/index.html"):
            self._reply(200, "text/html; charset=utf-8", self.page)
        elif path == "/stream":
            self._stream()
        elif path == "/status":
            st = dict(self.hub.status)
            st.update({"clients": self.hub.clients,
                       "paused": self.runtime.paused,
                       "motor_on": self.runtime.motor_on,
                       "serial_ok": self.motors.ok,
                       "serial_error": self.motors.error})
            self._reply(200, "application/json", json.dumps(st).encode())
        elif path == "/snapshot":
            with self.hub._cond:
                jpeg = self.hub._jpeg
            if jpeg is None:
                self._reply(503, "text/plain", b"no frame yet")
            else:
                self._reply(200, "image/jpeg", jpeg)
        elif path == "/api/motor":
            on = q.get("on") == "1"
            if on and not self.motors.ok:
                self._reply(409, "application/json", json.dumps({"error": self.motors.error}).encode())
                return
            self.runtime.motor_on = on
            if not on:
                self.motors.send(0, 0)
            self._reply(200, "application/json", json.dumps({"motor_on": on}).encode())
        elif path == "/api/pause":
            on = q.get("on") == "1"
            self.runtime.paused = on
            if on:
                self.motors.send(0, 0)
            self._reply(200, "application/json", json.dumps({"paused": on}).encode())
        elif path == "/api/restart":
            self.runtime.restart = True
            self._reply(200, "application/json", b'{"restart":true}')
        else:
            self._reply(404, "text/plain", b"not found")

    def _stream(self) -> None:
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.hub.attach()
        seq = -1
        try:
            while self.hub.clients > 0:
                jpeg, new_seq = self.hub.wait(seq)
                if new_seq == seq or jpeg is None:
                    continue
                seq = new_seq
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                 + str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError):
            pass
        finally:
            self.hub.detach()


PAGE = """<!doctype html>
<html lang="en">
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>KSTI lane replay · ESP32</title>
<style>
  :root{--bg:#0e1012;--panel:#16191c;--line:#252a2f;--ink:#e7eaee;--mute:#8b939c;
    --accent:#e8b04b;--bad:#f87171;--ok:#4ade80}
  *{box-sizing:border-box}
  body{margin:0;background:var(--bg);color:var(--ink);font:14px/1.5 system-ui,"Segoe UI",sans-serif;padding:20px}
  header{display:flex;align-items:baseline;gap:12px;margin-bottom:16px;flex-wrap:wrap}
  h1{font-size:16px;font-weight:600;margin:0;letter-spacing:.01em}
  .sub{color:var(--mute);font-size:12px;font-variant-numeric:tabular-nums}
  .wrap{display:grid;grid-template-columns:minmax(0,1fr) 310px;gap:16px;max-width:1250px}
  @media (max-width:900px){.wrap{grid-template-columns:1fr}}
  .video{background:#000;border:1px solid var(--line);border-radius:8px;overflow:hidden;line-height:0}
  .video img{width:100%;height:auto;display:block}
  .legend{display:flex;gap:14px;flex-wrap:wrap;color:var(--mute);font-size:11px;margin-top:8px}
  .lg{display:inline-block;width:9px;height:9px;border-radius:2px;margin-right:5px;vertical-align:middle}
  .panel{background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:14px 16px;align-self:start}
  .row{display:flex;justify-content:space-between;align-items:baseline;gap:12px;padding:7px 0;border-bottom:1px solid var(--line)}
  .row:last-child{border-bottom:0}
  .k{color:var(--mute);font-size:11px;text-transform:uppercase;letter-spacing:.06em}
  .v{font-variant-numeric:tabular-nums;font-weight:600}
  .mode{display:inline-flex;align-items:center;gap:7px;font-size:11px;font-weight:700;
    letter-spacing:.06em;text-transform:uppercase;color:var(--mute)}
  .dot{width:8px;height:8px;border-radius:50%;background:var(--mute)}
  .mode.run{color:var(--ok)} .mode.run .dot{background:var(--ok)}
  .mode.warn{color:var(--accent)} .mode.warn .dot{background:var(--accent)}
  .mode.bad{color:var(--bad)} .mode.bad .dot{background:var(--bad)}
  .bar{position:relative;height:8px;background:#0a0c0e;border:1px solid var(--line);border-radius:999px;margin-top:8px;overflow:hidden}
  .bar i{position:absolute;top:0;bottom:0;left:50%;width:2px;margin-left:-1px;background:var(--mute);opacity:.6}
  .bar b{position:absolute;top:0;bottom:0;background:var(--accent);border-radius:999px;transition:left .12s,width .12s}
  .motors{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-top:12px}
  .motor{background:#0a0c0e;border:1px solid var(--line);border-radius:6px;padding:9px 10px}
  .motor .k{display:block;margin-bottom:2px}
  .motor .n{font-size:22px;font-weight:600;font-variant-numeric:tabular-nums}
  .controls{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:13px}
  button{font:600 12px system-ui,"Segoe UI",sans-serif;color:var(--ink);background:#1d222a;
    border:1px solid var(--line);border-radius:6px;padding:9px 10px;cursor:pointer}
  button:hover{border-color:#3c4553}
  button.on{background:var(--accent);color:#14100a;border-color:var(--accent)}
  button.danger{background:#3a1114;border-color:#7f2028;color:#ffd9d9}
  button:disabled{opacity:.45;cursor:default}
  #bmotor{grid-column:1/-1;padding:12px;font-size:13px}
  .warn{font-size:11px;color:var(--mute);line-height:1.55;margin:12px 0 0}
  .warn b{color:var(--accent);font-weight:600}
  footer{max-width:1250px;color:var(--mute);font-size:11.5px;margin-top:14px;line-height:1.6}
  kbd{font:10px monospace;background:#22272f;border:1px solid var(--line);border-radius:3px;padding:0 4px}
</style>
<body>
<header>
  <h1>KSTI lane replay</h1>
  <span class="sub" id="meta">connecting…</span>
</header>
<div class="wrap">
  <div>
    <div class="video"><img src="/stream" alt="replay inference"></div>
    <div class="legend">
      <span><span class="lg" style="background:#5adc5a"></span>outer edge</span>
      <span><span class="lg" style="background:#2896ff"></span>inner edge</span>
      <span><span class="lg" style="background:#ffa03c"></span>centre line</span>
      <span><span class="lg" style="background:#46e6ff"></span>inferred centre</span>
    </div>
  </div>
  <aside class="panel">
    <div class="row"><span class="k">State</span><span class="mode" id="mode"><span class="dot"></span><span id="modetxt">connecting</span></span></div>
    <div class="row"><span class="k">Frame</span><span class="v" id="frame">—</span></div>
    <div class="row"><span class="k">Inference</span><span class="v" id="infer">—</span></div>
    <div class="row"><span class="k">Lane source</span><span class="v" id="src">—</span></div>
    <div class="row"><span class="k">Fit rows</span><span class="v" id="rows">—</span></div>
    <div class="row" style="display:block">
      <span class="k">Lateral offset</span>
      <div class="bar"><i></i><b id="offbar" style="left:50%;width:0"></b></div>
      <div class="sub" id="offtxt" style="margin-top:6px">—</div>
    </div>
    <div class="row"><span class="k">Heading error</span><span class="v" id="head">—</span></div>
    <div class="row"><span class="k">Steering</span><span class="v" id="steer">—</span></div>
    <div class="motors">
      <div class="motor"><span class="k">Left cmd</span><span class="n" id="lm">0</span></div>
      <div class="motor"><span class="k">Right cmd</span><span class="n" id="rm">0</span></div>
    </div>
    <div class="row" style="margin-top:10px"><span class="k">ESP32 serial</span><span class="v" id="serial">—</span></div>
    <div class="controls">
      <button id="bmotor" class="danger">Enable motors</button>
      <button id="bpause">Pause</button>
      <button id="brestart">Restart loop</button>
    </div>
    <p class="warn"><b>Motors follow the replay automatically (test-yolo style).</b> Lift the wheels before opening this page. The lane model runs on every frame and sends <kbd>M left right</kbd> to the ESP32; <b>Stop motors</b> gates them, and zero is sent while no page is connected.</p>
  </aside>
</div>
<footer>Recorded POV lap · 640×480 · YOLO11n-seg @480 · pure pursuit on the fitted lane · commands sent every frame.</footer>
<script>
const $ = id => document.getElementById(id);
let motorOn = false, paused = false, last = null;
function setMode(cls, text) { $('mode').className = 'mode ' + cls; $('modetxt').textContent = text; }
async function tick() {
  try {
    const s = await (await fetch('/status', {cache:'no-store'})).json();
    last = s;
    const offline = !s.serial_ok;
    const err = s.error;
    if (err) setMode('bad', 'error');
    else if (paused) setMode('warn', 'paused');
    else if (s.motor_on) setMode('run', 'driving');
    else setMode('run', 'inference');
    motorOn = !!s.motor_on; paused = !!s.paused;
    $('meta').textContent = (s.fps ? s.fps.toFixed(1)+' fps' : '—') + ' · loop ' + (s.loop ?? 0)
      + ' · frame ' + (s.frame ?? '—') + '/' + (s.total ?? '—');
    $('frame').textContent = (s.frame ?? '—') + ' / ' + (s.total ?? '—');
    $('infer').textContent = s.infer_ms != null ? s.infer_ms.toFixed(0) + ' ms' : '—';
    $('src').textContent = (s.src || '—') + (s.valid ? '' : ' (no fit)');
    $('rows').textContent = s.rows ?? '—';
    const off = s.lat_cm ?? 0, w = Math.min(50, Math.abs(off) * 2.2);
    $('offbar').style.left = (off > 0 ? 50 - w : 50) + '%';
    $('offbar').style.width = w + '%';
    $('offtxt').textContent = s.lat_cm != null ? (off >= 0 ? 'centre left of car' : 'centre right of car')
      + ' · ' + Math.abs(off).toFixed(1) + ' cm' : '—';
    $('head').textContent = s.head_deg != null ? (s.head_deg > 0 ? '+' : '') + s.head_deg.toFixed(1) + '°' : '—';
    $('steer').textContent = s.steer_deg != null ? (s.steer_deg > 0 ? '+' : '') + s.steer_deg.toFixed(1) + '°' : '—';
    $('lm').textContent = s.left ?? 0; $('rm').textContent = s.right ?? 0;
    $('serial').textContent = s.serial_ok ? (s.motor_on ? 'ok · driving' : 'ok · gated') : 'offline';
    $('serial').style.color = s.serial_ok ? 'var(--ok)' : 'var(--bad)';
    $('bmotor').textContent = motorOn ? 'Stop motors' : 'Enable motors';
    $('bmotor').className = motorOn ? 'on' : 'danger';
    $('bmotor').disabled = !s.serial_ok;
    $('bpause').textContent = paused ? 'Resume' : 'Pause';
    $('bpause').className = paused ? 'on' : '';
  } catch (e) {
    setMode('bad', 'offline');
    $('meta').textContent = 'server offline';
  }
}
setInterval(tick, 200); tick();
$('bmotor').onclick = async () => {
  const go = !motorOn;
  const r = await fetch('/api/motor?on=' + (go ? 1 : 0));
  if (!r.ok) { const j = await r.json().catch(()=>({})); alert(j.error || 'serial unavailable'); }
  tick();
};
$('bpause').onclick = async () => { await fetch('/api/pause?on=' + (paused ? 0 : 1)); tick(); };
$('brestart').onclick = async () => { await fetch('/api/restart'); tick(); };
</script>
</html>
"""


def lane_center_polyline(raw):
    """Average the outer/inner polylines in image space: the visual lane centre."""
    curves = {}
    for name in ("outer", "inner"):
        pts = np.asarray(raw.get(name) or [], dtype=np.float64)
        if len(pts) < 4:
            return None
        pts = pts[np.argsort(pts[:, 1])]
        vs, us = [], []
        for y in np.unique(np.round(pts[:, 1]).astype(int)):
            sel = np.abs(pts[:, 1] - y) <= 1.0
            if sel.any():
                vs.append(y); us.append(float(pts[sel, 0].mean()))
        curves[name] = (np.asarray(vs, np.float64), np.asarray(us, np.float64))
    vo, uo = curves["outer"]; vi, ui = curves["inner"]
    lo, hi = max(vo.min(), vi.min()), min(vo.max(), vi.max())
    if hi <= lo + 3:
        return None
    vs = np.linspace(lo, hi, 36)
    us = 0.5 * (np.interp(vs, vo, uo) + np.interp(vs, vi, ui))
    return np.stack([us, vs], axis=1)


def draw(frame, state, left, right, infer_ms, idx, total, loop, motor_on, status_note, gated=True):
    vis = frame.copy()
    for name in ("outer", "inner", "centre"):
        pts = state.get("raw", {}).get(name) or []
        if len(pts) < 2:
            continue
        arr = np.asarray(pts, dtype=np.int32).reshape(-1, 1, 2)
        cv2.polylines(vis, [arr], False, COLORS[name], 2, cv2.LINE_AA)
    # visual lane centre = average of the outer/inner masks in image space.
    centre = lane_center_polyline(state.get("raw") or {})
    if centre is not None and len(centre) >= 2:
        cv2.polylines(vis, [np.asarray(centre, np.int32).reshape(-1, 1, 2)],
                      False, (255, 230, 70), 2, cv2.LINE_AA)
    elif state.get("fit") is not None:
        c = state["fit"]
        cp, sp = math.cos(PITCH), math.sin(PITCH)
        xs = np.linspace(XREF, max(XREF + 0.05, state["xF"]), 24)
        pts = []
        for x in xs:
            y = float(np.polyval(c, x))
            lam = max(0.05, (x - CAM_FWD) / max(1e-3, cp))
            b = (CAM_H / lam - sp) / cp
            v = CY + b * FY
            u = CX - y * FX / lam
            if -20 <= u <= W + 20 and -20 <= v <= H + 20:
                pts.append((int(u), int(v)))
        if len(pts) >= 2:
            cv2.polylines(vis, [np.asarray(pts, np.int32).reshape(-1, 1, 2)],
                          False, (255, 230, 70), 2, cv2.LINE_AA)
    cv2.line(vis, (int(CX), 0), (int(CX), H), (70, 74, 78), 1)

    bar = np.zeros((76, W, 3), np.uint8)
    vis[H - 76:H] = cv2.addWeighted(vis[H - 76:H], 0.18, bar, 0.82, 0)
    c1 = (255, 255, 255)
    cv2.putText(vis, f"IDEAL LAP REPLAY | YOLO11n-seg @480 | ESP32", (12, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.48, (20, 22, 24), 3, cv2.LINE_AA)
    cv2.putText(vis, f"IDEAL LAP REPLAY | YOLO11n-seg @480 | ESP32", (12, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.48, c1, 1, cv2.LINE_AA)
    line1 = f"frame {idx + 1}/{total}  loop {loop}  {state['src']}  n={state['rows']}  {infer_ms:.0f} ms"
    line2 = f"lat {state['lat'] * 100:+.1f} cm   head {math.degrees(state['head']):+.1f} deg   steer {math.degrees(state['steer']):+.1f} deg"
    line3 = f"L {left:3d}   R {right:3d}   {'MOTORS ON' if motor_on else ('gated' if gated else 'motors off')}"
    if status_note:
        line3 += "   " + status_note
    for i, text in enumerate((line1, line2, line3)):
        cv2.putText(vis, text, (12, H - 50 + i * 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (20, 22, 24), 3, cv2.LINE_AA)
        cv2.putText(vis, text, (12, H - 50 + i * 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (235, 238, 242) if i < 2 else
                    ((120, 240, 120) if motor_on else (180, 186, 194)), 1, cv2.LINE_AA)
    return vis


def main() -> None:
    ap = argparse.ArgumentParser(description="ideal-lap replay -> lane inference -> ESP32 web")
    ap.add_argument("--frames", default=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                     "ideal_run_640x480"))
    ap.add_argument("--port", type=int, default=PORT)
    ap.add_argument("--server", default=PREDICT_URL)
    ap.add_argument("--serial", default=SERIAL_PORT)
    ap.add_argument("--no-motor", action="store_true", help="do not open the serial port")
    ap.add_argument("--no-auto-motor", action="store_true",
                    help="start with motors gated (default: drive as soon as the page is open, like test-yolo)")
    ap.add_argument("--fps", type=float, default=PLAY_FPS)
    args = ap.parse_args()

    files = sorted(glob.glob(os.path.join(args.frames, "*.jpg")))
    if not files:
        raise SystemExit(f"no .jpg frames in {args.frames}")
    print(f"{len(files)} frames from {args.frames}")
    print(f"inference: {args.server}")

    hub, rt = Hub(), Runtime()
    motors = Motors(args.serial, not args.no_motor)
    rt.motor_on = motors.ok and not args.no_auto_motor
    print("serial:", "ok" if motors.ok else (motors.error or "off"),
          "| motors:", "armed (test-yolo style)" if rt.motor_on else "gated")
    ctrl = Controller()
    total = len(files)
    idx, loop = 0, 0
    last_send = 0.0
    last_time = time.perf_counter()
    fps = 0.0
    error = ""

    handler = type("Handler", (_Handler,), {
        "hub": hub, "runtime": rt, "page": PAGE.encode(), "motors": motors})
    httpd = ThreadingHTTPServer(("0.0.0.0", args.port), handler)
    httpd.daemon_threads = True
    threading.Thread(target=httpd.serve_forever, name="web", daemon=True).start()
    print(f"web: http://0.0.0.0:{args.port}/")

    try:
        while True:
            if rt.restart:
                rt.restart = False
                idx, loop = 0, 0
                ctrl = Controller()
            if rt.paused or hub.clients == 0:
                now = time.perf_counter()
                if now - last_send > 0.15:
                    motors.send(0, 0)
                    last_send = now
                hub.set_status({"frame": idx, "total": total, "loop": loop,
                                "fps": round(fps, 1), "infer_ms": None, "src": "paused",
                                "valid": False, "rows": 0, "lat_cm": 0, "head_deg": 0,
                                "steer_deg": 0, "left": 0, "right": 0,
                                "motor_on": False, "error": error})
                time.sleep(0.08)
                continue
            t0 = time.perf_counter()
            frame = cv2.imread(files[idx])
            if frame is None:
                idx = (idx + 1) % total
                continue
            ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 88])
            if not ok:
                continue
            try:
                req = Request(args.server, data=buf.tobytes(),
                              headers={"Content-Type": "image/jpeg"})
                with urlopen(req, timeout=2.5) as r:
                    out = json.load(r)
                infer_ms = (time.perf_counter() - t0) * 1000
                error = ""
            except Exception as exc:
                error = f"inference: {exc}"
                motors.send(0, 0)
                hub.set_status({"frame": idx, "total": total, "loop": loop,
                                "fps": round(fps, 1), "infer_ms": None, "src": "error",
                                "valid": False, "rows": 0, "lat_cm": 0, "head_deg": 0,
                                "steer_deg": 0, "left": 0, "right": 0,
                                "motor_on": False, "error": error})
                time.sleep(0.2)
                continue
            st = ctrl.update(out)
            st["raw"] = {k: out.get(k, []) for k in ("outer", "inner", "centre")}
            cmd_left, cmd_right = motor_values(st, motors.ok)
            driving = rt.motor_on and hub.clients > 0
            now = time.perf_counter()
            if driving:
                motors.send(cmd_left, cmd_right)
                last_send = now
            elif now - last_send > 0.15:
                motors.send(0, 0)
                last_send = now
            vis = draw(frame, st, cmd_left, cmd_right, infer_ms, idx, total, loop, driving, "", not driving)
            ok, jb = cv2.imencode(".jpg", vis, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_Q])
            if ok:
                status = {"frame": idx + 1, "total": total, "loop": loop,
                          "fps": round(fps, 1), "infer_ms": round(infer_ms, 1),
                          "src": st["src"], "valid": st["valid"], "rows": st["rows"],
                          "lat_cm": round(st["lat"] * 100, 1),
                          "head_deg": round(math.degrees(st["head"]), 1),
                          "steer_deg": round(math.degrees(st["steer"]), 1),
                          "left": cmd_left, "right": cmd_right,
                          "motor_on": driving, "gated": not driving,
                          "sent": driving, "error": error}
                hub.publish(jb.tobytes(), status)
            idx = (idx + 1) % total
            if idx == 0:
                loop += 1
            sleep = 1.0 / max(1.0, args.fps) - (time.perf_counter() - t0)
            if sleep > 0:
                time.sleep(sleep)
            loop_dt = time.perf_counter() - t0
            fps = 0.85 * fps + 0.15 * (1.0 / max(loop_dt, 1e-3))
    except KeyboardInterrupt:
        pass
    finally:
        motors.close()
        httpd.shutdown()


if __name__ == "__main__":
    main()
