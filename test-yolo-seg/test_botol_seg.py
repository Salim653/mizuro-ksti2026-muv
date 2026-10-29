#!/usr/bin/env python3
"""Tes YOLO11-seg: ikuti botol, setir dari centroid mask, stream ke halaman web.

- Model instance segmentation (yolo11n-seg), satu kelas target (default COCO 39 = bottle).
- Arah belok diambil dari centroid mask terhadap tengah frame:
  objek di kiri -> motor kanan lebih cepat (robot belok kiri), dan sebaliknya.
- Kemiringan objek (lean) dihitung dari momen kedua mask (principal axis),
  hanya untuk ditampilkan; yang mengemudikan motor tetap offset centroid.
- Hasil inferensi (mask + HUD) di-stream sebagai MJPEG, plus panel status.

Jalankan di Jetson:
    cd ~/mizuro-ksti2026-muv/test-yolo-seg
    ../.venv/bin/python test_botol_seg.py

Uji aman tanpa motor (robot diam, serial tidak dibuka):
    ../.venv/bin/python test_botol_seg.py --no-motor

Buka http://<ip-jetson>:8090/ dari laptop.
"""

from __future__ import annotations

import argparse
import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import serial
from ultralytics import YOLO

# ===== Pengaturan =====
MODEL_PATH   = "yolo11n-seg.pt"
SERIAL_PORT  = "/dev/ttyCH341USB0"
BAUD         = 115200
TARGET_ID    = 39      # kelas "bottle" di COCO
CONF         = 0.45
BASE_SPEED   = 140     # kecepatan maju (0-255)
TURN_GAIN    = 140     # offset 1.0 -> selisih roda 140
DEADZONE     = 0.06    # offset di bawah ini dianggap lurus
SMOOTH       = 0.35    # EMA offset, biar motor tidak menyentak
CONFIRM      = 2       # frame berturut-turut terlihat -> MAJU
LOST         = 3       # frame berturut-turut hilang -> STOP
INFER_W, INFER_H = 320, 240
JPEG_QUALITY = 80
STREAM_FPS   = 20
PORT         = 8090

ACCENT = (75, 176, 232)   # BGR, amber
INK    = (238, 234, 231)


# ===== Web: MJPEG + status JSON =====
PAGE = """<!doctype html>
<html lang="en">
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>YOLO11-seg · bottle follow</title>
<style>
  :root{--bg:#0e1012;--panel:#16191c;--line:#252a2f;--ink:#e7eaee;--mute:#8b939c;--accent:#e8b04b}
  *{box-sizing:border-box}
  body{margin:0;background:var(--bg);color:var(--ink);font:14px/1.5 system-ui,"Segoe UI",sans-serif;padding:20px}
  header{display:flex;align-items:baseline;gap:12px;margin-bottom:16px}
  h1{font-size:16px;font-weight:600;margin:0;letter-spacing:.01em}
  .sub{color:var(--mute);font-size:12px}
  .wrap{display:grid;grid-template-columns:minmax(0,1fr) 280px;gap:16px;max-width:1200px}
  @media (max-width:820px){.wrap{grid-template-columns:1fr}}
  .video{background:#000;border:1px solid var(--line);border-radius:8px;overflow:hidden;line-height:0}
  .video img{width:100%;height:auto;display:block}
  .panel{background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:14px 16px}
  .row{display:flex;justify-content:space-between;align-items:baseline;gap:12px;padding:7px 0;border-bottom:1px solid var(--line)}
  .row:last-child{border-bottom:0}
  .k{color:var(--mute);font-size:11px;text-transform:uppercase;letter-spacing:.06em}
  .v{font-variant-numeric:tabular-nums;font-weight:600}
  .mode{display:inline-flex;align-items:center;gap:7px;font-size:12px;font-weight:600;letter-spacing:.05em;text-transform:uppercase;color:var(--mute)}
  .dot{width:8px;height:8px;border-radius:50%;background:var(--mute)}
  .mode.run{color:var(--accent)}
  .mode.run .dot{background:var(--accent)}
  .bar{position:relative;height:8px;background:#0a0c0e;border:1px solid var(--line);border-radius:999px;margin-top:8px;overflow:hidden}
  .bar i{position:absolute;top:0;bottom:0;left:50%;width:2px;margin-left:-1px;background:var(--mute);opacity:.6}
  .bar b{position:absolute;top:0;bottom:0;background:var(--accent);border-radius:999px;transition:left .12s,width .12s}
  .motors{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-top:12px}
  .motor{background:#0a0c0e;border:1px solid var(--line);border-radius:6px;padding:9px 10px}
  .motor .k{display:block;margin-bottom:2px}
  .motor .n{font-size:20px;font-weight:600;font-variant-numeric:tabular-nums}
  footer{max-width:1200px;color:var(--mute);font-size:12px;margin-top:14px}
  footer a{color:var(--accent);text-decoration:none}
</style>
<body>
<header>
  <h1>YOLO11-seg · bottle follow</h1>
  <span class="sub" id="meta">—</span>
</header>
<div class="wrap">
  <div class="video"><img src="/stream" alt="live inference"></div>
  <aside class="panel">
    <div class="row"><span class="k">Mode</span><span class="mode" id="mode"><span class="dot"></span><span id="modetxt">connecting</span></span></div>
    <div class="row"><span class="k">Target</span><span class="v" id="target">—</span></div>
    <div class="row"><span class="k">Confidence</span><span class="v" id="conf">—</span></div>
    <div class="row"><span class="k">Mask area</span><span class="v" id="area">—</span></div>
    <div class="row"><span class="k">Lean</span><span class="v" id="lean">—</span></div>
    <div class="row" style="display:block">
      <span class="k">Offset from centre</span>
      <div class="bar"><i></i><b id="offbar" style="left:50%;width:0"></b></div>
      <div class="sub" id="offtxt" style="margin-top:6px">—</div>
    </div>
    <div class="motors">
      <div class="motor"><span class="k">Left motor</span><span class="n" id="lmotor">0</span></div>
      <div class="motor"><span class="k">Right motor</span><span class="n" id="rmotor">0</span></div>
    </div>
  </aside>
</div>
<footer>Steering from segmentation centroid · command sent every frame · <a href="/snapshot">snapshot</a></footer>
<script>
const $ = id => document.getElementById(id);
async function tick() {
  try {
    const s = await (await fetch('/status', {cache: 'no-store'})).json();
    const run = !!(s.active && s.found);
    $('mode').className = 'mode' + (run ? ' run' : '');
    $('modetxt').textContent = run ? 'tracking' : (s.found ? 'confirming' : 'searching');
    $('meta').textContent = (s.fps ? s.fps.toFixed(1) + ' fps' : '—') + ' · ' + s.frame_w + '×' + s.frame_h;
    $('target').textContent = s.target ? s.target + ' [' + s.class_id + ']' : '—';
    $('conf').textContent = s.conf != null ? (s.conf * 100).toFixed(0) + '%' : '—';
    $('area').textContent = s.area != null ? s.area.toFixed(1) + '%' : '—';
    $('lean').textContent = s.lean != null ? (s.lean > 0 ? '+' : '') + s.lean.toFixed(0) + '°' : '—';
    const off = s.offset ?? 0, w = Math.min(50, Math.abs(off) * 50);
    $('offbar').style.left = (off < 0 ? 50 - w : 50) + '%';
    $('offbar').style.width = w + '%';
    $('offtxt').textContent = s.found
      ? (off < 0 ? 'target left' : 'target right') + ' · ' + Math.abs(off * 100).toFixed(0) + '% of half-width'
      : 'no target';
    $('lmotor').textContent = s.left;
    $('rmotor').textContent = s.right;
  } catch (e) {
    $('mode').className = 'mode';
    $('modetxt').textContent = 'offline';
  }
}
setInterval(tick, 400); tick();
</script>
</html>
"""


class FrameHub:
    """JPEG terakhir + status, dibagi antara loop inferensi dan semua klien HTTP."""

    def __init__(self, fps: float = STREAM_FPS, quality: int = JPEG_QUALITY) -> None:
        self.fps = fps
        self.quality = quality
        self._jpeg: bytes | None = None
        self._status: dict = {"found": False, "active": False, "target": None,
                              "class_id": TARGET_ID, "conf": None, "offset": None,
                              "lean": None, "area": None, "left": 0, "right": 0,
                              "fps": 0.0, "frame_w": 0, "frame_h": 0}
        self._seq = 0
        self._clients = 0
        self._pending = 0
        self._last_encode = 0.0
        self._cond = threading.Condition()

    @property
    def status(self) -> dict:
        with self._cond:
            return dict(self._status)

    def attach(self) -> None:
        with self._cond:
            self._clients += 1

    def detach(self) -> None:
        with self._cond:
            self._clients = max(0, self._clients - 1)

    def publish(self, frame, status: dict) -> None:
        with self._cond:
            self._status = status
            self._cond.notify_all()
            if self._clients == 0 and self._pending == 0:
                return
        now = time.monotonic()
        if now - self._last_encode < 1.0 / self.fps:
            return
        ok, buf = cv2.imencode(".jpg", frame,
                               [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        if not ok:
            return
        with self._cond:
            self._jpeg = buf.tobytes()
            self._seq += 1
            self._last_encode = now
            self._pending = 0
            self._cond.notify_all()

    def snapshot(self, timeout: float = 2.0) -> bytes | None:
        with self._cond:
            if self._jpeg is None and self._pending == 0:
                self._pending = 1
                seq = self._seq
                self._cond.wait_for(lambda: self._seq != seq, timeout)
            return self._jpeg

    def wait(self, seq: int, timeout: float = 5.0):
        with self._cond:
            self._cond.wait_for(lambda: self._seq != seq or self._clients == 0, timeout)
            return self._jpeg, self._seq


class _Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.0"
    hub: FrameHub
    page: bytes

    def log_message(self, *args) -> None:
        pass

    def do_GET(self) -> None:
        if self.path in ("/", "/index.html"):
            self._reply(200, "text/html; charset=utf-8", self.page)
        elif self.path == "/stream":
            self._stream()
        elif self.path == "/snapshot":
            jpeg = self.hub.snapshot()
            if jpeg is None:
                self._reply(503, "text/plain", b"no frame yet")
            else:
                self._reply(200, "image/jpeg", jpeg)
        elif self.path == "/status":
            body = json.dumps(self.hub.status).encode()
            self._reply(200, "application/json", body)
        else:
            self._reply(404, "text/plain", b"not found")

    def _reply(self, code: int, ctype: str, body: bytes) -> None:
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _stream(self) -> None:
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.hub.attach()
        seq = -1
        try:
            while True:
                jpeg, new_seq = self.hub.wait(seq)
                if new_seq == seq or jpeg is None:
                    continue
                seq = new_seq
                self.wfile.write(
                    b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                    + str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            pass
        finally:
            self.hub.detach()


def start_server(hub: FrameHub, port: int) -> ThreadingHTTPServer:
    handler = type("Handler", (_Handler,), {"hub": hub, "page": PAGE.encode()})
    httpd = ThreadingHTTPServer(("0.0.0.0", port), handler)
    httpd.daemon_threads = True
    threading.Thread(target=httpd.serve_forever, name="mjpeg-server", daemon=True).start()
    return httpd


# ===== Motor =====
class Motors:
    def __init__(self, port: str, baud: int, enabled: bool = True) -> None:
        self.ser = None
        if enabled:
            self.ser = serial.Serial(port, baud, timeout=0)
            time.sleep(2)  # ESP32 reset saat port dibuka

    def send(self, left: int, right: int) -> None:
        if self.ser is None:
            return
        self.ser.write(f"M {left} {right}\n".encode())
        self.ser.reset_input_buffer()  # buang balasan "OK" agar buffer tidak menumpuk

    def close(self) -> None:
        if self.ser is None:
            return
        try:
            self.send(0, 0)
        finally:
            self.ser.close()


# ===== Kamera =====
def open_camera(source: str):
    if source == "csi":
        pipeline = (
            "nvarguscamerasrc sensor-id=0 ! "
            "video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1 ! "
            "nvvidconv ! video/x-raw,format=BGRx ! "
            "videoconvert ! video/x-raw,format=BGR ! appsink drop=1"
        )
        return cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    return cv2.VideoCapture(source)


# ===== Segmentasi & kemudi =====
def analyze(result, small_shape):
    """Ambil instance target paling yakin, ukur dari mask-nya saja."""
    if result.masks is None or result.boxes is None or len(result.boxes) == 0:
        return None
    confs = result.boxes.conf
    best = int(confs.argmax())
    poly = result.masks.xy[best]
    if poly is None or len(poly) < 3:
        return None

    h, w = small_shape[:2]
    mask = np.zeros((h, w), np.uint8)
    cv2.fillPoly(mask, [np.round(poly).astype(np.int32)], 1)
    m = cv2.moments(mask, True)
    if m["m00"] <= 0:
        return None

    cx = m["m10"] / m["m00"]
    cy = m["m01"] / m["m00"]
    area = m["m00"] / (w * h)
    mu20 = m["mu20"] / m["m00"]
    mu02 = m["mu02"] / m["m00"]
    mu11 = m["mu11"] / m["m00"]
    axis = 0.5 * math.degrees(math.atan2(2 * mu11, mu20 - mu02))
    # principal axis hampir vertikal = 90 deg; selisih dari vertikal = kemiringan,
    # tandanya diambil dari mu11 supaya "+" berarti puncak objek miring ke kiri.
    lean = math.copysign(90.0 - abs(axis), mu11)

    return {
        "poly": poly,
        "cx": cx,
        "cy": cy,
        "offset": (cx - w / 2) / (w / 2),
        "area": area,
        "lean": lean,
        "conf": float(confs[best]),
    }


def steer(err: float, base: int) -> tuple[int, int]:
    """Offset negatif (objek di kiri) -> roda kanan lebih cepat, dan sebaliknya."""
    if abs(err) < DEADZONE:
        return base, base
    turn = int(round(TURN_GAIN * abs(err)))
    left, right = (base - turn, base + turn) if err < 0 else (base + turn, base - turn)
    clamp = lambda v: max(0, min(255, v))
    return clamp(left), clamp(right)


def put(frame, text: str, x: int, y: int, scale: float = 0.6,
        color=INK, bold: int = 2) -> None:
    cv2.putText(frame, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale,
                (0, 0, 0), bold + 2, cv2.LINE_AA)
    cv2.putText(frame, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale,
                color, bold, cv2.LINE_AA)


def draw(frame, det, small_shape, err, left, right, name, class_id, fps):
    h, w = frame.shape[:2]
    sx = w / small_shape[1]
    sy = h / small_shape[0]
    mid = w // 2
    cv2.line(frame, (mid, 0), (mid, h), (70, 74, 78), 1)

    if det:
        pts = (det["poly"] * [sx, sy]).astype(np.int32)
        shade = frame.copy()
        cv2.fillPoly(shade, [pts], ACCENT)
        cv2.addWeighted(shade, 0.30, frame, 0.70, 0, frame)
        cv2.polylines(frame, [pts], True, ACCENT, 2)

        px, py = int(det["cx"] * sx), int(det["cy"] * sy)
        cv2.line(frame, (mid, py), (px, py), ACCENT, 1, cv2.LINE_AA)
        cv2.circle(frame, (px, py), 6, (255, 255, 255), -1, cv2.LINE_AA)
        cv2.circle(frame, (px, py), 6, ACCENT, 2, cv2.LINE_AA)

    lines = [
        f"{name} [{class_id}]  conf {det['conf']:.2f}" if det else "no target",
        f"offset {err:+.2f}   lean {det['lean']:+.0f} deg" if det else f"offset {err:+.2f}",
        f"motor L {left:3d}  R {right:3d}   {'MAJU' if (left or right) else 'STOP'}",
        f"FPS {fps:.1f}",
    ]
    cv2.rectangle(frame, (12, 12), (360, 12 + 26 * len(lines) + 10), (16, 18, 20), -1)
    cv2.rectangle(frame, (12, 12), (360, 12 + 26 * len(lines) + 10), (37, 42, 47), 1)
    for i, text in enumerate(lines):
        put(frame, text, 24, 38 + i * 26)


# ===== Main =====
def parse_args():
    p = argparse.ArgumentParser(description="YOLO11-seg follow test")
    p.add_argument("--model", default=MODEL_PATH)
    p.add_argument("--class-id", type=int, default=TARGET_ID)
    p.add_argument("--conf", type=float, default=CONF)
    p.add_argument("--speed", type=int, default=BASE_SPEED)
    p.add_argument("--port", type=int, default=PORT)
    p.add_argument("--serial", default=SERIAL_PORT)
    p.add_argument("--source", default="csi",
                   help="'csi' untuk kamera IMX219, atau path video/gambar")
    p.add_argument("--no-motor", action="store_true",
                   help="jangan buka serial (uji di meja)")
    p.add_argument("--device", default="0",
                   help="0 = GPU, 'cpu' untuk uji tanpa mengganggu loop lain")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    motors = Motors(args.serial, BAUD, not args.no_motor)
    model = YOLO(args.model)
    name = model.names.get(args.class_id, str(args.class_id))

    still = cv2.imread(args.source) if args.source != "csi" else None
    cap = None
    if still is not None:
        print(f"Sumber gambar statis: {args.source} (diulang tiap frame)")
    else:
        cap = open_camera(args.source)
        if not cap.isOpened():
            motors.close()
            raise SystemExit("Error: camera tidak bisa dibuka.")

    hub = FrameHub()
    httpd = start_server(hub, args.port)
    print(f"Stream: http://<ip-jetson>:{httpd.server_address[1]}/  (Ctrl+C untuk berhenti)")
    if args.no_motor:
        print("Mode --no-motor: serial tidak dibuka, roda tidak bergerak.")

    seen = missed = 0
    active = False
    err_ema = 0.0
    t0, n, fps = time.time(), 0, 0.0

    try:
        while True:
            if still is not None:
                frame = still.copy()
                ret = True
            else:
                ret, frame = cap.read()
                if not ret and args.source != "csi":
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # loop video
                    ret, frame = cap.read()
            if not ret:
                print("Error: frame tidak terbaca.")
                break

            small = cv2.resize(frame, (INFER_W, INFER_H))
            result = model(small, verbose=False, device=args.device, imgsz=INFER_W,
                           classes=[args.class_id], conf=args.conf)[0]
            det = analyze(result, small.shape)
            found = det is not None

            if found:
                seen += 1
                missed = 0
                if seen >= CONFIRM:
                    active = True
                err_ema = SMOOTH * det["offset"] + (1 - SMOOTH) * err_ema
            else:
                missed += 1
                seen = 0
                if missed >= LOST:
                    active = False
                err_ema = 0.0

            left, right = steer(err_ema, args.speed) if (active and found) else (0, 0)
            motors.send(left, right)  # tiap frame: failsafe ESP32 butuh perintah rutin

            n += 1
            now = time.time()
            if now - t0 >= 1.0:
                fps = n / (now - t0)
                t0, n = now, 0
                print(f"FPS {fps:.1f} | {name}: {'ya' if found else 'tidak'} | "
                      f"offset {err_ema:+.2f} | motor L {left} R {right}")

            draw(frame, det, small.shape, err_ema, left, right, name, args.class_id, fps)
            hub.publish(frame, {
                "found": found,
                "active": active,
                "target": name if det else None,
                "class_id": args.class_id,
                "conf": det["conf"] if det else None,
                "offset": err_ema,
                "lean": det["lean"] if det else None,
                "area": det["area"] * 100 if det else None,
                "left": left,
                "right": right,
                "fps": fps,
                "frame_w": frame.shape[1],
                "frame_h": frame.shape[0],
            })
    except KeyboardInterrupt:
        print("\nBerhenti.")
    finally:
        motors.close()
        if cap is not None:
            cap.release()


if __name__ == "__main__":
    main()
