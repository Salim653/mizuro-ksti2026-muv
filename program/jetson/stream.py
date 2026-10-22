from __future__ import annotations

import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any

INDEX = """<!doctype html><meta charset=utf-8><title>Mizuro camera</title>
<style>
  body{{margin:0;background:#111;color:#eee;font:14px system-ui,sans-serif;
        display:flex;flex-direction:column;align-items:center;gap:12px;padding:16px}}
  img,canvas{{max-width:100%;height:auto;border:1px solid #333;border-radius:4px}}
  a{{color:#7fd}}
</style>
<h1>Mizuro camera</h1>
<img src="/stream" alt="live stream">
<p><a href="/snapshot">single frame</a> &middot; {width}x{height} @ {fps}fps, q{quality}</p>
"""


class FrameHub:
    """Latest JPEG frame shared by the capture loop and every HTTP client.

    Encoding is skipped while nobody is subscribed and throttled to `fps`
    otherwise, so a connected viewer cannot slow the control loop.
    """

    def __init__(self, quality: int = 75, fps: float = 15.0,
                 width: int = 0, height: int = 0) -> None:
        self.quality = quality
        self.fps = fps
        self.frame_size = (width, height) if width and height else None
        self.encoded_frames = 0
        self._cv2: Any = None
        self._jpeg: bytes | None = None
        self._seq = 0
        self._clients = 0
        self._pending = 0
        self._last_encode = 0.0
        self._cond = threading.Condition()

    @property
    def clients(self) -> int:
        with self._cond:
            return self._clients

    def attach(self) -> None:
        with self._cond:
            self._clients += 1

    def detach(self) -> None:
        with self._cond:
            self._clients = max(0, self._clients - 1)
            self._cond.notify_all()

    def publish(self, frame) -> None:
        with self._cond:
            if self._clients == 0 and self._pending == 0:
                return
        now = time.monotonic()
        if now - self._last_encode < 1.0 / self.fps:
            return
        if self._cv2 is None:
            import cv2
            self._cv2 = cv2
        if self.frame_size is not None:
            frame = self._cv2.resize(frame, self.frame_size, interpolation=self._cv2.INTER_AREA)
        ok, buf = self._cv2.imencode(".jpg", frame, [self._cv2.IMWRITE_JPEG_QUALITY, self.quality])
        if not ok:
            return
        with self._cond:
            self._jpeg = buf.tobytes()
            self._seq += 1
            self._last_encode = now
            self._pending = 0
            self.encoded_frames += 1
            self._cond.notify_all()

    def snapshot(self, timeout: float = 2.0) -> bytes | None:
        with self._cond:
            if self._jpeg is None and self._pending == 0:
                self._pending = 1
                seq = self._seq
                self._cond.wait_for(lambda: self._seq != seq, timeout)
            return self._jpeg

    def wait(self, seq: int, timeout: float = 5.0) -> tuple[bytes | None, int]:
        with self._cond:
            self._cond.wait_for(lambda: self._seq != seq or self._clients == 0, timeout)
            return self._jpeg, self._seq


class _Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.0"
    hub: FrameHub
    page: bytes

    def log_message(self, *args: Any) -> None:
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
                if new_seq == seq:
                    continue
                seq = new_seq
                if jpeg is None:
                    continue
                self.wfile.write(
                    b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                    + str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            pass
        finally:
            self.hub.detach()


class MjpegServer:
    """MJPEG over HTTP for one FrameHub; `clients` stays exact across reconnects."""

    def __init__(self, hub: FrameHub, host: str = "0.0.0.0", port: int = 8080) -> None:
        size = hub.frame_size or (0, 0)
        page = INDEX.format(width=size[0] or "native", height=size[1] or "native",
                            fps=hub.fps, quality=hub.quality).encode()
        handler = type("Handler", (_Handler,), {"hub": hub, "page": page})
        self.hub = hub
        self._httpd = ThreadingHTTPServer((host, port), handler)
        self._httpd.daemon_threads = True
        self._thread = threading.Thread(target=self._httpd.serve_forever,
                                        name="mjpeg-server", daemon=True)

    @property
    def port(self) -> int:
        return int(self._httpd.server_address[1])

    @property
    def url(self) -> str:
        host, port = self._httpd.server_address[:2]
        return f"http://{host}:{port}/"

    def start(self) -> "MjpegServer":
        self._thread.start()
        return self

    def stop(self) -> None:
        self._httpd.shutdown()
        self._httpd.server_close()

    def __enter__(self) -> "MjpegServer":
        return self.start()

    def __exit__(self, *exc: Any) -> None:
        self.stop()
