"""Bench server: dashboard yang bisa MENUKAR sumber kamera saat runtime.

Alur yang didukung (bench/pit saja, TANPA hardware):

    jalankan dashboard -> toggle kamera/video -> pilih video

`main.py --bench` menjalankan satu proses yang menggabungkan:

- kamera (CSI atau file video) di thread sendiri; tiap frame dipublish
  ke MJPEG (`mizuro/stream.py`) supaya terlihat di browser,
- `LaneTracker` per frame supaya angka lajur (error lateral, heading,
  confidence, stasiun) hidup di dashboard,
- endpoint kontrol sumber (`GET /api/sources`, `POST /api/source`) yang
  menukar kamera TANPA restart proses.

KESELAMATAN: modul ini TIDAK PERNAH menyentuh serial atau motor — tidak
ada `McuLink` di jalur ini sama sekali. Roda fisik tidak bisa menyala
dari sini, bukan karena ditambahi penjaga, tapi karena jalur ke motor
tidak ada. `dry_run` juga dipaksa true oleh `main.py` sebelum masuk sini.

Aturan lomba: TIDAK ADA komunikasi selama percobaan. Matikan proses ini
(dan semua HTTP) sebelum setiap attempt.
"""
from __future__ import annotations

import collections
import threading
import time
from pathlib import Path
from typing import Any

from .camera import CameraSource, EndOfVideo

# Ekstensi yang muncul di pemilih video dashboard.
VIDEO_SUFFIXES = (".mp4", ".avi", ".mov", ".mkv", ".webm", ".flv")
# Nama file dari query/body divalidasi ketat: tanpa separator, ekstensi
# dikenal, harus langsung di dalam direktori daftar (symlink pun).
MAX_NAME_LEN = 128
# Jendela pengukuran fps (detik) — angka fps yang tampil di dashboard.
FPS_WINDOW_S = 2.0
# Jeda kalau sumber gagal: jangan spin 100% CPU menunggu operator.
RETRY_S = 0.25


def _blank_status() -> dict[str, Any]:
    """Status sebelum ada frame: semua field ada supaya UI tidak bolong."""
    return {
        "available": False,     # kamera aktif dan frame mengalir
        "ended": False,         # file video habis (loop mati)
        "error": "",            # pesan gagal terakhir ("" = tidak ada)
        "tracker_error": "",    # error tracker (loop kamera tetap jalan)
        "frames": 0,
        "fps": 0.0,
        "lane_error_m": None,
        "heading_error_rad": None,
        "curvature_inv_m": None,
        "confidence": None,
        "stations": "",
        "polarity": "",
    }


class BenchControl:
    """Loop kamera + tracker di thread, dengan sumber yang bisa ditukar.

    Bukan server HTTP sendiri: objek ini disuntikkan ke
    `dashboard.make_server(..., control=...)` yang menambah endpoint
    `/api/sources`, `POST /api/source`, dan status kamera hidup di
    `/api/status` (key `bench`). Dipisah supaya logika kamera bisa
    diuji tanpa HTTP, dan HTTP tanpa cv2.
    """

    def __init__(self, config, *, videos_dir, calib, tracker, hub,
                 opener=None, initial: str = "",
                 video_loop: bool = True) -> None:
        self.config = config
        self.videos_dir = Path(videos_dir)
        self.calib = calib
        self.tracker = tracker
        self.hub = hub
        self.source = CameraSource(config.image_width, config.image_height,
                                   opener=opener, video_loop=video_loop)
        self._initial = initial
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        # _lock menjaga status hidup + penghitung frame; _select_lock
        # memserialkan select() supaya dua POST tidak bertabrakan.
        # RLock (bukan Lock): select() -> _open() mengambil lock yang
        # sama dari thread yang sama.
        self._lock = threading.Lock()
        self._select_lock = threading.RLock()
        self._status = _blank_status()
        self._frames = 0
        self._times: collections.deque[float] = collections.deque()
        self._desc = self.source.describe()

    # --- siklus hidup ---------------------------------------------------------

    def start(self) -> "BenchControl":
        """Buka sumber awal (boleh gagal — error masuk status) + jalankan
        thread kamera. Thread tetap hidup walau sumber awal gagal supaya
        operator bisa memilih video dari dashboard."""
        self._stop.clear()
        try:
            self._open(self._initial)
        except (RuntimeError, ValueError) as exc:
            self._patch(available=False, error=str(exc))
        self._thread = threading.Thread(target=self._loop, name="bench-camera",
                                        daemon=True)
        self._thread.start()
        return self

    def stop(self) -> None:
        """Hentikan thread kamera (tunggu maks 2 s) lalu tutup sumber.

        Kalau thread nyangkut di `read()` (kamera CSI macet), join
        timeout dan sumber ditutup begitu saja — bench tool, Ctrl+C
        harus tetap responsif.
        """
        self._stop.set()
        thread, self._thread = self._thread, None
        if thread is not None:
            thread.join(timeout=2.0)
        self.source.close()

    def wait(self, timeout: float | None = None) -> bool:
        """Tunggu sampai stop di-set (atau timeout). True kalau di-stop."""
        return self._stop.wait(timeout)

    def __enter__(self) -> "BenchControl":
        return self.start()

    def __exit__(self, *exc) -> None:
        self.stop()

    # --- endpoint kontrol -----------------------------------------------------

    def sources(self) -> dict[str, Any]:
        """Daftar sumber buat pemilih dashboard.

        CSI dilaporkan `available` hanya kalau build cv2 punya backend
        GStreamer — kalau tidak, tombol CSI dimatikan di UI dengan
        alasannya, bukan gagal saat ditekan.
        """
        videos: list[dict[str, Any]] = []
        directory = self.videos_dir
        if directory.is_dir():
            for path in sorted(directory.iterdir()):
                if not path.is_file():
                    continue
                if path.suffix.lower() not in VIDEO_SUFFIXES:
                    continue
                try:
                    size = path.stat().st_size
                except OSError:
                    continue
                videos.append({"name": path.name, "size_bytes": size})
        csi_ok = self._csi_available()
        pipeline = self._desc.get("pipeline", "")
        return {
            "current": self._desc.get("source", "none"),
            "current_name": Path(pipeline).name if pipeline else "",
            "csi": {
                "available": csi_ok,
                "reason": "" if csi_ok else
                "cv2 tanpa backend GStreamer — jalankan dengan "
                "/usr/bin/python3 (JetPack), bukan .venv",
            },
            "videos": videos,
            "videos_dir": str(directory),
            "video_loop": self.source.video_loop,
        }

    def select(self, payload: Any) -> dict[str, Any]:
        """Tukar sumber kamera. `payload` = {"source": "csi"} atau
        {"source": "video", "name": "..."}.

        ValueError = permintaan tidak sah (HTTP 400); RuntimeError =
        sumber gagal dibuka, sumber lama tetap aktif (HTTP 500).
        """
        if not isinstance(payload, dict):
            raise ValueError("payload harus objek JSON")
        kind = payload.get("source")
        if kind == "csi":
            pipeline = ""
        elif kind == "video":
            pipeline = self._resolve_video(payload.get("name"))
        else:
            raise ValueError("source harus 'csi' atau 'video'")
        self._open(pipeline)
        return self.status()

    def _open(self, pipeline: str) -> None:
        """Buka sumber baru + reset status. Dipakai `select()` (HTTP) dan
        `start()` (sumber awal). RuntimeError dari Camera berarti sumber
        gagal dibuka — sumber lama tetap aktif."""
        with self._select_lock:
            self.source.select(pipeline)
            with self._lock:
                self._frames = 0
                self._times.clear()
            self._desc = {**self.source.describe(),
                          "switches": self.source.switches,
                          "loop": self.source.video_loop}
            self._patch(ended=False, error="", tracker_error="",
                        available=True, frames=0, fps=0.0,
                        lane_error_m=None, heading_error_rad=None,
                        curvature_inv_m=None, confidence=None,
                        stations="", polarity="")

    def status(self) -> dict[str, Any]:
        """Status kamera hidup (dipanggil endpoint /api/status tiap tick)."""
        with self._lock:
            live = dict(self._status)
        return {**self._desc, **live}

    # --- internal --------------------------------------------------------------

    @staticmethod
    def _csi_available() -> bool:
        """True kalau build cv2 di interpreter ini punya GStreamer."""
        try:
            import cv2
            from . import camera as camera_module
            # lewat modul (bukan nama yang di-import) supaya tes bisa
            # monkeypatch mizuro.camera.has_gstreamer
            return camera_module.has_gstreamer(cv2)
        except Exception:      # cv2 tidak terpasang sama sekali
            return False

    def _resolve_video(self, name: Any) -> str:
        """Nama file -> path absolut di dalam `videos_dir`.

        Nama divalidasi ketat: tanpa separator path, ekstensi dikenal,
        dan (setelah resolve) induknya HARUS direktori daftar — symlink
        yang menunjuk ke luar juga ditolak. Path absolut dari body tidak
        pernah diterima; yang bisa dipilih hanya isi direktori.
        """
        if not isinstance(name, str) or not name:
            raise ValueError("nama video kosong")
        if len(name) > MAX_NAME_LEN:
            raise ValueError("nama video terlalu panjang")
        candidate = Path(name)
        if candidate.name != name:
            raise ValueError(f"nama video tidak valid: {name}")
        if candidate.suffix.lower() not in VIDEO_SUFFIXES:
            raise ValueError(
                f"ekstensi video tidak dikenal: {candidate.suffix or name}")
        root = self.videos_dir.resolve()
        path = (root / name).resolve()
        if path.parent != root:
            raise ValueError("video harus berada langsung di direktori daftar")
        if not path.is_file():
            raise ValueError(f"video tidak ada: {name}")
        return str(path)

    def _patch(self, **kwargs: Any) -> None:
        with self._lock:
            self._status.update(kwargs)

    def _bump_frame(self) -> tuple[int, float]:
        """Hitung frame ke-N + fps bergerak (jendela FPS_WINDOW_S)."""
        now = time.monotonic()
        with self._lock:
            self._frames += 1
            self._times.append(now)
            while self._times and now - self._times[0] > FPS_WINDOW_S:
                self._times.popleft()
            span = self._times[-1] - self._times[0]
            fps = ((len(self._times) - 1) / span
                   if len(self._times) > 1 and span > 0 else 0.0)
            return self._frames, fps

    def _loop(self) -> None:
        """Thread kamera: baca frame -> publish MJPEG -> tracker -> status.

        Satu frame buruk TIDAK boleh membunuh loop: EndOfVideo menandai
        `ended` (video habis, loop mati), error lain masuk `error` dan
        dicoba lagi setelah RETRY_S. Operator memperbaiki keadaan lewat
        dashboard (pilih video lain), bukan lewat restart proses.
        """
        while not self._stop.is_set():
            # Early exit: belum ada kamera -> jangan jeda (jaga error startup)
            if self.source.camera is None:
                self._stop.wait(RETRY_S)
                continue

            try:
                frame = self.source.read()
            except EndOfVideo:
                self._patch(available=False, ended=True, error="")
                self._stop.wait(RETRY_S)
                continue
            except RuntimeError as exc:
                self._patch(available=False, error=str(exc))
                self._stop.wait(RETRY_S)
                continue
            except Exception as exc:        # cv2 bisa melempar apa saja
                self._patch(available=False,
                            error=f"{type(exc).__name__}: {exc}")
                self._stop.wait(RETRY_S)
                continue

            self.hub.publish(frame)
            frames, fps = self._bump_frame()
            patch: dict[str, Any] = {
                "available": True, "ended": False, "error": "",
                "frames": frames, "fps": round(fps, 1),
            }
            try:
                obs = self.tracker.estimate(frame, self.calib, float(frames))
                patch.update({
                    "lane_error_m": obs.lane_error_m,
                    "heading_error_rad": obs.heading_error_rad,
                    "curvature_inv_m": obs.curvature_inv_m,
                    "confidence": obs.confidence,
                })
            except Exception as exc:        # tracker rusak != kamera rusak
                patch["tracker_error"] = f"{type(exc).__name__}: {exc}"
            debug = getattr(self.tracker, "last_debug", None) or {}
            found, total = debug.get("stations_found"), debug.get(
                "stations_total")
            patch["stations"] = (f"{found}/{total}"
                                 if found is not None and total is not None
                                 else "")
            patch["polarity"] = debug.get("polarity", "")
            self._patch(**patch)
