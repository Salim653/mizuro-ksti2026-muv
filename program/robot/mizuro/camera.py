"""Kamera CSI (GStreamer) + file video untuk bench tanpa hardware.

Dua jebakan besar di Jetson:

1. cv2 HARUS dari JetPack (`/usr/bin/python3`) — di sana stack kamera CSI
   (`nvarguscamerasrc` + nvargus-daemon) benar-benar jalan. Catatan:
   wheel `opencv-python` di `.venv` repo ini MELAPORKAN `GStreamer: YES
   (1.20.3)` dan bahkan membundel plugin nvstreamer, jadi pemeriksaan
   build info di konstruktor TIDAK menangkapnya — yang gagal adalah
   `read()` ("No cameras available") karena tidak ada kamera CSI
   terpasang. Pemeriksaan build info hanya menangkap build yang
   sungguh-sungguh tanpa backend GStreamer.
2. Hanya SATU proses boleh memegang kamera CSI. `--stream-only` ada untuk
   debug; kalau pipeline tidak terbuka, penyebab paling sering adalah
   proses lain (dashboard/stream lama) masih pegang kamera.

SUMBER VIDEO (bench tanpa hardware): kalau `camera_pipeline` berisi path
file video (bukan pipeline GStreamer), kamera dibuka sebagai file biasa
TANPA CAP_GSTREAMER — jadi replay video jalan dengan cv2 pip `.venv` di
laptop, tanpa Jetson. Habisnya video = EndOfVideo (bukan error); dengan
`video_loop=True` file diputar ulang dari awal. Frame di-resize ke
ukuran kalibrasi kalau beda (video rekaman jarang persis resolusi
config) — beda dari sumber CSI yang ukurannya DITOLAK.

`opener` bisa disuntikkan untuk tes: fungsi () -> objek bergaya cv2.VideoCapture
dengan `isOpened()`, `read()`, `release()`. Tanpa suntikan, cv2 asli dipakai.

`CameraSource` membungkus `Camera` dengan penukar sumber saat runtime
(CSI <-> video) untuk dashboard bench — lihat `mizuro/bench_server.py`.
"""
from __future__ import annotations

import sys
import threading


class EndOfVideo(RuntimeError):
    """Frame habis: file video selesai (bukan kegagalan kamera)."""


def has_gstreamer(cv2) -> bool:
    """True kalau build cv2 ini punya backend GStreamer."""
    for line in cv2.getBuildInformation().splitlines():
        if "GStreamer:" in line:
            return line.split("GStreamer:")[1].strip().upper().startswith("YES")
    return False


def is_gstreamer_pipeline(pipeline: str) -> bool:
    """True kalau `pipeline` adalah pipeline GStreamer, bukan path file.

    Heuristik: pipeline GStreamer selalu punya pemisah elemen '!'
    (`nvarguscamerasrc ! ... ! appsink`); path file/URL tidak. Kalau
    butuh pipeline custom, PASTIKAN ada '!' — tanpa itu string
    diperlakukan sebagai path file dan dibuka tanpa CAP_GSTREAMER.
    """
    if not pipeline:
        return False
    return ("!" in pipeline or "appsink" in pipeline
            or "nvarguscamerasrc" in pipeline)


def nvargus_pipeline(width: int, height: int, fps: int = 30,
                     sensor: int = 0) -> str:
    """Pipeline CSI bawaan (Pi camera module v2 / IMX219 di Jetson)."""
    return (
        f"nvarguscamerasrc sensor-id={sensor} ! "
        f"video/x-raw(memory:NVMM),width={width},height={height},"
        f"framerate={fps}/1,format=NV12 ! "
        "nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! "
        "video/x-raw,format=BGR ! appsink drop=1 max-buffers=1 sync=false"
    )


class Camera:
    """Pembungkus tipis cv2.VideoCapture dengan validasi ukuran frame.

    Dua jenis sumber:

    - GStreamer (CSI bawaan atau pipeline custom dengan '!'): ukuran
      frame yang berbeda dari kalibrasi = bug diam-diam (homography
      salah posisi), jadi read() menolak, bukan mengembalikan frame aneh.
    - file video (path tanpa '!'): bench tanpa hardware. Frame di-resize
      ke ukuran kalibrasi kalau beda, dan habisnya video = EndOfVideo.
      Tidak butuh backend GStreamer.
    """

    def __init__(self, pipeline: str, width: int, height: int, *,
                 opener=None, undistort=None, video_loop: bool = False) -> None:
        import cv2
        self.cv2 = cv2
        # Path file video tidak butuh GStreamer; CSI/pipeline butuh.
        self.is_file = bool(pipeline) and not is_gstreamer_pipeline(pipeline)
        if not self.is_file and not has_gstreamer(cv2):
            raise RuntimeError(
                f"cv2 {cv2.__version__} at {sys.executable} has no GStreamer "
                "backend; run this with /usr/bin/python3 (JetPack OpenCV), "
                "not the repo .venv")
        self.pipeline = pipeline or nvargus_pipeline(width, height)
        self.width, self.height = width, height
        self._opener = opener
        # undistort = (K 3x3, dist) dari kalibrasi terukur. Frame
        # direktifikasi sekali di sini supaya semua hilir (tracker,
        # homography) melihat gambar pinhole dengan K yang sama.
        self._undistort = undistort
        self.video_loop = video_loop
        self.cap = self._open()

    def _open(self):
        if self._opener is not None:
            cap = self._opener(self.pipeline)
        elif self.is_file:
            # File video: backend biasa (ffmpeg), TANPA CAP_GSTREAMER —
            # jalan dengan cv2 pip .venv di laptop.
            cap = self.cv2.VideoCapture(self.pipeline)
        else:
            cap = self.cv2.VideoCapture(self.pipeline, self.cv2.CAP_GSTREAMER)
        if cap is None or not cap.isOpened():
            if self.is_file:
                raise RuntimeError(
                    f"video gagal dibuka: {self.pipeline}. "
                    "File tidak ada atau formatnya tidak didukung build "
                    "cv2 ini.")
            raise RuntimeError(
                f"kamera gagal dibuka: {self.pipeline}. "
                "Penyebab paling sering: proses lain masih memegang kamera "
                "CSI (stream/dashboard lama), atau pipeline salah.")
        return cap

    @property
    def source_fps(self) -> float:
        """FPS sumber (video file); 0.0 kalau tidak diketahui."""
        if self.cap is None:
            return 0.0
        try:
            return float(self.cap.get(self.cv2.CAP_PROP_FPS) or 0.0)
        except Exception:
            return 0.0

    def _restart(self) -> bool:
        """Putar ulang video dari frame pertama (mode loop)."""
        if self.cap is None:
            return False
        try:
            self.cap.set(self.cv2.CAP_PROP_POS_FRAMES, 0.0)
            return True
        except Exception:
            return False

    def read(self):
        """Satu frame BGR (height, width, 3). Melempar kalau gagal.

        Kalau kalibrasi punya koefisien distorsi, frame di-undistort
        dulu (ukuran tidak berubah — K yang sama dipakai tracker).

        File video: habis -> EndOfVideo (atau putar ulang kalau
        `video_loop`); ukuran beda -> di-resize ke ukuran kalibrasi.
        """
        ok, frame = self.cap.read()
        if not ok or frame is None:
            if self.is_file:
                if self.video_loop and self._restart():
                    ok, frame = self.cap.read()
                if not ok or frame is None:
                    raise EndOfVideo(
                        f"pembacaan frame gagal: video selesai "
                        f"({self.pipeline})")
            else:
                raise RuntimeError("pembacaan frame kamera gagal")
        if self.is_file:
            # Replay: ukuran video rekaman jarang persis resolusi config.
            if frame.shape[1] != self.width or frame.shape[0] != self.height:
                shrink = (frame.shape[1] > self.width
                          or frame.shape[0] > self.height)
                frame = self.cv2.resize(
                    frame, (self.width, self.height),
                    interpolation=self.cv2.INTER_AREA if shrink
                    else self.cv2.INTER_LINEAR)
        elif frame.shape[1] != self.width or frame.shape[0] != self.height:
            raise RuntimeError(
                f"ukuran frame tak terduga {frame.shape[1]}x{frame.shape[0]}, "
                f"harap {self.width}x{self.height}")
        if self._undistort is not None:
            k_matrix, dist = self._undistort
            frame = self.cv2.undistort(frame, k_matrix, dist)
        return frame

    def close(self) -> None:
        if self.cap is not None:
            self.cap.release()
            self.cap = None

    def __enter__(self) -> "Camera":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    @classmethod
    def from_config(cls, config, *, opener=None,
                    undistort=None, video_loop: bool = False) -> "Camera":
        """Bangun dari Config.

        `camera_pipeline` kosong = CSI bawaan; mengandung '!' = pipeline
        GStreamer; selain itu = path file video (bench tanpa hardware).
        `video_loop` = putar ulang file kalau habis (buat run panjang).
        """
        return cls(config.camera_pipeline, config.image_width,
                   config.image_height, opener=opener, undistort=undistort,
                   video_loop=video_loop)


class CameraSource:
    """Penukar sumber kamera SAAT RUNTIME (CSI <-> file video).

    Dipakai dashboard bench (`main.py --bench`): operator menukar sumber
    dari browser tanpa restart proses. Sumber baru dibuka DULU dan baru
    menggantikan yang lama kalau berhasil — kegagalan buka (file hilang,
    kamera dipakai proses lain) tidak pernah meninggalkan keadaan
    "setengah tukar": sumber lama tetap jalan.

    Thread-safe dengan kontrak khusus: satu thread loop kamera memanggil
    `read()`, thread HTTP memanggil `select()`. Lock dipegang SELAMA
    pembacaan frame supaya kamera tidak di-`release()` tepat saat
    `read()` berjalan di thread lain — cv2 tidak menjamin aman untuk
    itu, dan release di tengah read adalah cara tercepat mendapatkan
    segfault. Konsekuensinya: `select()` menunggu frame yang sedang
    dibaca selesai (beberapa ms), tidak pernah sebaliknya.
    """

    def __init__(self, width: int, height: int, *, opener=None,
                 undistort=None, video_loop: bool = False) -> None:
        self.width, self.height = width, height
        self._opener = opener
        self._undistort = undistort
        self.video_loop = video_loop
        self._camera: Camera | None = None
        self._lock = threading.Lock()
        self.switches = 0

    @property
    def camera(self) -> Camera | None:
        """Kamera aktif (None = belum ada / sudah ditutup)."""
        with self._lock:
            return self._camera

    @property
    def pipeline(self) -> str:
        cam = self.camera
        return cam.pipeline if cam is not None else ""

    @property
    def is_file(self) -> bool:
        cam = self.camera
        return bool(cam is not None and cam.is_file)

    @property
    def source_fps(self) -> float:
        cam = self.camera
        return cam.source_fps if cam is not None else 0.0

    def select(self, pipeline: str) -> Camera:
        """Tukar ke `pipeline`; return kamera yang baru.

        Melempar (RuntimeError dari Camera) kalau sumber gagal dibuka —
        sumber lama TETAP aktif. `pipeline` kosong = CSI bawaan.
        """
        # Buka dulu di LUAR lock: kamera lama tetap bisa melayani
        # read() selama pembukaan (yang bisa lambat) berjalan.
        camera = Camera(pipeline, self.width, self.height,
                        opener=self._opener, undistort=self._undistort,
                        video_loop=self.video_loop)
        with self._lock:
            old, self._camera = self._camera, camera
            self.switches += 1
        if old is not None:
            # Aman: tidak ada read() yang bisa menyentuh `old` lagi —
            # pertukaran tadi terjadi di bawah lock yang sama dengan
            # lock yang dipegang read().
            old.close()
        return camera

    def read(self):
        """Satu frame BGR dari sumber aktif. Melempar kalau gagal."""
        with self._lock:
            if self._camera is None:
                raise RuntimeError("belum ada sumber kamera dipilih")
            return self._camera.read()

    def close(self) -> None:
        with self._lock:
            camera, self._camera = self._camera, None
        if camera is not None:
            camera.close()

    def describe(self) -> dict:
        """Ringkasan sumber aktif buat status dashboard (tanpa lock lama)."""
        cam = self.camera
        if cam is None:
            return {"source": "none", "pipeline": "", "is_file": False,
                    "source_fps": 0.0}
        return {"source": "video" if cam.is_file else "csi",
                "pipeline": cam.pipeline, "is_file": cam.is_file,
                "source_fps": cam.source_fps}

    def __enter__(self) -> "CameraSource":
        return self

    def __exit__(self, *exc) -> None:
        self.close()
