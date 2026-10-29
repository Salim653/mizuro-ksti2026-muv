"""Kamera CSI (GStreamer) dengan penjagaan yang gagal keras.

Dua jebakan besar di Jetson:

1. cv2 HARUS dari JetPack (`/usr/bin/python3`) — punya backend GStreamer.
   cv2 pip (`opencv-python`, termasuk yang di `.venv` repo ini) TIDAK punya,
   jadi `VideoCapture(CAP_GSTREAMER)` gagal dengan pesan membingungkan.
   Konstruktor memeriksa build info dan melempar pesai eksplisit.
2. Hanya SATU proses boleh memegang kamera CSI. `--stream-only` ada untuk
   debug; kalau pipeline tidak terbuka, penyebab paling sering adalah
   proses lain (dashboard/stream lama) masih pegang kamera.

`opener` bisa disuntikkan untuk tes: fungsi () -> objek bergaya cv2.VideoCapture
dengan `isOpened()`, `read()`, `release()`. Tanpa suntikan, cv2 asli dipakai.
"""
from __future__ import annotations

import sys


def has_gstreamer(cv2) -> bool:
    """True kalau build cv2 ini punya backend GStreamer."""
    for line in cv2.getBuildInformation().splitlines():
        if "GStreamer:" in line:
            return line.split("GStreamer:")[1].strip().upper().startswith("YES")
    return False


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

    Ukuran frame yang berbeda dari kalibrasi = bug diam-diam (homography
    salah posisi), jadi read() menolak, bukan mengembalikan frame aneh.
    """

    def __init__(self, pipeline: str, width: int, height: int, *,
                 opener=None, undistort=None) -> None:
        import cv2
        self.cv2 = cv2
        if not has_gstreamer(cv2):
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
        self.cap = self._open()

    def _open(self):
        if self._opener is not None:
            cap = self._opener(self.pipeline)
        else:
            cap = self.cv2.VideoCapture(self.pipeline, self.cv2.CAP_GSTREAMER)
        if cap is None or not cap.isOpened():
            raise RuntimeError(
                f"kamera gagal dibuka: {self.pipeline}. "
                "Penyebab paling sering: proses lain masih memegang kamera "
                "CSI (stream/dashboard lama), atau pipeline salah.")
        return cap

    def read(self):
        """Satu frame BGR (height, width, 3). Melempar kalau gagal.

        Kalau kalibrasi punya koefisien distorsi, frame di-undistort
        dulu (ukuran tidak berubah — K yang sama dipakai tracker).
        """
        ok, frame = self.cap.read()
        if not ok or frame is None:
            raise RuntimeError("pembacaan frame kamera gagal")
        if frame.shape[1] != self.width or frame.shape[0] != self.height:
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
                    undistort=None) -> "Camera":
        """Bangun dari Config: pipeline kosong = CSI bawaan.

        `undistort` = (K, dist) opsional dari kalibrasi terukur.
        """
        return cls(config.camera_pipeline, config.image_width,
                   config.image_height, opener=opener, undistort=undistort)
