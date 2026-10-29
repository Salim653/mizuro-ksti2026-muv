"""Konfigurasi runtime robot.

Config dimuat dari file JSON. Key yang tidak dikenal DITOLAK supaya salah
ketik tidak pernah diterapkan diam-diam, dan setiap rentang divalidasi saat
load. Batas kecepatan/kemudi berasal dari protokol serial (protocol.py) dan
tidak boleh dilebihi tanpa mengubah firmware MCU juga.
"""
from __future__ import annotations

import json
from dataclasses import dataclass, fields
from pathlib import Path

from .protocol import DEFAULT_BAUD, MAX_SPEED_MPS, MAX_STEER_RAD

MOTOR_DRIVERS = {"external_esc", "mcu_hbridge"}
LANE_POLARITIES = {"auto", "light_on_dark", "dark_on_light"}


@dataclass(frozen=True)
class Config:
    """Seluruh setelan runtime. Default-nya sengaja paling aman (dry_run)."""

    # --- output run ---
    output_dir: str = "runs"

    # --- serial ke MCU bodi ---
    serial_device: str = ""
    serial_baud: int = DEFAULT_BAUD

    # --- kamera ---
    # camera_pipeline kosong = pipeline CSI bawaan (nvarguscamerasrc).
    camera_pipeline: str = ""
    image_width: int = 1280
    image_height: int = 720
    camera_fps: int = 30

    # --- persepsi lajur (Fase 1) ---
    lane_polarity: str = "auto"      # auto | light_on_dark | dark_on_light
    lane_roi_top: float = 0.35       # batas atas ROI, fraksi tinggi frame
    lane_min_confidence: float = 0.3
    homography_path: str = ""        # kalibrasi kamera -> bidang tanah

    # --- detektor semantik (Fase 2) ---
    model_path: str = ""
    confidence_threshold: float = 0.5

    # --- kontrol ---
    max_speed_mps: float = MAX_SPEED_MPS
    steering_limit_rad: float = MAX_STEER_RAD
    lookahead_m: float = 0.6
    heading_gain: float = 0.8
    lateral_gain: float = 1.2
    max_lateral_accel: float = 3.0

    # --- loop & keselamatan ---
    loop_hz: float = 60.0
    max_inference_ms: float = 150.0
    lane_lost_frames: int = 3
    run_timeout_s: float = 300.0     # batas waktu percobaan lomba

    # --- mode ---
    dry_run: bool = True
    save_frames: bool = False
    motor_driver: str = "external_esc"

    # --- stream kamera (bench/pit saja; MATIKAN saat lomba) ---
    stream_enabled: bool = False
    stream_host: str = "127.0.0.1"   # 0.0.0.0 hanya untuk jaringan pit tepercaya
    stream_port: int = 8080
    stream_fps: float = 15.0
    stream_quality: int = 75
    stream_width: int = 640          # 0 = resolusi kamera penuh
    stream_height: int = 480

    @classmethod
    def load(cls, path: Path) -> "Config":
        """Muat + validasi config dari file JSON."""
        path = Path(path)
        try:
            text = path.read_text(encoding="utf-8")
        except FileNotFoundError as exc:
            raise ValueError(f"file config tidak ada: {path}") from exc
        try:
            data = json.loads(text)
        except json.JSONDecodeError as exc:
            raise ValueError(f"config bukan JSON valid: {path} ({exc})") from exc
        if not isinstance(data, dict):
            raise ValueError("config harus berupa objek JSON")
        known = {f.name for f in fields(cls)}
        unknown = sorted(set(data) - known)
        if unknown:
            raise ValueError(f"key config tidak dikenal: {unknown}")
        try:
            cfg = cls(**data)
            cfg.validate()
        except TypeError as exc:
            # dataclass tidak memeriksa tipe; salah tipe muncul sebagai
            # TypeError saat validasi — ubah menjadi ValueError yang jelas.
            raise ValueError(f"tipe key config salah: {exc}") from exc
        return cfg

    def validate(self) -> None:
        self._check_types()
        if not 0.0 <= self.confidence_threshold <= 1.0:
            raise ValueError("confidence_threshold harus dalam [0, 1]")
        if not 0.0 <= self.lane_min_confidence <= 1.0:
            raise ValueError("lane_min_confidence harus dalam [0, 1]")
        if not 0.0 < self.lane_roi_top < 1.0:
            raise ValueError("lane_roi_top harus fraksi (0, 1)")
        if self.lane_polarity not in LANE_POLARITIES:
            raise ValueError(
                f"lane_polarity harus salah satu dari {sorted(LANE_POLARITIES)}")
        if self.image_width <= 0 or self.image_height <= 0 or self.camera_fps <= 0:
            raise ValueError("dimensi dan fps kamera harus positif")
        if self.serial_baud <= 0:
            raise ValueError("serial_baud harus positif")
        if not 0.0 < self.max_speed_mps <= MAX_SPEED_MPS:
            raise ValueError(
                f"max_speed_mps harus dalam (0, {MAX_SPEED_MPS}] — batas protokol")
        if not 0.0 < self.steering_limit_rad <= MAX_STEER_RAD:
            raise ValueError(
                f"steering_limit_rad harus dalam (0, {MAX_STEER_RAD}] — batas protokol")
        if self.lookahead_m <= 0:
            raise ValueError("lookahead_m harus positif")
        if self.heading_gain < 0 or self.lateral_gain < 0:
            raise ValueError("gain kontrol tidak boleh negatif")
        if self.max_lateral_accel <= 0:
            raise ValueError("max_lateral_accel harus positif")
        if self.loop_hz <= 0:
            raise ValueError("loop_hz harus positif")
        if self.max_inference_ms <= 0:
            raise ValueError("max_inference_ms harus positif")
        if self.lane_lost_frames < 1:
            raise ValueError("lane_lost_frames minimal 1")
        if self.run_timeout_s <= 0:
            raise ValueError("run_timeout_s harus positif")
        if self.motor_driver not in MOTOR_DRIVERS:
            raise ValueError(
                f"motor_driver harus salah satu dari {sorted(MOTOR_DRIVERS)}")
        if not 1 <= self.stream_port <= 65535:
            raise ValueError("stream_port harus dalam [1, 65535]")
        if self.stream_fps <= 0 or not 1 <= self.stream_quality <= 100:
            raise ValueError("stream_fps harus positif dan stream_quality dalam [1, 100]")
        if self.stream_width < 0 or self.stream_height < 0:
            raise ValueError("dimensi stream harus >= 0 (0 = resolusi kamera)")
        if self.save_frames and self.dry_run:
            raise ValueError("save_frames butuh capture live (matikan dry_run)")
        if not self.dry_run:
            if not self.serial_device:
                raise ValueError("mode live mensyaratkan serial_device")
            if not self.model_path:
                raise ValueError("mode live mensyaratkan model_path")

    def _check_types(self) -> None:
        """Dataclass tidak memeriksa tipe; salah tipe bisa mengubah arti
        (mis. dry_run string dianggap truthy = live). Periksa eksplisit."""
        bools = ("dry_run", "save_frames")
        strs = ("output_dir", "serial_device", "camera_pipeline",
                "lane_polarity", "homography_path", "model_path",
                "motor_driver", "stream_host")
        ints = ("serial_baud", "image_width", "image_height", "camera_fps",
                "lane_lost_frames", "stream_port", "stream_quality",
                "stream_width", "stream_height")
        floats = ("confidence_threshold", "lane_min_confidence", "lane_roi_top",
                  "max_speed_mps", "steering_limit_rad", "lookahead_m",
                  "heading_gain", "lateral_gain", "max_lateral_accel",
                  "loop_hz", "max_inference_ms", "run_timeout_s", "stream_fps")
        for name in bools:
            if not isinstance(getattr(self, name), bool):
                raise ValueError(f"tipe salah: {name} harus bool")
        for name in strs:
            if not isinstance(getattr(self, name), str):
                raise ValueError(f"tipe salah: {name} harus string")
        for name in ints:
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int):
                raise ValueError(f"tipe salah: {name} harus integer")
        for name in floats:
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"tipe salah: {name} harus angka")
        if not self.output_dir.strip():
            raise ValueError("output_dir tidak boleh kosong")
