"""Tes tools/mcu_bench.py — pembentukan config bench + validasi argumen.

Loop closed-loop TIDAK dijalankan di sini (butuh file video); yang diuji
adalah kontrak keselamatan alat: kamera jadi file video, dry_run tidak
boleh bisa dimatikan lewat alat ini, dan driver ngawur ditolak.
"""
import importlib.util
from pathlib import Path

import pytest

_spec = importlib.util.spec_from_file_location(
    "mcu_bench",
    Path(__file__).resolve().parent.parent / "tools" / "mcu_bench.py")
bench = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(bench)

from mizuro.config import Config  # noqa: E402


def test_build_bench_config_uses_video_and_keeps_dry_run():
    base = Config(motor_driver="skid4_l298n")
    cfg = bench.build_bench_config(base, "video.mp4", None)
    assert cfg.camera_pipeline == "video.mp4"
    assert cfg.dry_run is True               # roda fisik tak bisa nyala
    assert cfg.motor_driver == "skid4_l298n"


def test_build_bench_config_driver_override():
    base = Config(motor_driver="skid4_l298n")
    cfg = bench.build_bench_config(base, "video.mp4", "external_esc")
    assert cfg.motor_driver == "external_esc"


def test_build_bench_config_rejects_unknown_driver():
    base = Config()
    with pytest.raises(ValueError, match="motor_driver"):
        bench.build_bench_config(base, "video.mp4", "sihir")


def test_main_requires_video(monkeypatch):
    monkeypatch.setattr("sys.argv", ["mcu_bench.py"])
    with pytest.raises(SystemExit):
        bench.main()
