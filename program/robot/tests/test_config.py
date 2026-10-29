"""Tes config: default, penolakan key asing, validasi rentang, mode live."""
import json
from dataclasses import asdict, fields
from pathlib import Path

import pytest

from mizuro.config import Config


def test_defaults_are_safe():
    cfg = Config()
    assert cfg.dry_run is True
    assert cfg.stream_enabled is False
    assert cfg.stream_host == "127.0.0.1"
    assert cfg.loop_hz == 60.0
    assert cfg.run_timeout_s == 300.0
    assert cfg.max_speed_mps == 0.8
    assert cfg.steering_limit_rad == 0.55


def test_load_valid_file(tmp_path):
    path = tmp_path / "c.json"
    path.write_text(json.dumps({"dry_run": True, "loop_hz": 30}), encoding="utf-8")
    cfg = Config.load(path)
    assert cfg.loop_hz == 30.0


def test_unknown_key_rejected(tmp_path):
    path = tmp_path / "c.json"
    path.write_text(json.dumps({"dry_run": True, "ngawur": 1}), encoding="utf-8")
    with pytest.raises(ValueError, match="ngawur"):
        Config.load(path)


def test_missing_file(tmp_path):
    with pytest.raises(ValueError, match="tidak ada"):
        Config.load(tmp_path / "nope.json")


def test_invalid_json(tmp_path):
    path = tmp_path / "c.json"
    path.write_text("{bukan json", encoding="utf-8")
    with pytest.raises(ValueError, match="JSON"):
        Config.load(path)


def test_not_an_object(tmp_path):
    path = tmp_path / "c.json"
    path.write_text("[1, 2, 3]", encoding="utf-8")
    with pytest.raises(ValueError, match="objek"):
        Config.load(path)


def test_wrong_type_rejected(tmp_path):
    path = tmp_path / "c.json"
    path.write_text(json.dumps({"loop_hz": "enam puluh"}), encoding="utf-8")
    with pytest.raises(ValueError, match="tipe"):
        Config.load(path)


@pytest.mark.parametrize("patch", [
    {"loop_hz": 0},
    {"loop_hz": -1},
    {"max_speed_mps": 0.9},
    {"max_speed_mps": 0},
    {"steering_limit_rad": 0.6},
    {"lookahead_m": 0},
    {"max_lateral_accel": 0},
    {"max_inference_ms": 0},
    {"lane_lost_frames": 0},
    {"run_timeout_s": -1},
    {"confidence_threshold": 1.5},
    {"lane_min_confidence": -0.1},
    {"lane_roi_top": 1.0},
    {"lane_roi_top": 0},
    {"lane_polarity": "ungu"},
    {"motor_driver": "sihir"},
    {"image_width": 0},
    {"camera_fps": 0},
    {"serial_baud": 0},
    {"stream_port": 0},
    {"stream_port": 70000},
    {"stream_quality": 0},
    {"stream_quality": 101},
    {"stream_fps": 0},
    {"stream_width": -1},
    {"save_frames": True},          # butuh live
    {"dry_run": False},             # live tanpa serial_device/model_path
    {"mount_z_m": 0.0},
    {"mount_z_m": 1.5},
    {"mount_x_m": 5.0},
    {"mount_pitch_deg": 90.0},
    {"mount_yaw_deg": 60.0},
    {"mount_roll_deg": -60.0},
])
def test_invalid_values_rejected(patch):
    with pytest.raises(ValueError):
        Config(**{**asdict(Config()), **patch}).validate()


@pytest.mark.parametrize("patch", [
    {"dry_run": "ya"},               # string truthy = live diam-diam
    {"loop_hz": "60"},               # string lolos konstruktor, meledak di validasi
    {"stream_port": True},           # bool adalah int di Python — harus ditolak
    {"image_width": 1280.0},         # float untuk field integer
    {"stream_host": 8080},           # angka untuk field string
    {"mount_z_m": "0.16"},           # string untuk field float mount
])
def test_wrong_scalar_types_rejected(patch):
    with pytest.raises(ValueError, match="[Tt]ipe"):
        Config(**{**asdict(Config()), **patch}).validate()


def test_example_config_matches_dataclass_fields():
    """Contoh config harus memuat SETIAP field Config — kalau ada field baru
    dan contoh tidak diupdate, tes ini gagal (bukan drift diam-diam)."""
    example_path = Path(__file__).resolve().parent.parent / "robot.example.json"
    data = json.loads(example_path.read_text(encoding="utf-8"))
    assert set(data) == {f.name for f in fields(Config)}


def test_live_mode_requires_serial_and_model():
    with pytest.raises(ValueError, match="serial_device"):
        Config(dry_run=False, model_path="m.onnx").validate()
    with pytest.raises(ValueError, match="model_path"):
        Config(dry_run=False, serial_device="/dev/ttyTHS1").validate()
    cfg = Config(dry_run=False, serial_device="/dev/ttyTHS1",
                 model_path="m.onnx", save_frames=True)
    cfg.validate()  # lengkap: tidak melempar


def test_mount_defaults_match_arena3d():
    cfg = Config()
    assert cfg.mount_z_m == 0.16 and cfg.mount_pitch_deg == 12.0
    assert cfg.mount_x_m == 0.0 and cfg.mount_y_m == 0.0
    assert cfg.mount_yaw_deg == 0.0 and cfg.mount_roll_deg == 0.0


def test_mount_ranges_rejected():
    with pytest.raises(ValueError, match="mount_z_m"):
        Config(mount_z_m=0.0).validate()
    with pytest.raises(ValueError, match="mount_z_m"):
        Config(mount_z_m=2.0).validate()
    with pytest.raises(ValueError, match="mount_x_m"):
        Config(mount_x_m=5.0).validate()
    with pytest.raises(ValueError, match="mount_y_m"):
        Config(mount_y_m=-5.0).validate()
    with pytest.raises(ValueError, match="mount_pitch_deg"):
        Config(mount_pitch_deg=90.0).validate()
    with pytest.raises(ValueError, match="mount_yaw_deg"):
        Config(mount_yaw_deg=60.0).validate()
    with pytest.raises(ValueError, match="mount_roll_deg"):
        Config(mount_roll_deg=-60.0).validate()
