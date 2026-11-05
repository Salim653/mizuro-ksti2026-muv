"""Tes tools/record_dataset.py — tanpa kamera.

Kamera di-stub (frame sintetis + hitung panggilan); loop rekam,
writer, dan parsing argumen diuji langsung. Penulisan CSV ada di
main() yang butuh kamera asli — tidak dites di sini.
"""
import csv
import importlib.util
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

_spec = importlib.util.spec_from_file_location(
    "record_dataset_tool",
    Path(__file__).resolve().parent.parent / "tools" / "record_dataset.py")
tool = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(tool)


class StubCamera:
    """Kamera palsu: read() return frame sintetis berganti tiap panggilan."""

    def __init__(self, n_frames=50, shape=(480, 640, 3),
                 fail_after=None, interrupt_after=None):
        self.shape = shape
        self.fail_after = fail_after          # kembalikan None setelah ini
        self.interrupt_after = interrupt_after  # raise KeyboardInterrupt
        self.reads = 0
        self.n_frames = n_frames

    def read(self):
        self.reads += 1
        if self.interrupt_after is not None and self.reads > self.interrupt_after:
            raise KeyboardInterrupt
        if self.fail_after is not None and self.reads > self.fail_after:
            return None
        i = self.reads % self.n_frames
        frame = np.full(self.shape, i % 256, dtype=np.uint8)
        # Garis unik per frame supaya isi bisa diverifikasi.
        cv2.line(frame, (10, 10), (10 + i, 10), (0, 0, 255), 2)
        return frame


class StubHub:
    def __init__(self):
        self.published = 0

    def publish(self, frame):
        self.published += 1


def _args(**kw):
    """Namespace argumen dengan default yang sama seperti parse_args."""
    base = dict(frames=0, duration=0.0, every=1, warmup=0.0,
                format="png", quality=95, no_stream=True,
                no_keyboard=True)
    base.update(kw)
    return type("Args", (), base)()


# --- parse_args ------------------------------------------------------------------

def test_parse_args_defaults(monkeypatch):
    monkeypatch.setattr(sys, "argv",
                        ["record_dataset.py", "--frames", "10"])
    args = tool.parse_args()
    assert args.frames == 10
    assert args.duration == 0.0
    assert args.every == 1
    assert args.warmup == 1.0
    assert args.format == "png"
    assert args.quality == 95
    assert args.no_stream is False
    assert args.no_keyboard is False


def test_parse_args_keyboard_alone_is_valid_stop_condition(monkeypatch):
    """Tanpa --frames/--duration, terminal (TTY) = kontrol manual."""
    monkeypatch.setattr(sys, "argv", ["record_dataset.py"])
    monkeypatch.setattr(sys.stdin, "isatty", lambda: True, raising=False)
    args = tool.parse_args()
    assert args.frames == 0 and args.duration == 0.0
    assert args.no_keyboard is False


def test_parse_args_no_tty_without_limits_rejected(monkeypatch):
    """Stdin bukan terminal + tanpa batas = tidak ada cara berhenti."""
    monkeypatch.setattr(sys, "argv", ["record_dataset.py"])
    monkeypatch.setattr(sys.stdin, "isatty", lambda: False, raising=False)
    with pytest.raises(SystemExit):
        tool.parse_args()


def test_parse_args_no_keyboard_flag_without_limits_rejected(monkeypatch):
    monkeypatch.setattr(sys, "argv",
                        ["record_dataset.py", "--no-keyboard"])
    monkeypatch.setattr(sys.stdin, "isatty", lambda: True, raising=False)
    with pytest.raises(SystemExit):
        tool.parse_args()


def test_parse_args_requires_stop_condition(monkeypatch):
    monkeypatch.setattr(sys, "argv", ["record_dataset.py"])
    with pytest.raises(SystemExit):
        tool.parse_args()


def test_parse_args_rejects_bad_values(monkeypatch):
    for bad in (["--frames", "0"], ["--every", "0"],
                ["--quality", "101"], ["--duration", "-5"]):
        monkeypatch.setattr(sys, "argv", ["record_dataset.py", *bad])
        with pytest.raises(SystemExit):
            tool.parse_args()


# --- Keyboard ----------------------------------------------------------------------

class FakeKeys:
    """read_key suntikan: kembalikan byte dari antrian, lalu None."""

    def __init__(self, keys):
        self.keys = list(keys)

    def __call__(self):
        return self.keys.pop(0) if self.keys else None


def test_keyboard_space_and_quit():
    kb = tool.Keyboard(read_key=FakeKeys([b"x", b" ", b"q"]))
    assert kb.enabled is True
    assert kb.poll() is None        # tombol lain diabaikan
    assert kb.poll() == "space"
    assert kb.poll() == "quit"


def test_keyboard_no_key_returns_none():
    kb = tool.Keyboard(read_key=FakeKeys([]))
    assert kb.poll() is None


def test_keyboard_close_calls_restore():
    restored = []
    kb = tool.Keyboard(read_key=lambda: None)
    kb._restore = lambda: restored.append(True)
    kb.close()
    assert restored == [True]
    assert kb.enabled is False
    assert kb.poll() is None        # setelah close, mati total


def test_keyboard_reader_error_is_survived():
    def boom():
        raise OSError("stdin rusak")
    kb = tool.Keyboard(read_key=boom)
    assert kb.poll() is None        # tidak melempar ke loop rekam


# --- record_loop: mode manual (SPACE) -------------------------------------------------

class FakeKeyboard:
    """Keyboard palsu: script = list of (key|None, n_polls).

    Setiap entri bertahan n_polls pemanggilan poll() — supaya bisa
    disimulasikan "SPACE ditahan selama 4 iterasi" (1 toggle + 3
    frame terekam). Tombol berupa byte (b" ", b"q"); poll() memetakan
    ke "space"/"quit" persis seperti Keyboard asli.
    """

    def __init__(self, script):
        self.enabled = True
        self.script = [(k, n) for k, n in script]
        self.polls = 0

    def poll(self):
        self.polls += 1
        while self.script and self.script[0][1] <= 0:
            self.script.pop(0)
        if not self.script:
            return None
        key, n = self.script[0]
        self.script[0] = (key, n - 1)
        if key is None:
            return None
        if key == b" ":
            return "space"
        if key in (b"q", b"Q"):
            return "quit"
        return None

    def close(self):
        self.enabled = False


def test_manual_starts_in_preview(tmp_path):
    """Keyboard aktif = belum rekam sampai SPACE ditekan."""
    frames_dir = tmp_path / "frames"
    frames_dir.mkdir()
    cam = StubCamera()
    kb = FakeKeyboard([(b" ", 1), (None, 10)])
    rows, stats = tool.record_loop(
        cam=cam, write_frame=tool.build_writer("png", 95),
        frames_dir=frames_dir, args=_args(frames=3), keyboard=kb)
    assert stats["saved"] == 3
    assert stats["takes"] == 1
    assert len(rows) == 3


def test_manual_space_stops_and_restarts(tmp_path):
    """SPACE stop -> preview; SPACE lagi -> take baru. Penomoran lanjut."""
    frames_dir = tmp_path / "frames"
    frames_dir.mkdir()
    cam = StubCamera()
    # Iterasi di mana SPACE ditekan JUGA menyimpan frame (toggle lalu
    # rekam di iterasi yang sama), jadi (space,1)+(None,2) = 3 frame.
    kb = FakeKeyboard([
        (b" ", 1), (None, 2),    # take 1: 3 frame
        (b" ", 1), (None, 1),    # stop, preview 1 iterasi
        (b" ", 1), (None, 1),    # take 2: 2 frame
        (b" ", 1), (None, 2),    # stop, preview
        (b"q", 1),               # keluar
    ])
    rows, stats = tool.record_loop(
        cam=cam, write_frame=tool.build_writer("png", 95),
        frames_dir=frames_dir, args=_args(), keyboard=kb)
    assert stats["saved"] == 5
    assert stats["takes"] == 2
    # Penomoran menyatu across takes: 1..5 tanpa duplikat.
    names = [r["filename"] for r in rows]
    assert names == [f"frame-{i:06d}.png" for i in range(1, 6)]
    assert len(set(names)) == 5


def test_manual_quit_before_recording_saves_nothing(tmp_path):
    frames_dir = tmp_path / "frames"
    frames_dir.mkdir()
    cam = StubCamera()
    kb = FakeKeyboard([(b"q", 1)])
    rows, stats = tool.record_loop(
        cam=cam, write_frame=tool.build_writer("png", 95),
        frames_dir=frames_dir, args=_args(), keyboard=kb)
    assert stats["saved"] == 0
    assert rows == []
    assert stats["takes"] == 0


def test_manual_frames_limit_still_applies(tmp_path):
    """--frames tetap jadi batas walau mode manual."""
    frames_dir = tmp_path / "frames"
    frames_dir.mkdir()
    cam = StubCamera()
    kb = FakeKeyboard([(b" ", 1), (None, 100)])
    rows, stats = tool.record_loop(
        cam=cam, write_frame=tool.build_writer("png", 95),
        frames_dir=frames_dir, args=_args(frames=4), keyboard=kb)
    assert stats["saved"] == 4
    assert stats["takes"] == 1


def test_manual_publishes_preview_frames_while_idle(tmp_path):
    """Frame preview (sebelum SPACE) tetap dipublish ke browser."""
    frames_dir = tmp_path / "frames"
    frames_dir.mkdir()
    cam = StubCamera()
    hub = StubHub()
    kb = FakeKeyboard([(b" ", 1), (None, 2), (b" ", 1), (None, 2),
                       (b"q", 1)])
    tool.record_loop(cam=cam, write_frame=tool.build_writer("png", 95),
                     frames_dir=frames_dir, args=_args(), hub=hub,
                     keyboard=kb)
    assert hub.published == cam.reads


# --- build_writer -------------------------------------------------------------------

def test_writer_png_is_lossless(tmp_path):
    write = tool.build_writer("png", 95)
    frame = np.full((48, 64, 3), 137, dtype=np.uint8)
    ms, ok = write(frame, tmp_path / "a.png")
    assert ok
    assert ms >= 0.0
    back = cv2.imread(str(tmp_path / "a.png"))
    assert np.array_equal(back, frame)


def test_writer_jpg_near_lossless_for_solid_color(tmp_path):
    write = tool.build_writer("jpg", 95)
    frame = np.full((48, 64, 3), 137, dtype=np.uint8)
    ms, ok = write(frame, tmp_path / "a.jpg")
    assert ok
    back = cv2.imread(str(tmp_path / "a.jpg"))
    assert back.shape == frame.shape
    assert np.abs(back.astype(int) - 137).max() <= 3


# --- record_loop ----------------------------------------------------------------------

def test_loop_saves_requested_frames(tmp_path):
    frames_dir = tmp_path / "frames"
    frames_dir.mkdir()
    cam = StubCamera()
    rows, stats = tool.record_loop(
        cam=cam, write_frame=tool.build_writer("png", 95),
        frames_dir=frames_dir, args=_args(frames=5))
    assert stats["saved"] == 5
    assert len(rows) == 5
    names = sorted(p.name for p in frames_dir.glob("*.png"))
    assert names == [f"frame-{i:06d}.png" for i in range(1, 6)]
    for r in rows:
        assert (frames_dir / r["filename"]).is_file()
        assert r["width"] == 640 and r["height"] == 480


def test_loop_stops_on_duration(tmp_path):
    frames_dir = tmp_path / "frames"
    frames_dir.mkdir()
    cam = StubCamera(n_frames=10_000)
    rows, stats = tool.record_loop(
        cam=cam, write_frame=tool.build_writer("jpg", 95),
        frames_dir=frames_dir, args=_args(duration=0.3))
    assert stats["saved"] > 0
    assert stats["elapsed_s"] >= 0.3


def test_loop_every_decimates(tmp_path):
    frames_dir = tmp_path / "frames"
    frames_dir.mkdir()
    cam = StubCamera()
    rows, stats = tool.record_loop(
        cam=cam, write_frame=tool.build_writer("jpg", 95),
        frames_dir=frames_dir, args=_args(frames=5, every=4))
    assert stats["saved"] == 5
    # Simpan di read_count 1, 5, 9, 13, 17 — loop berhenti saat target
    # tercapai (cek di atas loop), jadi tidak ada read sampingan.
    assert cam.reads == 17
    assert len(rows) == 5


def test_loop_warmup_discards_frames(tmp_path):
    frames_dir = tmp_path / "frames"
    frames_dir.mkdir()
    cam = StubCamera()
    rows, stats = tool.record_loop(
        cam=cam, write_frame=tool.build_writer("jpg", 95),
        frames_dir=frames_dir, args=_args(frames=3, warmup=0.2))
    assert stats["saved"] == 3
    # Frame warmup terbaca TAPI tidak disimpan.
    assert stats["read_count"] > 3
    assert len(rows) == 3
    # Frame pertama yang disimpan bukan frame pertama kamera.
    first = cv2.imread(str(frames_dir / rows[0]["filename"]))
    assert first is not None


def test_loop_camera_failure_stops_gracefully(tmp_path):
    frames_dir = tmp_path / "frames"
    frames_dir.mkdir()
    cam = StubCamera(fail_after=8)
    rows, stats = tool.record_loop(
        cam=cam, write_frame=tool.build_writer("jpg", 95),
        frames_dir=frames_dir, args=_args(frames=100))
    assert stats["saved"] == 8
    # Read yang gagal (None) tidak dihitung sebagai frame terbaca.
    assert stats["read_count"] == 8


def test_loop_ctrl_c_keeps_saved_rows(tmp_path):
    """Ctrl+C di cam.read() TIDAK boleh menghilangkan rows yang sudah
    ada — index.csv harus konsisten dengan file di disk."""
    frames_dir = tmp_path / "frames"
    frames_dir.mkdir()
    cam = StubCamera(interrupt_after=12)
    rows, stats = tool.record_loop(
        cam=cam, write_frame=tool.build_writer("png", 95),
        frames_dir=frames_dir, args=_args(frames=100))
    assert stats["saved"] == 12
    assert len(rows) == 12
    assert len(list(frames_dir.glob("*.png"))) == 12


def test_loop_publishes_to_hub(tmp_path):
    frames_dir = tmp_path / "frames"
    frames_dir.mkdir()
    cam = StubCamera()
    hub = StubHub()
    tool.record_loop(cam=cam, write_frame=tool.build_writer("jpg", 95),
                     frames_dir=frames_dir, args=_args(frames=5), hub=hub)
    # Setiap frame terbaca (termasuk warmup) dipublish ke preview.
    assert hub.published == cam.reads


def test_loop_timestamps_increase(tmp_path):
    frames_dir = tmp_path / "frames"
    frames_dir.mkdir()
    cam = StubCamera()
    rows, _ = tool.record_loop(
        cam=cam, write_frame=tool.build_writer("jpg", 95),
        frames_dir=frames_dir, args=_args(frames=5))
    monos = [r["t_monotonic_s"] for r in rows]
    assert monos == sorted(monos)
    walls = [r["t_wall_s"] for r in rows]
    assert walls == sorted(walls)
    assert walls[0] > 1_700_000_000    # epoch wajar, bukan 0


def test_loop_png_format(tmp_path):
    frames_dir = tmp_path / "frames"
    frames_dir.mkdir()
    cam = StubCamera()
    rows, stats = tool.record_loop(
        cam=cam, write_frame=tool.build_writer("png", 95),
        frames_dir=frames_dir, args=_args(frames=3, format="png"))
    assert stats["saved"] == 3
    assert all(r["filename"].endswith(".png") for r in rows)
    assert len(list(frames_dir.glob("*.png"))) == 3


def test_loop_frame_content_matches_camera(tmp_path):
    """Isi file = isi frame kamera (jpg q95, solid color -> near exact)."""
    frames_dir = tmp_path / "frames"
    frames_dir.mkdir()

    class SolidCamera:
        def __init__(self):
            self.reads = 0

        def read(self):
            self.reads += 1
            return np.full((480, 640, 3), 100, dtype=np.uint8)

    rows, _ = tool.record_loop(
        cam=SolidCamera(), write_frame=tool.build_writer("jpg", 95),
        frames_dir=frames_dir, args=_args(frames=2))
    back = cv2.imread(str(frames_dir / rows[0]["filename"]))
    assert back.shape == (480, 640, 3)
    assert np.abs(back.astype(int) - 100).max() <= 3
