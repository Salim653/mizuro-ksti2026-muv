"""State machine runtime.

Alur: BOOT -> READY -> RUNNING <-> DEGRADED -> STOPPED -> READY ...
FAULT bersifat terminal: keluar dari FAULT hanya lewat restart proses yang
disengaja. Transisi tidak valid melempar RuntimeError alih-alih diabaikan —
state yang salah diam-diam lebih berbahaya daripada crash.
"""
from __future__ import annotations

import time
from enum import Enum, auto


class State(Enum):
    BOOT = auto()
    READY = auto()
    RUNNING = auto()
    DEGRADED = auto()
    STOPPED = auto()
    FAULT = auto()


_TRANSITIONS: dict[State, frozenset[State]] = {
    State.BOOT: frozenset({State.READY, State.FAULT, State.STOPPED}),
    State.READY: frozenset({State.RUNNING, State.FAULT, State.STOPPED}),
    State.RUNNING: frozenset({State.DEGRADED, State.STOPPED, State.FAULT}),
    State.DEGRADED: frozenset({State.RUNNING, State.STOPPED, State.FAULT}),
    State.STOPPED: frozenset({State.READY, State.FAULT}),
    State.FAULT: frozenset(),  # terminal
}


class StateMachine:
    def __init__(self) -> None:
        self.state = State.BOOT
        self.reason = "startup"
        self.started_at: float | None = None

    def transition(self, new: State, reason: str) -> None:
        if new not in _TRANSITIONS[self.state]:
            raise RuntimeError(
                f"transisi tidak valid: {self.state.name} -> {new.name} ({reason})")
        self.state, self.reason = new, reason
        if new is State.RUNNING and self.started_at is None:
            self.started_at = time.monotonic()

    def elapsed_s(self, now: float | None = None) -> float:
        """Detik sejak RUNNING pertama (0.0 jika belum pernah berjalan).

        Dipakai untuk batas 300 detik; sengaja TIDAK reset saat DEGRADED —
        waktu percobaan terus berjalan walau persepsi drop.
        """
        if self.started_at is None:
            return 0.0
        return (time.monotonic() if now is None else now) - self.started_at
