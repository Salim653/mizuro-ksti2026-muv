"""Tes state machine: alur benar, transisi tidak valid, FAULT terminal."""
import pytest

from mizuro.states import State, StateMachine


def test_happy_path_and_rearm():
    sm = StateMachine()
    assert sm.state is State.BOOT
    sm.transition(State.READY, "checks")
    sm.transition(State.RUNNING, "start")
    assert sm.started_at is not None
    first_started = sm.started_at
    sm.transition(State.DEGRADED, "lane lost")
    sm.transition(State.RUNNING, "recovered")
    assert sm.started_at == first_started  # timer tidak reset oleh DEGRADED
    sm.transition(State.STOPPED, "done")
    sm.transition(State.READY, "rearm")
    assert sm.reason == "rearm"


def test_invalid_transition_raises():
    sm = StateMachine()
    with pytest.raises(RuntimeError, match="tidak valid"):
        sm.transition(State.RUNNING, "lewati READY")


def test_fault_is_terminal():
    sm = StateMachine()
    sm.transition(State.FAULT, "boom")
    for target in (State.READY, State.RUNNING, State.STOPPED, State.DEGRADED):
        with pytest.raises(RuntimeError):
            sm.transition(target, "coba keluar dari FAULT")


def test_elapsed_zero_before_run():
    sm = StateMachine()
    assert sm.elapsed_s() == 0.0


def test_elapsed_uses_injected_clock():
    sm = StateMachine()
    sm.transition(State.READY, "checks")
    sm.transition(State.RUNNING, "start")
    assert sm.elapsed_s(now=sm.started_at + 12.5) == pytest.approx(12.5)
