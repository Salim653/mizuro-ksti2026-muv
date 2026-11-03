"""Closed-loop runner Fase 4: persepsi -> kontrol -> MCU, satu loop.

Arsitektur: semua dependensi disuntik (kamera, tracker, controller,
mcu, log, state machine) supaya loop bisa diuji TANPA hardware —
test pakai stub yang merekam apa yang dikirim.

Kontrak keselamatan yang ditegakkan loop ini:
1. Setiap iterasi WAJIB kirim setpoint ke MCU (watchdog 100 ms) —
   bahkan nol saat berhenti. Ini heartbeat.
2. Confidence 0 / lane hilang -> controller kirim nol; state ->
   DEGRADED tapi loop TETAP JALAN (robot berhenti, bukan crash).
3. Loop overrun (> 1/loop_hz) di-log sebagai event; bukan fatal,
   tapi kalau terus terjadi berarti persepsi terlalu lambat.
4. Timeout run (default 300 s) -> STOPPED.
5. dry_run: MCU tidak menulis serial, tapi loop penuh berjalan —
   output di-log untuk analisis.
"""
from __future__ import annotations

import time

from .states import State, StateMachine
from .telemetry import RunLog
from .types import Command, Observation


class ClosedLoopRunner:
    """Satu loop kontrol: baca frame -> track -> kontrol -> kirim -> log."""

    def __init__(self, *, camera, tracker, controller, mcu, log: RunLog,
                 machine: StateMachine, calib, loop_hz: float = 60.0,
                 run_timeout_s: float = 300.0,
                 max_inference_ms: float = 150.0,
                 detector=None) -> None:
        self.camera = camera
        self.tracker = tracker
        self.controller = controller
        self.mcu = mcu
        self.log = log
        self.machine = machine
        self.calib = calib
        self.loop_hz = loop_hz
        self.run_timeout_s = run_timeout_s
        self.max_inference_ms = max_inference_ms
        self.detector = detector
        self._period = 1.0 / loop_hz if loop_hz > 0 else 0.0
        self._stop = False
        self.iterations = 0
        self.overruns = 0
        self.lane_lost_count = 0

    # -- loop utama ------------------------------------------------------

    def run(self) -> None:
        """Jalankan sampai timeout, KeyboardInterrupt, atau stop()."""
        self.machine.transition(State.READY, "closed-loop siap")
        self.log.event("ready", loop_hz=self.loop_hz,
                       dry_run=self.mcu.dry_run)
        try:
            self.machine.transition(State.RUNNING, "loop mulai")
            self.log.event("running")
            self._loop()
        except KeyboardInterrupt:
            self.log.event("keyboard_interrupt")
        finally:
            self._shutdown()

    def _loop(self) -> None:
        start = time.monotonic()
        while not self._stop:
            now = time.monotonic()
            if now - start >= self.run_timeout_s:
                self.log.event("run_timeout", elapsed_s=now - start)
                break

            t0 = time.monotonic()
            command = self._iterate(now)
            elapsed_ms = (time.monotonic() - t0) * 1000.0

            # Heartbeat: MCU butuh setpoint segar setiap iterasi.
            self.mcu.send(command)

            self._check_overrun(elapsed_ms, now)
            self.iterations += 1

            if self._period > 0:
                remaining = self._period - (time.monotonic() - t0)
                if remaining > 0:
                    time.sleep(remaining)

    def _iterate(self, now: float) -> Command:
        """Satu iterasi: frame -> track -> (deteksi) -> kontrol -> Command."""
        frame = self.camera.read()

        obs = self.tracker.estimate(frame, self.calib, now)

        # Detektor semantik opsional: tidak memblokir loop kontrol.
        # Kalau model_path tidak di-set, self.detector None -> skip.
        if self.detector is not None:
            try:
                dets = self.detector.infer(frame)
                obs = obs._replace(detections=tuple(
                    {"label": d.label, "confidence": d.confidence,
                     "class_id": d.class_id} for d in dets))
            except Exception as exc:  # detektor gagal != robot berhenti
                self.log.event("detector_error", error=str(exc))

        command = self.controller.command(obs)

        if obs.confidence <= 0.0:
            self.lane_lost_count += 1
            if self.machine.state is State.RUNNING:
                self.machine.transition(State.DEGRADED, "lajur hilang")
                self.log.event("lane_lost")
        elif self.machine.state is State.DEGRADED:
            self.machine.transition(State.RUNNING, "lajur ditemukan lagi")
            self.log.event("lane_recovered")

        self.log.row(self.machine, obs, command)
        return command

    # -- pembantu ----------------------------------------------------------

    def _check_overrun(self, elapsed_ms: float, now: float) -> None:
        budget_ms = self._period * 1000.0 if self._period > 0 else 0.0
        if budget_ms > 0 and elapsed_ms > budget_ms:
            self.overruns += 1
            # Log setiap 10 overrun supaya events.jsonl tidak membanjir.
            if self.overruns % 10 == 1:
                self.log.event("loop_overrun", elapsed_ms=round(elapsed_ms, 1),
                               budget_ms=round(budget_ms, 1),
                               total_overruns=self.overruns)

    def _shutdown(self) -> None:
        # Perintah nol terakhir: MCU watchdog melihat setpoint valid.
        try:
            self.mcu.send(Command(reason="shutdown"))
        except Exception:
            pass
        if self.machine.state is not State.FAULT:
            try:
                self.machine.transition(State.STOPPED, "loop selesai")
            except RuntimeError:
                pass
        self.log.event("shutdown", state=self.machine.state.name,
                       reason=self.machine.reason,
                       iterations=self.iterations,
                       overruns=self.overruns,
                       lane_lost=self.lane_lost_count)

    def stop(self) -> None:
        """Minta loop berhenti setelah iterasi ini (dipakai test/ESC)."""
        self._stop = True
