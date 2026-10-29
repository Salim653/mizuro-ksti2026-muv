"""Tautan serial ke MCU bodi.

Setiap send() adalah heartbeat: MCU memutus penggerak ketika setpoint tidak
datang dalam WATCHDOG_MS. Kalau penulisan serial gagal (kabel lepas, MCU
mati), link ditandai mati (alive=False) alih-alih melempar exception —
loop utama yang memutuskan transisi ke FAULT, sementara watchdog MCU sudah
menghentikan penggerak secara independen.
"""
from __future__ import annotations

from .protocol import DEFAULT_BAUD, format_command
from .types import Command


class McuLink:
    """Driver serial satu arah (setpoint saja sampai Fase 4 menambah status).

    `opener` bisa disuntik untuk test; default membuka port sungguhan lewat
    pyserial. Import pyserial sengaja lokal supaya self-check dan pytest
    berjalan di mesin tanpa pyserial.
    """

    def __init__(self, device: str, baud: int = DEFAULT_BAUD, *,
                 dry_run: bool = True, opener=None) -> None:
        self.device = device
        self.baud = baud
        self.dry_run = dry_run
        self.alive = True
        self.serial = None
        if not dry_run:
            if opener is None:
                import serial as pyserial

                def opener(dev: str, baud: int, _mod=pyserial):
                    return _mod.Serial(dev, baudrate=baud,
                                       timeout=0.05, write_timeout=0.05)

            self.serial = opener(device, baud)

    def send(self, command: Command) -> None:
        """Kirim satu setpoint (heartbeat). Dry-run atau link mati: no-op."""
        if self.dry_run or self.serial is None or not self.alive:
            return
        try:
            self.serial.write(format_command(command))
        except OSError:
            # pyserial.SerialException dan SerialTimeoutException turunan OSError.
            self.alive = False

    def close(self) -> None:
        """Kirim nol terakhir (best effort) lalu tutup port. Selalu aman."""
        if self.serial is None:
            return
        try:
            if self.alive:
                self.serial.write(format_command(Command(reason="shutdown")))
        except OSError:
            pass
        finally:
            try:
                self.serial.close()
            except OSError:
                pass
            self.serial = None
            self.alive = False
