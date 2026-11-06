"""Tautan serial ke MCU bodi.

Setiap send() adalah heartbeat: MCU memutus penggerak ketika setpoint tidak
datang dalam WATCHDOG_MS. Kalau penulisan serial gagal (kabel lepas, MCU
mati), link ditandai mati (alive=False) alih-alih melempar exception —
loop utama yang memutuskan transisi ke FAULT, sementara watchdog MCU sudah
menghentikan penggerak secara independen.

Saat port dibuka (bukan dry-run), baris mode #M,<mode> dikirim lebih dulu
supaya firmware memakai tahap output yang sama dengan config robot.json.
read_status() membaca balik status MCU (#v1,...) secara non-blocking.
"""
from __future__ import annotations

from .protocol import (
    DEFAULT_BAUD,
    FIRMWARE_MODES,
    format_command,
    format_mode,
    parse_status_line,
)
from .types import Command, McuStatus

MAX_STATUS_BUFFER = 256   # sampah tanpa newline tidak boleh menumpuk


class McuLink:
    """Driver serial satu arah (setpoint saja sampai Fase 4 menambah status).

    `opener` bisa disuntik untuk test; default membuka port sungguhan lewat
    pyserial. Import pyserial sengaja lokal supaya self-check dan pytest
    berjalan di mesin tanpa pyserial.
    """

    def __init__(self, device: str, baud: int = DEFAULT_BAUD, *,
                 dry_run: bool = True, opener=None,
                 driver: str = "external_esc") -> None:
        self.device = device
        self.baud = baud
        self.dry_run = dry_run
        self.driver = driver
        self.alive = True
        self.serial = None
        self._rx_buffer = b""
        # Validasi driver SEKARANG (dry-run pun): config salah harus
        # gagal di sini, bukan jadi output salah di roda.
        self._mode_line = format_mode(driver)
        if not dry_run:
            if opener is None:
                import serial as pyserial

                def opener(dev: str, baud: int, _mod=pyserial):
                    return _mod.Serial(dev, baudrate=baud,
                                       timeout=0.05, write_timeout=0.05)

            self.serial = opener(device, baud)
            try:
                # Mode dulu, sebelum setpoint pertama: firmware perlu tahu
                # tahap output mana yang dipakai.
                self.serial.write(self._mode_line)
            except OSError:
                self.alive = False

    def send(self, command: Command) -> None:
        """Kirim satu setpoint (heartbeat). Dry-run atau link mati: no-op."""
        if self.dry_run or self.serial is None or not self.alive:
            return
        try:
            self.serial.write(format_command(command))
        except OSError:
            # pyserial.SerialException dan SerialTimeoutException turunan OSError.
            self.alive = False

    def read_status(self) -> McuStatus | None:
        """Baca baris status MCU terbaru (non-blocking).

        Mengembalikan None kalau tidak ada baris lengkap baru, link mati,
        atau dry-run. Baris yang tidak sesuai format diabaikan; buffer
        dibatasi supaya sampah tanpa newline tidak menumpuk.
        """
        if self.dry_run or self.serial is None or not self.alive:
            return None
        try:
            waiting = int(getattr(self.serial, "in_waiting", 0) or 0)
            if waiting > 0:
                chunk = self.serial.read(waiting)
                if chunk:
                    self._rx_buffer += chunk
        except OSError:
            self.alive = False
            return None

        status: McuStatus | None = None
        while b"\n" in self._rx_buffer:
            raw, _, self._rx_buffer = self._rx_buffer.partition(b"\n")
            parsed = parse_status_line(raw.decode("utf-8", errors="replace"))
            if parsed is not None:
                status = parsed   # simpan yang terbaru
        if len(self._rx_buffer) > MAX_STATUS_BUFFER:
            self._rx_buffer = self._rx_buffer[-64:]
        return status

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

    def __enter__(self) -> "McuLink":
        return self

    def __exit__(self, *exc) -> None:
        self.close()
