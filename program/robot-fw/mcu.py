from core import Command


def format_command(command: Command) -> bytes:
    if not (-0.8 <= command.speed_mps <= 0.8 and -0.55 <= command.steering_rad <= 0.55):
        raise ValueError("command outside wire protocol limits")
    return f"{command.speed_mps:.4f},{command.steering_rad:.4f}\n".encode("ascii")


class McuLink:
    def __init__(self, device: str, baud: int, dry_run: bool) -> None:
        self.dry_run = dry_run
        self.serial = None
        if not dry_run:
            import serial
            self.serial = serial.Serial(device, baudrate=baud, timeout=0.05, write_timeout=0.05)

    def send(self, command: Command) -> None:
        if self.dry_run:
            return
        self.serial.write(format_command(command))

    def close(self) -> None:
        if self.serial is not None:
            try:
                self.send(Command(reason="shutdown"))
            finally:
                self.serial.close()
