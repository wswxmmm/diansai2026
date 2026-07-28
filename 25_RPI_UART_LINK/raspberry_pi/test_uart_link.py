#!/usr/bin/env python3
"""Host-only integration test using a pseudo terminal and a fake MCU."""

from __future__ import annotations

import os
import pty
import subprocess
import sys
import threading


def fake_mcu(master_fd: int) -> None:
    pending = b""
    while True:
        try:
            data = os.read(master_fd, 256)
        except OSError:
            return
        if not data:
            return
        pending += data
        while b"\n" in pending:
            raw, pending = pending.split(b"\n", 1)
            command = raw.rstrip(b"\r").decode("ascii")
            if command.startswith("PING "):
                response = "PONG " + command[5:]
            elif command.startswith("ECHO "):
                response = command
            elif command == "STATUS":
                response = "STATUS OK MSPM0G3507 UART3 115200 FW5"
            else:
                response = "ERR UNKNOWN_COMMAND"
            os.write(master_fd, (response + "\r\n").encode("ascii"))


def main() -> int:
    master_fd, slave_fd = pty.openpty()
    slave_name = os.ttyname(slave_fd)
    worker = threading.Thread(target=fake_mcu, args=(master_fd,), daemon=True)
    worker.start()

    result = subprocess.run(
        [sys.executable, "uart_link.py", "--port", slave_name, "--timeout", "1"],
        cwd=os.path.dirname(os.path.abspath(__file__)),
        check=False,
    )
    os.close(slave_fd)
    os.close(master_fd)
    return result.returncode


if __name__ == "__main__":
    raise SystemExit(main())
