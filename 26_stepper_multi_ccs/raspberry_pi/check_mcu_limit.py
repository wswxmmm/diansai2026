#!/usr/bin/env python3
"""Read firmware identity and relative-jog limits without moving a motor."""

import serial


def exchange(port: serial.Serial, command: str) -> str:
    port.write((command + "\n").encode("ascii"))
    port.flush()
    return port.readline().decode("ascii", errors="replace").strip()


with serial.Serial("/dev/ttyAMA0", 115200, timeout=2.0) as link:
    link.reset_input_buffer()
    print(exchange(link, "STATUS"))
    print(link.readline().decode("ascii", errors="replace").strip())
    print(exchange(link, "HELP"))
    print(exchange(link, "RELATIVE START"))
    print(exchange(link, "RELATIVE STOP"))
