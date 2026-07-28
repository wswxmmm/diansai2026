#!/usr/bin/env python3
"""Raspberry Pi side of the MSPM0 UART link."""

from __future__ import annotations

import argparse
import os
import secrets
import sys
import time

import serial


DEFAULT_PORT = "/dev/ttyAMA0"
DEFAULT_BAUD = 115200


def open_serial(port: str, baud: int, timeout: float) -> serial.Serial:
    try:
        return serial.Serial(port=port, baudrate=baud, timeout=0.1, write_timeout=timeout)
    except (OSError, serial.SerialException) as exc:
        raise SystemExit(
            f"无法打开 {port}: {exc}\n"
            "Pi 5 的 GPIO14/15 UART 通常是 /dev/ttyAMA0。若该设备不存在，"
            "请先运行 ./setup_uart.sh 后重启。"
        ) from exc


def read_line(link: serial.Serial, deadline: float) -> str | None:
    while time.monotonic() < deadline:
        raw = link.readline()
        if raw:
            return raw.decode("ascii", errors="replace").strip()
    return None


def request(link: serial.Serial, command: str, expected_prefix: str, timeout: float) -> str:
    link.write((command + "\n").encode("ascii"))
    link.flush()
    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        line = read_line(link, deadline)
        if line is None:
            break
        print(f"MCU > {line}")
        if line.startswith(expected_prefix):
            return line

    raise TimeoutError(f"发送 {command!r} 后未收到 {expected_prefix!r}")


def run_self_test(link: serial.Serial, timeout: float) -> None:
    token = secrets.token_hex(4)
    message = "hello_from_raspberry_pi"
    cases = [
        (f"PING {token}", f"PONG {token}"),
        (f"ECHO {message}", f"ECHO {message}"),
        ("STATUS", "STATUS OK MSPM0G3507 UART3 115200 FW5"),
        ("INVALID", "ERR UNKNOWN_COMMAND"),
    ]

    link.reset_input_buffer()
    for command, expected in cases:
        print(f"Pi  > {command}")
        response = request(link, command, expected, timeout)
        if response != expected:
            raise RuntimeError(f"响应不匹配: 期望 {expected!r}, 实际 {response!r}")

    print("PASS: 天猛星与树莓派双向 UART 通信正常")


def run_terminal(link: serial.Serial, timeout: float) -> None:
    print("交互模式：输入 PING、ECHO text、STATUS、HELP；输入 quit 退出。")
    while True:
        try:
            command = input("Pi  > ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return
        if command.lower() in {"quit", "exit"}:
            return
        if not command:
            continue
        link.write((command + "\n").encode("ascii"))
        link.flush()
        response = read_line(link, time.monotonic() + timeout)
        print(f"MCU > {response if response is not None else '[超时]'}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="MSPM0G3507 天猛星 UART 通信测试")
    parser.add_argument("--port", default=DEFAULT_PORT, help=f"串口设备（默认 {DEFAULT_PORT}）")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="波特率")
    parser.add_argument("--timeout", type=float, default=2.0, help="单次请求超时秒数")
    parser.add_argument("--terminal", action="store_true", help="进入交互终端；默认运行自检")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not os.path.exists(args.port):
        print(
            f"提示: {args.port} 当前不存在。Pi 5 需启用 40 针排针上的 UART0。",
            file=sys.stderr,
        )

    with open_serial(args.port, args.baud, args.timeout) as link:
        time.sleep(0.1)
        if args.terminal:
            run_terminal(link, args.timeout)
        else:
            run_self_test(link, args.timeout)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
