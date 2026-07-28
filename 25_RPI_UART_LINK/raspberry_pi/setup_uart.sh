#!/bin/sh
set -eu

if [ "$(id -u)" -eq 0 ]; then
    echo "请以普通用户运行本脚本；需要写启动配置时脚本会调用 sudo。" >&2
    exit 2
fi

model=$(tr -d '\000' </proc/device-tree/model 2>/dev/null || true)
case "$model" in
    *"Raspberry Pi 5"*) ;;
    *)
        echo "警告：检测到的型号是 '$model'，本工程按 Raspberry Pi 5 配置。" >&2
        ;;
esac

if [ -e /dev/ttyAMA0 ]; then
    echo "UART0 已启用：/dev/ttyAMA0"
    exit 0
fi

echo "将启用 Pi 5 的 GPIO14/15 UART0（写入 dtparam=uart0=on）。"
sudo raspi-config nonint do_serial_hw 0
echo "配置完成。请执行 sudo reboot；重启后应出现 /dev/ttyAMA0。"
