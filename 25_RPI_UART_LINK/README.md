# 天猛星 MSPM0G3507 ↔ Raspberry Pi 5 UART 通信

本工程通过 3.3V TTL UART 实现树莓派与天猛星的双向命令/响应通信。

## 接线

断电后交叉连接，只接三根线：

| 天猛星 | Raspberry Pi 5 | 说明 |
|---|---|---|
| PB12 / UART3 TX | GPIO15 / RXD，物理 10 脚 | MCU 发 → Pi 收 |
| PB13 / UART3 RX | GPIO14 / TXD，物理 8 脚 | Pi 发 → MCU 收 |
| GND | GND，物理 6 脚 | 必须共地 |

两端均为 3.3V 电平。不要连接 5V，也不要将 TX 接 TX。

## 天猛星端

- 芯片：MSPM0G3507，LQFP-64
- CCS 工程：直接导入本目录
- UART3：PB12 TX、PB13 RX、115200、8N1（避开板载 CH340E 和综合工程已用引脚）
- 烧录 `main.c` 后 PB22 板载 LED 每 0.5 秒翻转，并约每秒发送一次 `HEARTBEAT MSPM0G3507`

支持的文本协议（每条命令以 `\n` 结尾）：

| 请求 | 响应 |
|---|---|
| `PING [token]` | `PONG [token]` |
| `ECHO text` | `ECHO text` |
| `STATUS` | `STATUS OK MSPM0G3507 UART3 115200 FW5` |
| `HELP` | 命令列表 |

## 树莓派端

Pi 5 的 40 针排针 UART0 是 `/dev/ttyAMA0`。首次使用先执行：

```bash
cd /home/wxm/tmx_uart_link
./setup_uart.sh
sudo reboot
```

重启后运行自动收发测试：

```bash
cd /home/wxm/tmx_uart_link
python3 uart_link.py
```

成功时最后一行应为：

```text
PASS: 天猛星与树莓派双向 UART 通信正常
```

交互调试：

```bash
python3 uart_link.py --terminal
```

如果改用 USB-TTL 模块，可指定设备，例如：

```bash
python3 uart_link.py --port /dev/ttyUSB0
```

## 无硬件软件自检

`test_uart_link.py` 会创建伪终端模拟 MCU，用于验证树莓派脚本和协议逻辑：

```bash
python3 test_uart_link.py
```
