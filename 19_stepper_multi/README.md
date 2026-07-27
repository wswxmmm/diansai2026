# 19_stepper_multi：单 UART 控制三台 Emm 电机

本工程使用 MSPM0G3507 的一个 UART 控制三块 Emm 固件闭环步进驱动器。每按一次按键，三台电机同步转动十分之一圈。

## 驱动器设置

三块驱动器均设置为：

- 固件/协议：Emm
- 通信模式：UART/TTL
- 波特率：115200，8N1
- 地址：分别为 1、2、3
- 细分：16（本工程按 3200 脉冲/圈计算）
- 到位返回和主动返回：关闭，避免三块 TTL TX 同时输出

如果屏幕细分不是 16，必须修改 `multi_motor_config.h` 中的 `MOTOR_PULSES_PER_REV`。例如 32 细分应设为 6400。

## 接线

只需要一个 UART：

```text
MSPM0 PA10 (UART0 TX) ──┬── 驱动器1 RX/RAH（地址1）
                         ├── 驱动器2 RX/RAH（地址2）
                         └── 驱动器3 RX/RAH（地址3）

MSPM0 GND ──────────────┬── 驱动器1 GND
                         ├── 驱动器2 GND
                         └── 驱动器3 GND
```

三块驱动器的 TX/TBL 不要互相并联，也不要接 PA11。此接法只发送控制命令，不读取反馈，适合桌面短距离测试。长线或正式使用建议改用 3.3V RS485 收发器和 A/B 总线。

## 按键测试

上电后等待2秒，程序自动使能并停止地址1、2、3。PB21每按一次同步运行三台电机：

| 操作 | 动作 |
| --- | --- |
| 每按一次PB21 | 1号、2号同方向转36°，3号反方向转36°，随后全部停止 |

每次运行结束后PB22闪烁3次，表示三台电机均已执行。3号通过 `MOTOR_3_DIRECTION_XOR=1` 设置为反方向。

## Emm 测试帧

地址 1、100 RPM、加速度参数 10、相对 36°（320 脉冲）的待同步定位帧：

```text
01 FD 00 00 64 0A 00 00 01 40 00 01 6B
```

地址2、3的帧首字节分别为 `02`、`03`，其中3号方向字节为 `01`。三条待同步命令发完后广播：

```text
00 FF 66 6B
```

## CCS 调试变量

- `g_multiMotorMode`：按键前为0，三台执行后为3
- `g_multiLastOperationOk`：最近一组帧是否成功从 MCU UART 发出
- `g_emmMultiFrameCount`：已成功发送的帧数
- `g_emmMultiSendErrorCount`：UART 发送失败次数
- `g_emmMultiLastFrame` / `g_emmMultiLastFrameLength`：最后发送的原始帧

构建并下载 `Debug/19_stepper_multi.out`。首次测试请让三个电机空载，并准备随时切断驱动器电源。
