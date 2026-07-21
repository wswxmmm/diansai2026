# 16_stepper_uart：双串口控制两台X V2电机

MSPM0G3507使用两个独立硬件串口分别连接两块X V2闭环步进驱动器。每台电机必须各配一块驱动器。

## 接线

| 电机 | MSPM0发送 | MSPM0接收 | 驱动器 |
| --- | --- | --- | --- |
| 电机1 / 驱动器1 | PA10 / UART0 TX → RX | PA11 / UART0 RX ← TX | 地址1 |
| 电机2 / 驱动器2 | PA8 / UART1 TX → RX | PA9 / UART1 RX ← TX | 地址1 |

两块驱动器都连接MSPM0的GND，同时各自连接自己的电机和电机电源。因为使用独立串口，两块驱动器都可以保持默认地址1，TX也不会发生并联冲突。

串口参数：115200、8N1、TTL模式，校验字节`0x6B`。不要把RS232或RS485电平直接接到MSPM0 GPIO。

## 双电机启动方式

程序分别通过UART0、UART1给两块驱动器发送带同步标志的待执行命令，然后在两路串口分别发送：

```text
00 FF 66 6B
```

两路硬件UART独立工作，触发时间差约为几百微秒。若应用要求严格到微秒级的硬同步，应使用同一RS485总线广播或外部硬件触发。

## 按键模式

PB21每按一次切换模式，PB22闪烁次数对应模式编号：

| 次数 | 两台电机动作 |
| ---: | --- |
| 0 | 停止并保持使能 |
| 1 | 方向0，100 RPM连续转动 |
| 2 | 方向1，100 RPM连续转动 |
| 3 | 方向0，相对定位90° |
| 4 | 方向1，相对定位90° |
| 5 | 返回停止 |

如果电机2在机构中需要与电机1方向相反：

```c
#define MOTOR_2_REVERSE_DIRECTION 1U
```

默认值为0，两块驱动器收到相同方向位。

## 配置摘要

```c
#define MOTOR_PROTOCOL              MOTOR_PROTOCOL_X
#define MOTOR_1_ADDRESS              1U
#define MOTOR_2_ADDRESS              1U
#define MOTOR_2_REVERSE_DIRECTION    0U
#define MOTOR_TEST_SPEED_RPM         100U
#define MOTOR_TEST_ACCEL             1000U
#define MOTOR_POSITION_DEGREES       90.0f
```

X定位命令直接按0.1°发送角度，90°对应数值900，不依赖驱动器细分设置。

构建后下载`Debug/16_stepper_uart.out`。首次测试让两个电机空载，并准备随时切断两块驱动器的电机电源。
