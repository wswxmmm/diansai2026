# 25_stepper_5motor_cm

MSPM0G3507（天猛星）通过一组 UART 总线控制 1～5 号 Emm 步进电机驱动器，并以厘米为单位调用运动函数。

## 接线与驱动器设置

- 天猛星 PA10（UART0 TX）连接所有驱动器 RX
- 天猛星 PA11（UART0 RX）连接所有驱动器 TX
- 天猛星 GND 与所有驱动器通信 GND 共地
- 五个驱动器地址分别设置为 1、2、3、4、5
- 所有驱动器使用 Emm 模式、115200 baud、校验字节 0x6B
- 电机动力电源单独供电，不要从开发板 USB 取电

## 最简单的调用

正数表示正方向，负数表示反方向：

```c
Motor1_MoveCm(10.0F);  /* 1号正向 10 cm */
Motor2_MoveCm(-2.5F);  /* 2号反向 2.5 cm */
Motor3_MoveCm(5.0F);
Motor4_MoveCm(1.0F);
Motor5_MoveCm(0.5F);
```

也可以按编号调用：

```c
StepperCm_Move(STEPPER_MOTOR_4, -1.0F);
```

指定速度和加速度：

```c
StepperCm_MoveWithParams(STEPPER_MOTOR_1, 10.0F, 600U, 10U);
```

## 多电机同步运动

例如 2、3 号同时正向移动 10 cm：

```c
const StepperCmMove yAxis[] = {
    {STEPPER_MOTOR_2, 10.0F},
    {STEPPER_MOTOR_3, 10.0F}
};

StepperCm_MoveSynchronized(yAxis, 2U);
```

同步接口先向每台驱动器发送排队命令，最后只发送一次广播同步帧，因此多台电机同时启动。

## 当前按键演示

烧录后等待约 2 秒。每按一次 PB21，下一个编号的电机正向移动 1 cm，测试顺序循环为：

`1 -> 2 -> 3 -> 4 -> 5 -> 1 ...`

PB22 闪烁次数表示刚刚发送命令的电机编号。LED 常亮表示串口命令发送失败。

## 标定和参数

参数集中在 `stepper_cm_config.h`：

- 默认速度：300 RPM
- 默认加速度：10
- 当前细分：3200 脉冲/圈
- 当前丝杆导程：4 mm/圈（根据实测 10 圈移动 4 cm）
- 单条命令最大绝对距离：200 cm，用于防止误输入

如果某个电机的机械正方向相反，把对应的 `STEPPER_MOTOR_x_DIRECTION_XOR` 从 0 改为 1。
