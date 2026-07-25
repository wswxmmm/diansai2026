# 20_DUAL_WHEEL_PID_TUNER

Dual-wheel speed PID test and live tuner for the existing M0 vehicle wiring.
Lift both drive wheels before the first run.

## Keys

| Key | Function |
| --- | --- |
| KEY1 / PB17 | Select target, KP, KI or KD |
| KEY2 / PB18 | Increase selected value |
| KEY3 / PB19 | Decrease selected value |
| KEY4 / PB20 | Start or stop both wheels |
| KEY5 / PB24 | Toggle the 30/60 encoder-speed automatic step test |
| KEY6 / PB25 | Clear PID state, speed filters and encoder counts |

The target and measured speed share a 0 to 100 display scale. One display unit
is 200 encoder pulses per second, and each key press changes the target by 5.
The OLED uses `T` for target, `A` for actual speed and `O` for PID output in
percent. `M` is manual target mode and `A` in the header is automatic mode.

## Fast tuning order

1. Select `KI` and set it to 0.000. Keep `KD` at 0.000.
2. Enable automatic step mode and increase `KP` until response is quick but
   does not continuously oscillate. Reduce KP by one or two steps if it does.
3. Increase `KI` until steady-state speed error disappears in a reasonable
   time. Use KEY6 after a large gain change for a repeatable comparison.
4. Leave `KD` at zero unless overshoot remains. Add it one very small step at
   a time because encoder-speed differentiation amplifies pulse noise.

The tuned initial values are KP=0.020, KI=0.030 and KD=0.000. Output is limited
to 0.0 to 60.0 percent. Outputs below 5.0 percent use pulse-density drive;
otherwise the PWM is continuous. The controller includes target ramping,
speed filtering, feed-forward, derivative filtering, integral limiting and
anti-windup.

With the wheels lifted, the final 30/60 automatic step test corresponds to
6000/12000 encoder pulses per second. Repeated steady platform errors were
below 1 percent on both wheels.

The right encoder sign is inverted in `bsp_encoder.h`, so forward motion is
positive for both wheels. The configured GMR conversion is 1000 counts per
wheel revolution. The 0 to 100 tuning scale intentionally uses raw encoder
increments and does not depend on that conversion being exact.

For CCS Live Expressions, the editable variables are:

- `g_tuneTargetSpeed`
- `g_tuneKpX1000`
- `g_tuneKiX1000`
- `g_tuneKdX1000`

## PC telemetry and remote tuning

The preferred transport is SEGGER RTT over the existing J-Link SWD connection.
It needs no UART TX/RX wires. UART0 on PA10/PA11 at 115200 baud remains
available as a backup. Both transports accept the same commands and receive
the same telemetry.

The firmware sends a `DATA` line after every 50 ms control update with the
ramped target, filtered left/right speed, errors, outputs and gains. Supported
commands are:

```text
GET
PING
RUN
STOP
RESET
AUTO,1
AUTO,0
SET,TARGET,40
SET,KP,200
SET,KI,0
SET,KD,0
```

Remote `RUN` and `AUTO,1` enable a two-second heartbeat watchdog. The motors
stop if the PC stops sending commands or `PING` messages.

`capture_pid_rtt.ps1` connects to the RTT server exposed by an active CCS
J-Link debug session and records telemetry to CSV. It does not start the motors
unless the explicit `-Arm` switch is supplied. Example monitor-only capture:

```powershell
powershell -ExecutionPolicy Bypass -File .\capture_pid_rtt.ps1 -DurationSeconds 5
```

Example automatic 30/60 step capture:

```powershell
powershell -ExecutionPolicy Bypass -File .\capture_pid_rtt.ps1 -DurationSeconds 20 -Kp 20 -Ki 30 -Kd 0 -AutoStep -Arm
```

`capture_pid.ps1` provides the same workflow over UART when a CH340 or USB-TTL
adapter is available:

```powershell
powershell -ExecutionPolicy Bypass -File .\capture_pid.ps1 -Port COM9 -DurationSeconds 20 -Kp 20 -Ki 30 -Kd 0 -AutoStep -Arm
```
