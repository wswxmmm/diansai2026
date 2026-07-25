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

The target and measured speed use encoder counts per 50 ms control period.
Their adjustment range is 0 to 100 and each key press changes the target by 5.
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

The initial values are KP=0.200, KI=0.100 and KD=0.000. Output is limited to
8.0 to 60.0 percent while running. The controller includes target ramping,
speed filtering, feed-forward, derivative filtering, integral limiting and
anti-windup.

The right encoder sign is inverted in `bsp_encoder.h`, so forward motion is
positive for both wheels. The configured GMR conversion is 1000 counts per
wheel revolution. The 0 to 100 tuning scale intentionally uses raw encoder
increments and does not depend on that conversion being exact.

For CCS Live Expressions, the editable variables are:

- `g_tuneTargetSpeed`
- `g_tuneKpX1000`
- `g_tuneKiX1000`
- `g_tuneKdX1000`
