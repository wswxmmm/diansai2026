# 18_DUAL_WHEEL_PID_TEST

Standalone dual-wheel encoder PI speed-control test for MSPM0G3507 and
TB6612FNG. Test with both drive wheels lifted off the ground first.

## Wiring

| Signal | MSPM0G3507 pin |
| --- | --- |
| TB6612 PWMA / PWMB | PA16 / PA17 |
| TB6612 AIN1 / AIN2 | PA14 / PA15 |
| TB6612 BIN1 / BIN2 | PA12 / PA13 |
| Left encoder A / B | PA7 / PA8 |
| Right encoder A / B | PB6 / PB8 |
| OLED SCL / SDA | PB2 / PB3 |
| KEY1..KEY6 | PB17, PB18, PB19, PB20, PB24, PB25 |

The motor supply, TB6612, encoders and MSPM0G3507 must share ground.

## Keys

| Key | Function |
| --- | --- |
| KEY1 | Start both wheel controllers |
| KEY2 | Increase target by 0.2 revolutions/second |
| KEY3 | Decrease target by 0.2 revolutions/second |
| KEY4 | Stop both motors |
| KEY5 | Stop and toggle forward/reverse direction |
| KEY6 | Reset both PI controllers and restart the target ramp |

The default target is 0.8 revolutions/second. The selectable range is 0.2 to
3.0 revolutions/second. KEY5 never reverses a moving motor; press KEY1 after
selecting the new direction.

## OLED

The display shows run state, direction, target speed, filtered left/right
speeds, PWM percentages, PI errors, encoder counts per revolution and the
50 ms controller period.

## Controller

Each wheel has an independent PI controller. The setpoint ramps by 50 pulses
per second every update. The controller contains feed-forward, proportional
feedback, integral limiting, a 10 percent minimum useful PWM and a 60 percent
safety limit. Derivative feedback is intentionally disabled for initial speed
tuning because encoder speed is quantized.

Initial parameters are in `speed_pi.h`:

- `SPEED_PI_KP_DIVISOR = 100`
- `SPEED_PI_KI_DIVISOR = 2000`
- `SPEED_PI_EXPECTED_MAX_PPS = 5000`
- `SPEED_PI_MIN_PWM_PERCENT = 10`
- `SPEED_PI_MAX_PWM_PERCENT = 60`

A smaller KP or KI divisor makes that term stronger. Tune KP first with KI
made very weak, then increase KI only enough to remove steady-state error.

The current conversion assumes 500 PPR and rising-edge interrupts on both A
and B channels, giving 1000 counts per wheel revolution. Verify this by turning
each wheel exactly one revolution by hand. Replace `ENCODER_COUNTS_PER_REV` if
the measured count is different.

For this first test, measured speed uses the absolute encoder pulse rate. This
avoids left/right phase-order differences while tuning speed magnitude. Signed
encoder direction should be normalized before integrating this controller into
the full vehicle motion and line-tracking project.
