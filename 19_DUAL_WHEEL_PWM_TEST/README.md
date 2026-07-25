# 19_DUAL_WHEEL_PWM_TEST

Open-loop dual-motor PWM and encoder speed measurement test. No PI or PID
controller is enabled in this project.

Lift both drive wheels off the ground before the first test.

## Keys

| Key | Function |
| --- | --- |
| KEY1 / PB17 | Left PWM +1 percent |
| KEY2 / PB18 | Left PWM -1 percent |
| KEY3 / PB19 | Right PWM +1 percent |
| KEY4 / PB20 | Right PWM -1 percent |
| KEY5 / PB24 | Start both wheels forward |
| KEY6 / PB25 | Stop both wheels immediately |

The default left and right commands are both 12 percent. The allowed test
range is 0 to 60 percent. PWM values can be adjusted while stopped or running.

## OLED

- `P` is the PWM command in percent.
- `S` is signed encoder speed in revolutions per second.
- `DL` and `DR` are raw pulse increments during the latest 100 ms window.
- `CPR` is the configured encoder count per wheel revolution.

The current configuration assumes 1000 counts per wheel revolution. Turn each
wheel exactly one revolution by hand and verify the total count before relying
on the displayed revolutions per second.

The right encoder polarity is inverted in software so both displayed speeds
are positive when the vehicle moves forward.

Record the lowest PWM at which each wheel starts reliably, then record both
wheel speeds at 12, 15, 20, 25 and 30 percent. These measurements are the
feed-forward and dead-zone data needed for the next PI-control step.
