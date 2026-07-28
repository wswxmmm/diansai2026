# 16_DUAL_MOTOR_TEST

Square black-line tracking test for the TB6612FNG and LCKFB MSPM0G3507 board.

Place the vehicle so both wheels can rotate freely before powering it on.

## Wiring

| TB6612 module | MSPM0G3507 board |
| --- | --- |
| GND | GND |
| VCC | 3.3V |
| VM | Motor power, 12V for the current motors |
| STBY | 3.3V |
| PWMA | PA16 / TIMA1_CCP1 |
| AIN1 | PA14 |
| AIN2 | PA15 |
| PWMB | PA17 / TIMA1_CCP0 |
| BIN1 | PA12 |
| BIN2 | PA13 |

Encoder and OLED connections:

| Signal | MSPM0G3507 pin |
| --- | --- |
| Left encoder A/B | PA7 / PA8 |
| Right encoder A/B | PB6 / PB8 |
| OLED SCL/SDA | PB2 / PB3 |

MPU6050 connections:

| MPU6050 signal | MSPM0G3507 pin |
| --- | --- |
| SDA | PA0 |
| SCL | PA1 |
| AD0 | GND, address 0x68 |
| VCC/GND | 3.3V/GND |

Eight-channel grayscale sensor connections:

| Sensor signal | MSPM0G3507 pin |
| --- | --- |
| AD0 | PB0 |
| AD1 | PB1 |
| AD2 | PB4 |
| ADC_OUT | PA27 / ADC0 channel 0 |

The grayscale ADC uses 12-bit raw values from 0 to 4095. Each displayed value
is the average of eight conversions after a discarded settling conversion.

The keys use internal pull-ups and are active-low. Connect the common key
terminal to GND.

| Key | MSPM0G3507 pin | Function |
| --- | --- | --- |
| KEY1 | PB17 | Start one lap |
| KEY2 | PB18 | Start two laps |
| KEY3 | PB19 | Start continuous tracking |
| KEY4 | PB20 | Emergency stop / idle |
| KEY5 | PB24 | Unused |
| KEY6 | PB25 | Unused |

The car follows values at or below 800 as black. Corner detection uses a
stricter threshold of 600: five or more channels must remain below it for five
consecutive samples. A wide-line corner candidate is retained for 200 ms; after
two samples it can still start a turn if the line then disappears, even if the
intermediate channel count drops gradually. Ordinary line loss has a 180 ms
grace period instead of stopping on one sample. If the line is still missing,
the car enters a `FIND` state rather than stopping. It then turns along a
tight forward arc, with the outside wheel moving faster than the inside wheel.
After
the center sensors leave the old line, three consecutive center-line detections
at 55 degrees or more complete the turn. The car then tracks at reduced speed
for 300 ms before returning to normal speed. It slows the outside wheel again at
70 degrees. If no new line is found by 130 degrees or within 6 seconds, it
changes to `FIND` and continues searching. Four corners count as one lap. The
default
ambiguous-corner direction is right; change
`DEFAULT_TURN_RIGHT` to -1 for a left-turn course.

Motor commands use a 0-100 percent range. Tracking uses 17 percent base speed.
Cornering uses 20/5 percent outside/inside wheel speeds, then 16/4 percent while
searching. The OLED header shows completed/target laps, progress through the
four corners of the current lap, and the car state. For example, `L:1/2 C:2/4`
means one lap is complete and the car has passed two corners of lap two. In
continuous mode the target is shown as `*`. The remaining rows show all eight
raw grayscale ADC values. Keep the car completely still during the MPU
calibration shown at startup.

KEY1 stops after one lap, KEY2 stops after two laps, and KEY3 keeps tracking
without a lap limit. In continuous mode the OLED target corner count is shown
as `*`. KEY4 always stops the vehicle and returns it to idle.

MPU6050 reads are retried three times. A transient read failure no longer stops
the motors, and a 500 ms time fallback allows line reacquisition even when gyro
samples are briefly unavailable.
