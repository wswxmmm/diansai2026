# 23_SPEED_CLOSED_LOOP_TRACK

Square black-line tracking with dual-wheel closed-loop speed control for the
TB6612FNG and LCKFB MSPM0G3507 board. This project keeps the complete project 16
tracking, corner, lap, key and OLED state machine, and inserts the tuned project
20 PID controller between its wheel-speed commands and the motor PWM outputs.

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
| KEY6 | PB25 | Run the ten-second MPU straight-line test |

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

Motor commands are closed-loop speed targets. One command unit is 1000 encoder
pulses per second, so tracking uses a target of 17 (17000 pulses/s). Cornering
uses 20/5 outside/inside targets, then 16/4 while searching. The two wheel PID
loops run every 10 ms with the tuned project 20 gains KP=0.020, KI=0.030 and
KD=0.000. PWM output is limited to 60 percent and includes acceleration target
ramping, immediate target reduction for responsive corner entry, speed filtering,
feed-forward, low-output pulse-density drive, integral limiting and anti-windup.
The OLED header shows completed/target laps, progress through the
four corners of the current lap, and the car state. For example, `L:1/2 C:2/4`
means one lap is complete and the car has passed two corners of lap two. In
continuous mode the target is shown as `*`. The remaining rows show all eight
raw grayscale ADC values. Keep the car completely still during the MPU
calibration shown at startup.

KEY1 stops after one lap, KEY2 stops after two laps, and KEY3 keeps tracking
without a lap limit. In continuous mode the OLED target corner count is shown
as `*`. KEY4 always stops the vehicle and returns it to idle.

KEY6 starts a standalone straight-line test. Both wheels use a closed-loop
target of 12 (12000 encoder pulses/s with the current five-times speed scale)
for ten seconds, then stop automatically. The calibrated MPU6050 Z gyro is
integrated into heading error; approximately every three degrees of error adds
one target unit to one wheel and removes one from the other, up to a correction
of four. During the test the OLED header shows `LINE` and the signed yaw error
in degrees, for example `B+ Y:-2 LINE`. If the car consistently corrects in the
wrong direction because of sensor mounting orientation, change
`STRAIGHT_GYRO_CORRECTION_SIGN` from `1` to `-1`.

An HC-04 receiver is connected through UART2 at 9600 baud. Connect HC-04 TX to
PB16 (MCU RX), HC-04 RX to PB15 (MCU TX), and share 3.3 V/GND. A received
`START 2 <sequence>` line starts the same two-lap path as local KEY2. The car
replies with `ACK START 2 <sequence>` and ignores short-interval retransmissions
of the same sequence so that its lap count is not repeatedly reset.

Project 24 sends `LINK PING <sequence>` once per second and project 23 replies
with `LINK ACK <sequence>`. The first OLED header character reports the link:
`B+` means heartbeat received within 2.5 seconds, `B?` means no valid heartbeat
has ever arrived, and `B-` means a previously active link has timed out. The
compact header still shows completed/target laps, current corner and car state.

MPU6050 reads are retried three times. A transient read failure no longer stops
the motors, and a 500 ms time fallback allows line reacquisition even when gyro
samples are briefly unavailable.
