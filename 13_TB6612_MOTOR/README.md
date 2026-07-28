# 13_TB6612_MOTOR

TB6612FNG dual DC motor demo for the LCKFB MSPM0G3507 board in CCS Theia.

## Wiring

| TB6612 module | MSPM0G3507 board |
| --- | --- |
| GND | GND |
| VCC | 5V0 |
| VM | Motor power, 3.7V to 12V |
| STBY | 3.3V |
| PWMA | PA16 / TIMA1_CCP1 |
| AIN1 | PA14 |
| AIN2 | PA15 |
| PWMB | PA17 / TIMA1_CCP0 |
| BIN1 | PA12 |
| BIN2 | PA13 |

AO_Control(dir, speed) controls motor A. BO_Control(dir, speed) controls motor B.
The speed range is 0 to 999. Change dir between 0 and 1 to reverse direction.

The demo in empty.c ramps both motors forward, stops, ramps both motors backward, and repeats.