# 14_TB6612_ENCODER_OLED

TB6612 dual motor + dual GMR encoder normalized wheel speed display on SSD1306 OLED.

## Wiring

TB6612:
- STBY -> 3.3V
- VCC -> 5V0
- VM -> motor power
- GND -> common GND
- PWMA -> PA16, AIN1 -> PA14, AIN2 -> PA15
- PWMB -> PA17, BIN1 -> PA12, BIN2 -> PA13

OLED I2C:
- VCC -> 3.3V
- GND -> GND
- SCL -> PA1
- SDA -> PA0

Left GMR encoder:
- VCC -> 3.3V
- GND -> common GND
- A phase -> PA7
- B phase -> PA8

Right GMR encoder:
- VCC -> 3.3V
- GND -> common GND
- A phase -> PB6
- B phase -> PB8

The OLED shows left/right wheel speed in revolutions per second.
Motors default to 0 PWM. Change MOTOR_LEFT_SPEED and MOTOR_RIGHT_SPEED in empty.c.
GMR defaults: 500 PPR, two rising edges counted, so ENCODER_COUNTS_PER_REV is 1000.