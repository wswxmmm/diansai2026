# 17_MPU6050_ANGLE_TEST

Standalone MPU6050 Z-axis gyro integration accuracy test for the
MSPM0G3507 board. This project does not drive either motor.

## Wiring

| Module signal | MSPM0G3507 pin |
| --- | --- |
| MPU6050 SDA | PA0 |
| MPU6050 SCL | PA1 |
| MPU6050 VCC | 3.3V |
| MPU6050 GND | GND |
| MPU6050 AD0 | GND for address 0x68 |
| OLED SCL | PB2 |
| OLED SDA | PB3 |
| OLED VCC/GND | 3.3V/GND |
| KEY1 | PB17, active-low |
| KEY2 | PB18, active-low |

The OLED and MPU6050 must share ground with the MSPM0G3507 board.

## Operation

Keep the sensor completely still during the five-second startup calibration.
The program accepts an MPU6050 only when `WHO_AM_I` is 0x68. The OLED displays:

- I2C address and WHO_AM_I value
- integrated signed Z-axis angle in degrees
- signed Z-axis angular rate in degrees per second
- peak absolute angle since reset
- communication error and valid sample counters

KEY1 resets the angle, peak and valid-sample counter. KEY2 initializes and
calibrates the MPU6050 again; keep the sensor still while it runs.

For an accuracy test, put the board flat, press KEY1, rotate it around the Z
axis by a measured 90, 180 or 360 degrees, then stop and compare the displayed
angle. Repeat clockwise and counter-clockwise. Also leave it still for 60
seconds after KEY1 to measure drift.

This is pure gyroscope integration. MPU6050 has no magnetometer, so absolute
heading cannot be corrected and long-term yaw drift is expected. Temperature,
calibration motion, axis tilt and rotation-time accuracy all affect the result.
