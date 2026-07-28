# 15_MPU6050_YAW_OLED

MPU6050 yaw angle + Ganv 8-channel no-MCU grayscale sensor display on SSD1306 OLED for the LCKFB/TMX MSPM0G3507 board.

## Wiring

OLED I2C:
- OLED SCL -> PB2
- OLED SDA -> PB3
- OLED VCC/GND -> 3.3V/GND

MPU6050 software I2C:
- MPU6050 SCL -> PA1
- MPU6050 SDA -> PA0
- MPU6050 VCC/GND -> 3.3V/GND
- MPU6050 AD0 -> GND, address 0x68

Ganv grayscale sensor:
- OUT -> PA27
- AD0 -> PB0
- AD1 -> PB1
- AD2 -> PB4
- VCC/GND -> stable sensor supply/GND

Keep the MPU6050 still during startup calibration. The grayscale driver uses default calibration values: white=1000, black=100 for all 8 channels. Tune `g_gray_white` and `g_gray_black` in `empty.c` if your track or lighting is different.

