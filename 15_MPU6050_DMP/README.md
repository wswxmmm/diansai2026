# 15_MPU6050_DMP

MPU6050 DMP angle demo for the TMX MSPM0G3507 board.

- Shared I2C bus: PA1 = SCL, PA0 = SDA
- MPU6050 address: 0x68 when AD0/ADDR is connected to GND
- OLED address: 0x3C
- Status LED: PB22
- SysTick: 1 ms tick at the default 32 MHz CPU clock

The project reuses the InvenSense MPU6050 DMP driver and adapts its platform layer to MSPM0 DriverLib. The OLED shows pitch, roll, and yaw after DMP initialization succeeds.
