# JY61P I2C angle test

This CCS project follows the LCKFB Tianmengxing MSPM0G3507 JY61P example. It
uses GPIO software I2C for the sensor and hardware I2C1 for the OLED.

## Wiring

| JY61P | Tianmengxing MSPM0G3507 |
| --- | --- |
| VCC | 5V |
| GND | GND |
| SCL | PA1 |
| SDA | PA0 |

OLED wiring remains: VCC to 3.3V, GND to GND, SCL to PB2 and SDA to PB3.

The driver reads six bytes from JY61P I2C address `0x50`, starting at register
`0x3D`. The values are little-endian signed 16-bit roll, pitch and yaw values,
scaled by `raw / 32768 * 180 degrees`.

The startup path intentionally does not execute the example's zero-and-save
commands, so rebooting the board does not change the JY61P reference angle.
