# 21_HC04_MASTER_NODE

HC-04 master node for one TMX MSPM0G3507 board. It sends
`MASTER PING n` once per second and expects `SLAVE ACK n` from the slave node.

## Wiring

| HC-04 master | TMX MSPM0G3507 |
| --- | --- |
| VCC | 3.3V for the bare HC-04 V2.5 module |
| GND | GND |
| TXD | PB16 / UART2 RX |
| RXD | PB15 / UART2 TX |

| SSD1306 OLED | TMX MSPM0G3507 |
| --- | --- |
| VCC | 3.3V |
| GND | GND |
| SCL | PA1 / I2C SCL |
| SDA | PA0 / I2C SDA |

UART2 runs at 9600 8N1. The firmware attempts to set `AT+ROLE=M` during
startup. If the module is already connected, AT mode may be unavailable; the
saved role is then used without interrupting transparent communication.

The OLED shows transmitted PING count, received ACK count, last acknowledged
sequence, and `WAITING`, `LINK OK`, or `TIMEOUT`.
