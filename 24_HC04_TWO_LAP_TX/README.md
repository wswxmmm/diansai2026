# 24_HC04_TWO_LAP_TX

HC-04 transmitter for remotely starting project 23 in two-lap mode.

## Wiring

| Signal | MSPM0G3507 pin |
| --- | --- |
| HC-04 TXD | PB16 / UART2 RX |
| HC-04 RXD | PB15 / UART2 TX |
| HC-04 VCC | 3.3 V for the bare HC-04 V2.5 module |
| HC-04 GND | GND |
| Start button | PB21 to GND, internal pull-up enabled |

UART2 runs at 9600 baud. A debounced PB21 press sends
`START 2 <sequence>` up to ten times at 100 ms intervals. Project 23 replies
with `ACK START 2 <sequence>`; a matching acknowledgement stops retransmission.
The transmitter also sends `LINK PING <sequence>` once per second so project 23
can show live Bluetooth link status on its OLED.
The master attempts to select SPP master mode at startup and clears an old
paired address so it can search for the currently powered project 23 slave.
