# 09_DMA

ADC to memory DMA demo with UART output.

- UART0 TX/RX: PA10/PA11, 9600 baud
- ADC_VOLTAGE: ADC0 channel 2 on PA25
- DMA_CH0 copies ADC0 MEM0 results into `ADC_VALUE`.
