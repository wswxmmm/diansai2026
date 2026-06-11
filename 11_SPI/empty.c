#include "ti_msp_dl_config.h"
#include "bsp_w25q128.h"
#include <stdio.h>

#define delay_ms(x) delay_cycles((CPUCLK_FREQ / 1000U) * (x))

void uart0_send_string(const char *str);

int main(void)
{
    char uart_output_buff[64] = {0};
    unsigned char read_write_buff[10] = {0};

    SYSCFG_DL_init();
    SPI_CS(1);
    delay_ms(1);

    sprintf(uart_output_buff, "ID = %X\r\n", W25Q128_readID());
    uart0_send_string(uart_output_buff);

    W25Q128_read(read_write_buff, 0, 5);
    read_write_buff[5] = 0;
    sprintf(uart_output_buff, "before = %s\r\n", read_write_buff);
    uart0_send_string(uart_output_buff);

    W25Q128_write((uint8_t *) "lckfb", 0, 5);
    delay_ms(10);

    W25Q128_read(read_write_buff, 0, 5);
    read_write_buff[5] = 0;
    sprintf(uart_output_buff, "after = %s\r\n", read_write_buff);
    uart0_send_string(uart_output_buff);

    while (1) {
    }
}

void uart0_send_string(const char *str)
{
    while ((str != 0) && (*str != 0)) {
        while (DL_UART_isBusy(UART_0_INST) == true) {
        }
        DL_UART_Main_transmitData(UART_0_INST, *str++);
    }
}
