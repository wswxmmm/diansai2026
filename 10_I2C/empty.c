#include "ti_msp_dl_config.h"
#include "bsp_sht20.h"
#include <stdio.h>

void uart0_send_string(const char *str);

int main(void)
{
    char output_buff[100] = {0};
    float temperature = 0.0f;
    float humidity = 0.0f;

    SYSCFG_DL_init();
    uart0_send_string("SHT20 start\r\n");

    while (1) {
        temperature = SHT20_Read(SHT20_CMD_TEMP_NO_HOLD);
        humidity = SHT20_Read(SHT20_CMD_HUMIDITY_NO_HOLD);

        sprintf(output_buff, "temperature = %.2f C, humidity = %.0f %%RH\r\n", temperature, humidity);
        uart0_send_string(output_buff);

        delay_cycles(CPUCLK_FREQ);
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
