#include "ti_msp_dl_config.h"

volatile unsigned char uart_data = 0;

void uart0_send_char(char ch);
void uart0_send_string(const char *str);

int main(void)
{
    SYSCFG_DL_init();

    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);

    uart0_send_string("uart0 start work\r\n");

    while (1) {
        __WFI();
    }
}

void uart0_send_char(char ch)
{
    while (DL_UART_isBusy(UART_0_INST) == true) {
    }
    DL_UART_Main_transmitData(UART_0_INST, ch);
}

void uart0_send_string(const char *str)
{
    while ((str != 0) && (*str != 0)) {
        uart0_send_char(*str++);
    }
}

void UART_0_INST_IRQHandler(void)
{
    switch (DL_UART_getPendingInterrupt(UART_0_INST)) {
        case DL_UART_IIDX_RX:
            uart_data = DL_UART_Main_receiveData(UART_0_INST);
            uart0_send_char((char) uart_data);
            break;
        default:
            break;
    }
}
