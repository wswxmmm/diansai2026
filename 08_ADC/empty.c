#include "ti_msp_dl_config.h"
#include <stdio.h>

void uart0_send_string(const char *str);
unsigned int adc_getValue(void);

int main(void)
{
    char output_buff[50] = {0};
    unsigned int adc_value = 0;
    float voltage_value = 0;

    SYSCFG_DL_init();

    uart0_send_string("adc demo start\r\n");

    while (1) {
        adc_value = adc_getValue();
        sprintf(output_buff, "adc value:%u\r\n", adc_value);
        uart0_send_string(output_buff);

        voltage_value = adc_value / 4095.0f * 3.3f;
        sprintf(output_buff, "voltage value:%.2f\r\n", voltage_value);
        uart0_send_string(output_buff);

        delay_cycles(32000000);
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

unsigned int adc_getValue(void)
{
    unsigned int adc_result = 0;

    DL_ADC12_enableConversions(ADC_VOLTAGE_INST);
    DL_ADC12_startConversion(ADC_VOLTAGE_INST);

    while (DL_ADC12_getStatus(ADC_VOLTAGE_INST) != DL_ADC12_STATUS_CONVERSION_IDLE) {
    }

    DL_ADC12_stopConversion(ADC_VOLTAGE_INST);
    DL_ADC12_disableConversions(ADC_VOLTAGE_INST);

    adc_result = DL_ADC12_getMemResult(ADC_VOLTAGE_INST, ADC_VOLTAGE_ADCMEM_0);

    return adc_result;
}
