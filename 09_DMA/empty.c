#include "ti_msp_dl_config.h"
#include <stdio.h>

#define ADC_SAMPLE_COUNT (20U)

volatile uint16_t ADC_VALUE[ADC_SAMPLE_COUNT];

void uart0_send_string(const char *str);
unsigned int adc_getValue(unsigned int number);

int main(void)
{
    char output_buff[50] = {0};
    unsigned int adc_value = 0;
    float voltage_value = 0;

    SYSCFG_DL_init();

    DL_DMA_setSrcAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t) &ADC0->ULLMEM.MEMRES[0]);
    DL_DMA_setDestAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t) &ADC_VALUE[0]);
    DL_DMA_setTransferSize(DMA, DMA_CH0_CHAN_ID, ADC_SAMPLE_COUNT);
    DL_DMA_enableChannel(DMA, DMA_CH0_CHAN_ID);

    DL_ADC12_enableConversions(ADC_VOLTAGE_INST);
    DL_ADC12_startConversion(ADC_VOLTAGE_INST);

    uart0_send_string("adc dma demo start\r\n");

    while (1) {
        adc_value = adc_getValue(10);
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

unsigned int adc_getValue(unsigned int number)
{
    unsigned int adc_result = 0;
    unsigned int i = 0;

    if (number > ADC_SAMPLE_COUNT) {
        number = ADC_SAMPLE_COUNT;
    }

    for (i = 0; i < number; i++) {
        adc_result += ADC_VALUE[i];
    }

    return adc_result / number;
}
