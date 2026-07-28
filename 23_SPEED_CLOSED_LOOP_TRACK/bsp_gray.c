#include "bsp_gray.h"

#define GRAY_AVERAGE_SAMPLES   (8U)
#define GRAY_SETTLE_CYCLES     (CPUCLK_FREQ / 100000U)

static void gray_set_address(uint8_t channel)
{
    if ((channel & 0x01U) == 0U) {
        DL_GPIO_setPins(GRAY_ADDRESS_PORT, GRAY_ADDRESS_AD0_PIN);
    } else {
        DL_GPIO_clearPins(GRAY_ADDRESS_PORT, GRAY_ADDRESS_AD0_PIN);
    }

    if ((channel & 0x02U) == 0U) {
        DL_GPIO_setPins(GRAY_ADDRESS_PORT, GRAY_ADDRESS_AD1_PIN);
    } else {
        DL_GPIO_clearPins(GRAY_ADDRESS_PORT, GRAY_ADDRESS_AD1_PIN);
    }

    if ((channel & 0x04U) == 0U) {
        DL_GPIO_setPins(GRAY_ADDRESS_PORT, GRAY_ADDRESS_AD2_PIN);
    } else {
        DL_GPIO_clearPins(GRAY_ADDRESS_PORT, GRAY_ADDRESS_AD2_PIN);
    }
}

static uint16_t gray_adc_read_once(void)
{
    uint16_t result;

    DL_ADC12_enableConversions(GRAY_ADC_INST);
    DL_ADC12_startConversion(GRAY_ADC_INST);

    while (DL_ADC12_getStatus(GRAY_ADC_INST) !=
        DL_ADC12_STATUS_CONVERSION_IDLE) {
    }

    DL_ADC12_stopConversion(GRAY_ADC_INST);
    DL_ADC12_disableConversions(GRAY_ADC_INST);
    result = (uint16_t) DL_ADC12_getMemResult(
        GRAY_ADC_INST, GRAY_ADC_ADCMEM_0);

    return result;
}

void GraySensor_Read(uint16_t values[GRAY_SENSOR_CHANNELS])
{
    uint32_t channel;

    for (channel = 0; channel < GRAY_SENSOR_CHANNELS; channel++) {
        uint32_t sample;
        uint32_t sum = 0U;

        gray_set_address((uint8_t) channel);
        delay_cycles(GRAY_SETTLE_CYCLES);

        (void) gray_adc_read_once();
        for (sample = 0; sample < GRAY_AVERAGE_SAMPLES; sample++) {
            sum += gray_adc_read_once();
        }

        values[(GRAY_SENSOR_CHANNELS - 1U) - channel] =
            (uint16_t) (sum / GRAY_AVERAGE_SAMPLES);
    }
}
