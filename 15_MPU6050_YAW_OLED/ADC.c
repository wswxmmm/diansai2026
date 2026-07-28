#include "ADC.h"

unsigned int adc_getValue(void)
{
    unsigned int adc_result;

    DL_ADC12_enableConversions(GRAY_ADC_INST);
    DL_ADC12_startConversion(GRAY_ADC_INST);

    while (DL_ADC12_getStatus(GRAY_ADC_INST) != DL_ADC12_STATUS_CONVERSION_IDLE) {
    }

    DL_ADC12_stopConversion(GRAY_ADC_INST);
    DL_ADC12_disableConversions(GRAY_ADC_INST);

    adc_result = DL_ADC12_getMemResult(GRAY_ADC_INST, GRAY_ADC_ADCMEM_0);

    return adc_result;
}
