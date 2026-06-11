#include "ti_msp_dl_config.h"

int main(void)
{
    int i = 0;

    SYSCFG_DL_init();
    DL_TimerG_startCounter(PWM_LED_INST);

    while (1) {
        for (i = 0; i <= 999; i++) {
            DL_TimerG_setCaptureCompareValue(PWM_LED_INST, i, GPIO_PWM_LED_C1_IDX);
            delay_cycles(32000);
        }

        for (i = 999; i > 0; i--) {
            DL_TimerG_setCaptureCompareValue(PWM_LED_INST, i, GPIO_PWM_LED_C1_IDX);
            delay_cycles(32000);
        }
    }
}
