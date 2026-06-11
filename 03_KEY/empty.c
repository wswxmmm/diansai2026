#include "ti_msp_dl_config.h"

int main(void)
{
    SYSCFG_DL_init();

    while (1) {
        if (DL_GPIO_readPins(KEY_PORT, KEY_PIN_21_PIN) == 0) {
            DL_GPIO_setPins(LED1_PORT, LED1_PIN_22_PIN);
        } else {
            DL_GPIO_clearPins(LED1_PORT, LED1_PIN_22_PIN);
        }
    }
}
