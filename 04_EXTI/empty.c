#include "ti_msp_dl_config.h"

int main(void)
{
    SYSCFG_DL_init();
    NVIC_EnableIRQ(KEY_INT_IRQN);

    while (1) {
        __WFI();
    }
}

void GROUP1_IRQHandler(void)
{
    switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1)) {
        case KEY_INT_IIDX:
            if (DL_GPIO_readPins(KEY_PORT, KEY_PIN_21_PIN) == 0) {
                DL_GPIO_togglePins(LED1_PORT, LED1_PIN_22_PIN);
            }
            break;
        default:
            break;
    }
}
