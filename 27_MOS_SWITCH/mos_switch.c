#include "mos_switch.h"

#include <stdint.h>

#include "ti_msp_dl_config.h"

static bool s_is_on;

void MOS_Switch_Init(void)
{
    MOS_Switch_Off();
}

void MOS_Switch_On(void)
{
    DL_GPIO_setPins(MOS_CTRL_PORT, MOS_CTRL_CONTROL_PIN);
    DL_GPIO_setPins(STATUS_LED_PORT, STATUS_LED_LED_PIN);
    s_is_on = true;
}

void MOS_Switch_Off(void)
{
    /* Switch the load off before updating the indicator. */
    DL_GPIO_clearPins(MOS_CTRL_PORT, MOS_CTRL_CONTROL_PIN);
    DL_GPIO_clearPins(STATUS_LED_PORT, STATUS_LED_LED_PIN);
    s_is_on = false;
}

void MOS_Switch_Set(bool enabled)
{
    if (enabled) {
        MOS_Switch_On();
    } else {
        MOS_Switch_Off();
    }
}

bool MOS_Switch_IsOn(void)
{
    return s_is_on;
}
