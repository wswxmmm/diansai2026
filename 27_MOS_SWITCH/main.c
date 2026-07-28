#include <stdbool.h>
#include <stdint.h>

#include "ti_msp_dl_config.h"
#include "mos_switch.h"

/* B21 is the on-board active-low user key. */
static bool user_key_is_pressed(void)
{
    return (DL_GPIO_readPins(USER_KEY_PORT, USER_KEY_BUTTON_PIN) &
               USER_KEY_BUTTON_PIN) == 0U;
}

int main(void)
{
    bool first_sample;
    bool second_sample;

    SYSCFG_DL_init();
    MOS_Switch_Init();

    while (1) {
        /* Require two pressed samples 10 ms apart for simple debouncing. */
        first_sample = user_key_is_pressed();
        delay_cycles(CPUCLK_FREQ / 100U);
        second_sample = user_key_is_pressed();

        /* Fail-safe hold-to-run control: release the key to switch off. */
        MOS_Switch_Set(first_sample && second_sample);
    }
}
