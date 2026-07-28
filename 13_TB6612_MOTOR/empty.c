#include "ti_msp_dl_config.h"
#include "bsp_tb6612.h"

int main(void)
{
    SYSCFG_DL_init();
    TB6612_Motor_Stop();
    DL_TimerA_startCounter(PWM_0_INST);

    while (1) {
        for (uint32_t speed = 0; speed <= TB6612_MAX_SPEED; speed += 50) {
            AO_Control(1, speed);
            BO_Control(1, speed);
            delay_cycles(CPUCLK_FREQ / 2);
        }

        TB6612_Motor_Stop();
        delay_cycles(CPUCLK_FREQ);

        for (uint32_t speed = 0; speed <= TB6612_MAX_SPEED; speed += 50) {
            AO_Control(0, speed);
            BO_Control(0, speed);
            delay_cycles(CPUCLK_FREQ / 2);
        }

        TB6612_Motor_Stop();
        delay_cycles(CPUCLK_FREQ);
    }
}