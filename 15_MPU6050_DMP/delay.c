#include "delay.h"

void delay_ms(uint32_t ms)
{
    while (ms-- > 0U) {
        delay_cycles(CPUCLK_FREQ / 1000U);
    }
}
