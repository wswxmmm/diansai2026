#include "ti_msp_dl_config.h"
#include "oled.h"

int main(void)
{
    SYSCFG_DL_init();

    OLED_Init();
    OLED_ColorTurn(0);
    OLED_DisplayTurn(0);

    while (1) {
        OLED_Clear();
        OLED_DrawRectangle(0, 0, 127, 63, 1);
        OLED_ShowString(8, 8, "LCKFB OLED", 16, 1);
        OLED_ShowString(8, 26, "MSPM0G3507", 8, 1);
        OLED_ShowString(8, 38, "I2C SSD1306", 8, 1);
        OLED_ShowString(8, 50, "PA1 SCL PA0 SDA", 8, 1);
        OLED_Refresh();
        delay_cycles(CPUCLK_FREQ);

        OLED_Clear();
        OLED_ShowString(0, 0, "0123456789", 8, 1);
        OLED_ShowString(0, 12, "ABCDEFGHIJ", 8, 1);
        OLED_ShowString(0, 24, "KLMNOPQRST", 8, 1);
        OLED_ShowString(0, 36, "UVWXYZ", 8, 1);
        OLED_ShowString(0, 50, "COUNT:", 8, 1);
        OLED_ShowNum(42, 50, 3507, 4, 8, 1);
        OLED_Refresh();
        delay_cycles(CPUCLK_FREQ);
    }
}
