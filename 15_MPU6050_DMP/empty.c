#include "ti_msp_dl_config.h"
#include "delay.h"
#include "oled.h"
#include "mpu_port.h"
#include <stdio.h>

extern volatile uint32_t sys_tick_ms;

void SysTick_Handler(void)
{
    sys_tick_ms++;
}

int main(void)
{
    float pitch = 0.0f;
    float roll = 0.0f;
    float yaw = 0.0f;
    char text[32];

    SYSCFG_DL_init();

    OLED_Init();
    OLED_ColorTurn(0);
    OLED_DisplayTurn(0);
    OLED_Clear();
    OLED_ShowString(0, 0, "MPU6050 DMP", 16, 1);
    OLED_ShowString(0, 24, "INIT...", 16, 1);
    OLED_Refresh();

    while (DMP_Init() != 0) {
        DL_GPIO_togglePins(LED1_PORT, LED1_PIN_22_PIN);
        delay_ms(200);
    }

    OLED_Clear();
    OLED_ShowString(0, 0, "DMP READY", 16, 1);
    OLED_Refresh();
    delay_ms(500);

    while (1) {
        if (DMP_Read_Data(&pitch, &roll, &yaw) == 0) {
            OLED_Clear();

            snprintf(text, sizeof(text), "P:%6.2f", pitch);
            OLED_ShowString(0, 0, text, 16, 1);

            snprintf(text, sizeof(text), "R:%6.2f", roll);
            OLED_ShowString(0, 18, text, 16, 1);

            snprintf(text, sizeof(text), "Y:%6.2f", yaw);
            OLED_ShowString(0, 36, text, 16, 1);

            OLED_Refresh();
            DL_GPIO_togglePins(LED1_PORT, LED1_PIN_22_PIN);
        }
    }
}
