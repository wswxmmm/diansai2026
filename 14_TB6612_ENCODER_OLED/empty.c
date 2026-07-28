#include "ti_msp_dl_config.h"
#include "bsp_tb6612.h"
#include "bsp_encoder.h"
#include "oled.h"
#include <stdio.h>

#define MOTOR_LEFT_SPEED   (650U)
#define MOTOR_RIGHT_SPEED  (650U)
#define MOTOR_START_SPEED  (900U)

static void show_rps(uint8_t x, uint8_t y, int32_t rps_milli)
{
    char text[12];
    int32_t value = rps_milli;

    if (value < 0) {
        value = -value;
        (void) snprintf(text, sizeof(text), "-%ld.%03ld",
            (long) (value / SPEED_DECIMAL_SCALE),
            (long) (value % SPEED_DECIMAL_SCALE));
    } else {
        (void) snprintf(text, sizeof(text), "%ld.%03ld",
            (long) (value / SPEED_DECIMAL_SCALE),
            (long) (value % SPEED_DECIMAL_SCALE));
    }

    OLED_ShowString(x, y, text, 16, 1);
}

static void OLED_ShowWheelSpeed(const WheelSpeed *left, const WheelSpeed *right)
{
    OLED_Clear();
    OLED_DrawRectangle(0, 0, 127, 63, 1);

    OLED_ShowString(8, 6, "REV PER SEC", 8, 1);

    OLED_ShowString(8, 22, "L:", 16, 1);
    show_rps(34, 22, left->rps_milli);

    OLED_ShowString(8, 42, "R:", 16, 1);
    show_rps(34, 42, right->rps_milli);

    OLED_Refresh();
}

int main(void)
{
    SYSCFG_DL_init();

    OLED_Init();
    OLED_ColorTurn(0);
    OLED_DisplayTurn(0);

    Encoder_Init();
    TB6612_Motor_Stop();
    DL_TimerA_startCounter(PWM_0_INST);

    AO_Control(1, MOTOR_START_SPEED);
    BO_Control(1, MOTOR_START_SPEED);
    delay_cycles(CPUCLK_FREQ / 2U);

    AO_Control(1, MOTOR_LEFT_SPEED);
    BO_Control(1, MOTOR_RIGHT_SPEED);

    while (1) {
        WheelSpeed left = Encoder_UpdateSpeed(ENCODER_LEFT);
        WheelSpeed right = Encoder_UpdateSpeed(ENCODER_RIGHT);

        OLED_ShowWheelSpeed(&left, &right);
        delay_cycles(CPUCLK_FREQ / (1000U / ENCODER_SAMPLE_MS));
    }
}