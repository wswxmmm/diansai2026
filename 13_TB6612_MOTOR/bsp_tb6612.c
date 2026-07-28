#include "bsp_tb6612.h"

static uint32_t clamp_speed(uint32_t speed)
{
    return (speed > TB6612_MAX_SPEED) ? TB6612_MAX_SPEED : speed;
}

void TB6612_Motor_Stop(void)
{
    DL_TimerA_setCaptureCompareValue(PWM_0_INST, 0, GPIO_PWM_0_C0_IDX);
    DL_TimerA_setCaptureCompareValue(PWM_0_INST, 0, GPIO_PWM_0_C1_IDX);

    AIN1_OUT(1);
    AIN2_OUT(1);
    BIN1_OUT(1);
    BIN2_OUT(1);
}

void AO_Control(uint8_t dir, uint32_t speed)
{
    if (dir != 0U) {
        AIN1_OUT(0);
        AIN2_OUT(1);
    } else {
        AIN1_OUT(1);
        AIN2_OUT(0);
    }

    DL_TimerA_setCaptureCompareValue(PWM_0_INST, clamp_speed(speed), GPIO_PWM_0_C1_IDX);
}

void BO_Control(uint8_t dir, uint32_t speed)
{
    if (dir != 0U) {
        BIN1_OUT(0);
        BIN2_OUT(1);
    } else {
        BIN1_OUT(1);
        BIN2_OUT(0);
    }

    DL_TimerA_setCaptureCompareValue(PWM_0_INST, clamp_speed(speed), GPIO_PWM_0_C0_IDX);
}