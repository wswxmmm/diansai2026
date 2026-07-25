#include "bsp_tb6612.h"

static uint32_t speed_percent_to_compare(uint32_t speed_percent)
{
    if (speed_percent > TB6612_MAX_SPEED_PERCENT) {
        speed_percent = TB6612_MAX_SPEED_PERCENT;
    }

    return TB6612_PWM_PERIOD_COUNT -
        (((speed_percent * TB6612_PWM_PERIOD_COUNT) + 50U) /
            TB6612_MAX_SPEED_PERCENT);
}

void TB6612_Motor_Stop(void)
{
    AO_Stop();
    BO_Stop();
}

void AO_Stop(void)
{
    DL_TimerA_setCaptureCompareValue(
        PWM_0_INST, TB6612_PWM_PERIOD_COUNT, GPIO_PWM_0_C1_IDX);
    AIN1_OUT(1);
    AIN2_OUT(1);
}

void BO_Stop(void)
{
    DL_TimerA_setCaptureCompareValue(
        PWM_0_INST, TB6612_PWM_PERIOD_COUNT, GPIO_PWM_0_C0_IDX);
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

    DL_TimerA_setCaptureCompareValue(PWM_0_INST,
        speed_percent_to_compare(speed), GPIO_PWM_0_C1_IDX);
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

    DL_TimerA_setCaptureCompareValue(PWM_0_INST,
        speed_percent_to_compare(speed), GPIO_PWM_0_C0_IDX);
}
