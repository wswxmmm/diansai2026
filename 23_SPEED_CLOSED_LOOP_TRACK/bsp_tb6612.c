#include "bsp_tb6612.h"

static uint32_t duty_permille_to_compare(uint32_t duty_permille)
{
    if (duty_permille > TB6612_MAX_DUTY_PERMILLE) {
        duty_permille = TB6612_MAX_DUTY_PERMILLE;
    }

    return TB6612_PWM_PERIOD_COUNT -
        (((duty_permille * TB6612_PWM_PERIOD_COUNT) + 500U) /
            TB6612_MAX_DUTY_PERMILLE);
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

void AO_Coast(void)
{
    DL_TimerA_setCaptureCompareValue(
        PWM_0_INST, TB6612_PWM_PERIOD_COUNT, GPIO_PWM_0_C1_IDX);
    AIN1_OUT(0);
    AIN2_OUT(0);
}

void BO_Coast(void)
{
    DL_TimerA_setCaptureCompareValue(
        PWM_0_INST, TB6612_PWM_PERIOD_COUNT, GPIO_PWM_0_C0_IDX);
    BIN1_OUT(0);
    BIN2_OUT(0);
}

void AO_Control(uint8_t dir, uint32_t speed)
{
    if (speed > TB6612_MAX_SPEED_PERCENT) {
        speed = TB6612_MAX_SPEED_PERCENT;
    }
    AO_ControlPermille(dir, speed * 10U);
}

void BO_Control(uint8_t dir, uint32_t speed)
{
    if (speed > TB6612_MAX_SPEED_PERCENT) {
        speed = TB6612_MAX_SPEED_PERCENT;
    }
    BO_ControlPermille(dir, speed * 10U);
}

void AO_ControlPermille(uint8_t dir, uint32_t duty_permille)
{
    if (dir != 0U) {
        AIN1_OUT(0);
        AIN2_OUT(1);
    } else {
        AIN1_OUT(1);
        AIN2_OUT(0);
    }

    DL_TimerA_setCaptureCompareValue(PWM_0_INST,
        duty_permille_to_compare(duty_permille), GPIO_PWM_0_C1_IDX);
}

void BO_ControlPermille(uint8_t dir, uint32_t duty_permille)
{
    if (dir != 0U) {
        BIN1_OUT(0);
        BIN2_OUT(1);
    } else {
        BIN1_OUT(1);
        BIN2_OUT(0);
    }

    DL_TimerA_setCaptureCompareValue(PWM_0_INST,
        duty_permille_to_compare(duty_permille), GPIO_PWM_0_C0_IDX);
}
