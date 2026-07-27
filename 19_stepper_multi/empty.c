#include "ti_msp_dl_config.h"
#include "multi_motor_config.h"
#include "emm_multi_motor.h"

#include <stdint.h>

#define CPU_CLOCK_HZ 32000000U

static const uint8_t g_motorAddresses[MOTOR_COUNT] = {
    MOTOR_1_ADDRESS, MOTOR_2_ADDRESS, MOTOR_3_ADDRESS
};

static const uint8_t g_motorDirectionXor[MOTOR_COUNT] = {
    MOTOR_1_DIRECTION_XOR,
    MOTOR_2_DIRECTION_XOR,
    MOTOR_3_DIRECTION_XOR
};

volatile uint8_t g_multiMotorMode;
volatile bool g_multiLastOperationOk;

static void app_delay_ms(uint32_t milliseconds)
{
    while (milliseconds-- > 0U) {
        delay_cycles(CPU_CLOCK_HZ / 1000U);
    }
}

static uint8_t motor_direction(uint8_t motorIndex, uint8_t direction)
{
    return (uint8_t)(direction ^ g_motorDirectionXor[motorIndex]);
}

static void app_blink_motor(uint8_t motorNumber)
{
    uint8_t count = motorNumber;

    DL_GPIO_clearPins(LED1_PORT, LED1_PIN_22_PIN);
    while (count-- > 0U) {
        DL_GPIO_setPins(LED1_PORT, LED1_PIN_22_PIN);
        app_delay_ms(120U);
        DL_GPIO_clearPins(LED1_PORT, LED1_PIN_22_PIN);
        app_delay_ms(120U);
    }
}

static bool app_enable_all(void)
{
    uint8_t i;
    bool ok = true;

    for (i = 0U; i < MOTOR_COUNT; i++) {
        ok = EmmMulti_Enable(g_motorAddresses[i], true) && ok;
        app_delay_ms(20U);
    }
    return ok;
}

static bool app_sync_stop_all(void)
{
    uint8_t i;
    bool ok = true;

    for (i = 0U; i < MOTOR_COUNT; i++) {
        ok = EmmMulti_QueueStop(g_motorAddresses[i]) && ok;
        app_delay_ms(10U);
    }
    return EmmMulti_TriggerSynchronous() && ok;
}

static void app_run_all_motors(void)
{
    uint8_t i;
    bool ok;

    ok = app_sync_stop_all();

    app_delay_ms(80U);

    for (i = 0U; i < MOTOR_COUNT; i++) {
        ok = EmmMulti_QueuePosition(
                 g_motorAddresses[i], motor_direction(i, 0U),
                 MOTOR_TEST_SPEED_RPM, MOTOR_TEST_ACCEL,
                 MOTOR_TEST_POSITION_PULSES) && ok;
        app_delay_ms(10U);
    }
    ok = EmmMulti_TriggerSynchronous() && ok;

    app_delay_ms(MOTOR_POSITION_STOP_MS);
    ok = app_sync_stop_all() && ok;

    g_multiLastOperationOk = ok;
    g_multiMotorMode = MOTOR_COUNT;
    app_blink_motor(g_multiMotorMode);
}

int main(void)
{
    SYSCFG_DL_init();
    EmmMulti_Init();

    g_multiMotorMode = 0U;
    g_multiLastOperationOk = false;
    DL_GPIO_clearPins(LED1_PORT, LED1_PIN_22_PIN);

    app_delay_ms(2000U);
    g_multiLastOperationOk = app_enable_all();
    app_delay_ms(100U);
    g_multiLastOperationOk = app_sync_stop_all() && g_multiLastOperationOk;

    while (1) {
        if (DL_GPIO_readPins(KEY_PORT, KEY_PIN_21_PIN) == 0U) {
            app_delay_ms(20U);
            if (DL_GPIO_readPins(KEY_PORT, KEY_PIN_21_PIN) != 0U) {
                continue;
            }

            app_run_all_motors();

            while (DL_GPIO_readPins(KEY_PORT, KEY_PIN_21_PIN) == 0U) {
                app_delay_ms(1U);
            }
            app_delay_ms(20U);
        }
    }
}
