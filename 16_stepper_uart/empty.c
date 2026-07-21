#include "ti_msp_dl_config.h"
#include "motor_driver.h"
#include "motor_driver_config.h"

#include <stdint.h>

#define CPU_CLOCK_HZ 32000000U

typedef enum {
    APP_MODE_STOP = 0,
    APP_MODE_VELOCITY_CW,
    APP_MODE_VELOCITY_CCW,
    APP_MODE_POSITION_CW,
    APP_MODE_POSITION_CCW,
    APP_MODE_COUNT
} AppMotorMode;

/* Watch these variables in the CCS debugger. */
volatile uint32_t g_motorCommandSendCount;
volatile uint32_t g_motorSendErrorCount;
volatile uint8_t g_motorMode;
volatile bool g_motorLastCommandOk;

static void app_delay_ms(uint32_t milliseconds)
{
    while (milliseconds-- > 0U) {
        delay_cycles(CPU_CLOCK_HZ / 1000U);
    }
}

static void app_record_result(bool ok)
{
    g_motorLastCommandOk = ok;
    if (ok) {
        g_motorCommandSendCount++;
    } else {
        g_motorSendErrorCount++;
    }
}

static void app_blink_mode(AppMotorMode mode)
{
    uint8_t count = (uint8_t)mode;

    DL_GPIO_clearPins(LED1_PORT, LED1_PIN_22_PIN);
    while (count-- > 0U) {
        DL_GPIO_setPins(LED1_PORT, LED1_PIN_22_PIN);
        app_delay_ms(120U);
        DL_GPIO_clearPins(LED1_PORT, LED1_PIN_22_PIN);
        app_delay_ms(120U);
    }
}

static uint8_t app_motor2_direction(uint8_t direction)
{
    return (uint8_t)(direction ^ MOTOR_2_REVERSE_DIRECTION);
}

static void app_sync_stop(void)
{
    app_record_result(Motor_QueueStop(MOTOR_PORT_1, MOTOR_1_ADDRESS));
    app_delay_ms(10U);
    app_record_result(Motor_QueueStop(MOTOR_PORT_2, MOTOR_2_ADDRESS));
    app_delay_ms(10U);
    app_record_result(Motor_TriggerSynchronous(MOTOR_PORT_1));
    app_record_result(Motor_TriggerSynchronous(MOTOR_PORT_2));
}

static void app_sync_velocity(uint8_t direction)
{
    app_record_result(Motor_QueueVelocity(MOTOR_PORT_1, MOTOR_1_ADDRESS,
                                          direction,
                                          MOTOR_TEST_ACCEL,
                                          (float)MOTOR_TEST_SPEED_RPM));
    app_delay_ms(10U);
    app_record_result(Motor_QueueVelocity(MOTOR_PORT_2, MOTOR_2_ADDRESS,
                                          app_motor2_direction(direction),
                                          MOTOR_TEST_ACCEL,
                                          (float)MOTOR_TEST_SPEED_RPM));
    app_delay_ms(10U);
    app_record_result(Motor_TriggerSynchronous(MOTOR_PORT_1));
    app_record_result(Motor_TriggerSynchronous(MOTOR_PORT_2));
}

static void app_sync_position(uint8_t direction)
{
    app_record_result(Motor_QueueMoveRelativeDegrees(
        MOTOR_PORT_1, MOTOR_1_ADDRESS, direction,
        (float)MOTOR_TEST_SPEED_RPM, MOTOR_TEST_ACCEL,
        MOTOR_POSITION_DEGREES));
    app_delay_ms(10U);
    app_record_result(Motor_QueueMoveRelativeDegrees(
        MOTOR_PORT_2, MOTOR_2_ADDRESS, app_motor2_direction(direction),
        (float)MOTOR_TEST_SPEED_RPM, MOTOR_TEST_ACCEL,
        MOTOR_POSITION_DEGREES));
    app_delay_ms(10U);
    app_record_result(Motor_TriggerSynchronous(MOTOR_PORT_1));
    app_record_result(Motor_TriggerSynchronous(MOTOR_PORT_2));
}

static void app_execute_mode(AppMotorMode mode)
{
    /* Stop first so a velocity command cannot remain active during switching. */
    app_sync_stop();
    app_delay_ms(80U);

    switch (mode) {
        case APP_MODE_STOP:
            /* Stop keeps the motor enabled, so it may still have holding torque. */
            break;

        case APP_MODE_VELOCITY_CW:
            app_sync_velocity(0U);
            break;

        case APP_MODE_VELOCITY_CCW:
            app_sync_velocity(1U);
            break;

        case APP_MODE_POSITION_CW:
            app_sync_position(0U);
            app_delay_ms(MOTOR_POSITION_STOP_MS);
            app_sync_stop();
            break;

        case APP_MODE_POSITION_CCW:
            app_sync_position(1U);
            app_delay_ms(MOTOR_POSITION_STOP_MS);
            app_sync_stop();
            break;

        default:
            break;
    }

    g_motorMode = (uint8_t)mode;
    app_blink_mode(mode);
}

int main(void)
{
    SYSCFG_DL_init();
    Motor_Init();

    g_motorMode = (uint8_t)APP_MODE_STOP;
    g_motorLastCommandOk = false;
    DL_GPIO_clearPins(LED1_PORT, LED1_PIN_22_PIN);

    /* Give older X42 drivers enough time to finish their power-on setup. */
    app_delay_ms(2000U);

    app_record_result(Motor_Enable(MOTOR_PORT_1, MOTOR_1_ADDRESS, true));
    app_delay_ms(100U);
    app_record_result(Motor_Enable(MOTOR_PORT_2, MOTOR_2_ADDRESS, true));
    app_delay_ms(100U);
    app_execute_mode(APP_MODE_STOP);

    while (1) {
        if (DL_GPIO_readPins(KEY_PORT, KEY_PIN_21_PIN) == 0U) {
            AppMotorMode nextMode;

            app_delay_ms(20U);
            if (DL_GPIO_readPins(KEY_PORT, KEY_PIN_21_PIN) != 0U) {
                continue;
            }

            nextMode = (AppMotorMode)(g_motorMode + 1U);
            if (nextMode >= APP_MODE_COUNT) {
                nextMode = APP_MODE_STOP;
            }
            app_execute_mode(nextMode);

            /* One physical press produces exactly one mode change. */
            while (DL_GPIO_readPins(KEY_PORT, KEY_PIN_21_PIN) == 0U) {
                app_delay_ms(1U);
            }
            app_delay_ms(20U);
        }
    }
}
