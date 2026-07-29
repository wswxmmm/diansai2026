#include "ti_msp_dl_config.h"

#include "stepper_cm.h"

#include <stdbool.h>
#include <stdint.h>

#define CPU_CLOCK_HZ     32000000U
#define DEMO_DISTANCE_CM 1.0F

volatile uint8_t g_demoMotorId;
volatile bool g_demoLastOk;

static void app_delay_ms(uint32_t milliseconds)
{
    while (milliseconds-- > 0U) {
        delay_cycles(CPU_CLOCK_HZ / 1000U);
    }
}

static void blink_motor_number(uint8_t motorId)
{
    uint8_t i;

    for (i = 0U; i < motorId; i++) {
        DL_GPIO_setPins(LED1_PORT, LED1_PIN_22_PIN);
        app_delay_ms(120U);
        DL_GPIO_clearPins(LED1_PORT, LED1_PIN_22_PIN);
        app_delay_ms(120U);
    }
}

static bool run_selected_motor(uint8_t motorId)
{
    switch (motorId) {
        case 1U:
            return Motor1_MoveCm(DEMO_DISTANCE_CM);
        case 2U:
            return Motor2_MoveCm(DEMO_DISTANCE_CM);
        case 3U:
            return Motor3_MoveCm(DEMO_DISTANCE_CM);
        case 4U:
            return Motor4_MoveCm(DEMO_DISTANCE_CM);
        case 5U:
            return Motor5_MoveCm(DEMO_DISTANCE_CM);
        default:
            return false;
    }
}

int main(void)
{
    SYSCFG_DL_init();
    StepperCm_Init();

    g_demoMotorId = 1U;
    g_demoLastOk = false;
    DL_GPIO_clearPins(LED1_PORT, LED1_PIN_22_PIN);

    app_delay_ms(2000U);

    while (1) {
        if (DL_GPIO_readPins(KEY_PORT, KEY_PIN_21_PIN) == 0U) {
            app_delay_ms(20U);
            if (DL_GPIO_readPins(KEY_PORT, KEY_PIN_21_PIN) != 0U) {
                continue;
            }

            /* Each press tests the next motor: 1, 2, 3, 4, 5, then 1 again. */
            g_demoLastOk = run_selected_motor(g_demoMotorId);
            if (g_demoLastOk) {
                blink_motor_number(g_demoMotorId);
            } else {
                DL_GPIO_setPins(LED1_PORT, LED1_PIN_22_PIN);
            }

            g_demoMotorId++;
            if (g_demoMotorId > 5U) {
                g_demoMotorId = 1U;
            }

            while (DL_GPIO_readPins(KEY_PORT, KEY_PIN_21_PIN) == 0U) {
                app_delay_ms(1U);
            }
            app_delay_ms(20U);
        }
    }
}
