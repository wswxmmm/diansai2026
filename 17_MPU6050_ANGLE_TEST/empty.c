#include "ti_msp_dl_config.h"
#include "bsp_mpu6050_sw.h"
#include "oled.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define MPU_SAMPLE_PERIOD_MS       (10U)
#define OLED_UPDATE_PERIOD_MS      (200U)
#define KEY_POLL_PERIOD_MS         (10U)
#define KEY_DEBOUNCE_SAMPLES       (3U)
#define GYRO_DEADBAND_MDPS         (200)
#define MPU6050_EXPECTED_WHO_AM_I  (0x68U)

typedef struct {
    uint8_t raw;
    uint8_t stable;
    uint8_t samples;
} KeyState;

static volatile uint32_t g_milliseconds;
static KeyState g_key1;
static KeyState g_key2;
static bool g_mpuReady;
static int32_t g_gyroZMdps;
static int32_t g_yawMdeg;
static int32_t g_peakAbsYawMdeg;
static int64_t g_integrationRemainder;
static uint32_t g_validSamples;
static uint32_t g_commErrors;

void SysTick_Handler(void)
{
    g_milliseconds++;
}

static int32_t abs_i32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static uint8_t key_pressed(uint32_t pin)
{
    return (DL_GPIO_readPins(KEYS_PORT, pin) == 0U) ? 1U : 0U;
}

static bool key_update(KeyState *state, uint8_t pressed)
{
    if (pressed != state->raw) {
        state->raw = pressed;
        state->samples = 1U;
        return false;
    }

    if (state->samples < KEY_DEBOUNCE_SAMPLES) {
        state->samples++;
    }
    if ((state->samples == KEY_DEBOUNCE_SAMPLES) &&
        (state->stable != pressed)) {
        state->stable = pressed;
        return (pressed != 0U);
    }
    return false;
}

static void keys_init(void)
{
    g_key1.raw = key_pressed(KEYS_KEY1_PIN);
    g_key1.stable = g_key1.raw;
    g_key1.samples = KEY_DEBOUNCE_SAMPLES;
    g_key2.raw = key_pressed(KEYS_KEY2_PIN);
    g_key2.stable = g_key2.raw;
    g_key2.samples = KEY_DEBOUNCE_SAMPLES;
}

static void reset_angle(void)
{
    g_yawMdeg = 0;
    g_peakAbsYawMdeg = 0;
    g_integrationRemainder = 0;
    g_validSamples = 0U;
}

static void oled_show_calibration(void)
{
    OLED_Clear();
    OLED_DrawRectangle(0, 0, 127, 63, 1);
    OLED_ShowString(16, 8, "MPU CAL", 16, 1);
    OLED_ShowString(28, 34, "KEEP STILL", 8, 1);
    OLED_ShowString(34, 48, "5 SECONDS", 8, 1);
    OLED_Refresh();
}

static void oled_show_failure(void)
{
    char line[20];

    OLED_Clear();
    OLED_DrawRectangle(0, 0, 127, 63, 1);
    OLED_ShowString(20, 7, "MPU ERROR", 16, 1);
    (void) snprintf(line, sizeof(line), "ADDR:%02X WHO:%02X",
        MPU6050_SW_GetAddress(), MPU6050_SW_GetWhoAmI());
    OLED_ShowString(8, 34, line, 8, 1);
    OLED_ShowString(28, 49, "CHECK / K2", 8, 1);
    OLED_Refresh();
}

static void format_signed_tenths(char *text, size_t size,
    const char *label, int32_t value_milli)
{
    uint32_t tenths;
    char sign = '+';

    if (value_milli < 0) {
        sign = '-';
        value_milli = -value_milli;
    }
    tenths = ((uint32_t) value_milli + 50U) / 100U;
    (void) snprintf(text, size, "%s:%c%04lu.%1lu", label, sign,
        (unsigned long) (tenths / 10U),
        (unsigned long) (tenths % 10U));
}

static void oled_show_measurement(void)
{
    char line[22];

    OLED_Clear();
    OLED_DrawRectangle(0, 0, 127, 63, 1);
    (void) snprintf(line, sizeof(line), "MPU OK A:%02X W:%02X",
        MPU6050_SW_GetAddress(), MPU6050_SW_GetWhoAmI());
    OLED_ShowString(4, 3, line, 8, 1);

    format_signed_tenths(line, sizeof(line), "ANG", g_yawMdeg);
    OLED_ShowString(8, 15, line, 8, 1);
    format_signed_tenths(line, sizeof(line), "GZ", g_gyroZMdps);
    OLED_ShowString(8, 27, line, 8, 1);
    format_signed_tenths(line, sizeof(line), "PEAK", g_peakAbsYawMdeg);
    OLED_ShowString(8, 39, line, 8, 1);
    (void) snprintf(line, sizeof(line), "ERR:%03lu N:%05lu",
        (unsigned long) (g_commErrors % 1000U),
        (unsigned long) (g_validSamples % 100000U));
    OLED_ShowString(8, 51, line, 8, 1);
    OLED_Refresh();
}

static void calibrate_mpu(void)
{
    oled_show_calibration();
    g_mpuReady = MPU6050_SW_Init();
    if (g_mpuReady &&
        (MPU6050_SW_GetWhoAmI() == MPU6050_EXPECTED_WHO_AM_I)) {
        g_mpuReady = MPU6050_SW_CalibrateGyroZ();
    } else {
        g_mpuReady = false;
    }

    g_gyroZMdps = 0;
    g_commErrors = 0U;
    reset_angle();
    if (g_mpuReady) {
        oled_show_measurement();
    } else {
        oled_show_failure();
    }
}

static void sample_mpu(uint32_t elapsed_ms)
{
    int32_t gyro_mdps;
    int64_t numerator;
    int32_t abs_yaw;

    if (!MPU6050_SW_ReadGyroZDpsMilli(&gyro_mdps)) {
        g_commErrors++;
        return;
    }

    if (abs_i32(gyro_mdps) < GYRO_DEADBAND_MDPS) {
        gyro_mdps = 0;
    }
    g_gyroZMdps = gyro_mdps;
    numerator = ((int64_t) gyro_mdps * elapsed_ms) +
        g_integrationRemainder;
    g_yawMdeg += (int32_t) (numerator / 1000);
    g_integrationRemainder = numerator % 1000;
    g_validSamples++;

    abs_yaw = abs_i32(g_yawMdeg);
    if (abs_yaw > g_peakAbsYawMdeg) {
        g_peakAbsYawMdeg = abs_yaw;
    }
}

int main(void)
{
    uint32_t last_sample;
    uint32_t last_oled;
    uint32_t last_key_poll;

    SYSCFG_DL_init();
    (void) SysTick_Config(CPUCLK_FREQ / 1000U);
    OLED_Init();
    OLED_ColorTurn(0);
    OLED_DisplayTurn(0);
    keys_init();
    calibrate_mpu();

    last_sample = g_milliseconds;
    last_oled = g_milliseconds;
    last_key_poll = g_milliseconds;

    while (1) {
        uint32_t now = g_milliseconds;

        if ((now - last_key_poll) >= KEY_POLL_PERIOD_MS) {
            last_key_poll = now;
            if (key_update(&g_key1, key_pressed(KEYS_KEY1_PIN))) {
                reset_angle();
            }
            if (key_update(&g_key2, key_pressed(KEYS_KEY2_PIN))) {
                calibrate_mpu();
                now = g_milliseconds;
                last_sample = now;
                last_oled = now;
                last_key_poll = now;
            }
        }

        if (g_mpuReady &&
            ((now - last_sample) >= MPU_SAMPLE_PERIOD_MS)) {
            uint32_t elapsed_ms = now - last_sample;
            last_sample = now;
            sample_mpu(elapsed_ms);
        }

        if ((now - last_oled) >= OLED_UPDATE_PERIOD_MS) {
            last_oled = now;
            if (g_mpuReady) {
                oled_show_measurement();
            } else {
                oled_show_failure();
            }
        }
    }
}
