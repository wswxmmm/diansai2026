#include "ti_msp_dl_config.h"
#include "bsp_encoder.h"
#include "bsp_tb6612.h"
#include "oled.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define SPEED_SAMPLE_MS              (100U)
#define OLED_UPDATE_MS               (200U)
#define KEY_POLL_MS                  (10U)
#define KEY_DEBOUNCE_SAMPLES         (3U)
#define DEFAULT_LEFT_PWM             (12U)
#define DEFAULT_RIGHT_PWM            (12U)
#define PWM_STEP                     (1U)
#define PWM_TEST_MAX                 (60U)

#define LEFT_FORWARD_DIR             (0U)
#define RIGHT_FORWARD_DIR            (1U)

typedef enum {
    KEY_1 = 0,
    KEY_2,
    KEY_3,
    KEY_4,
    KEY_5,
    KEY_6,
    KEY_COUNT
} KeyId;

typedef struct {
    uint8_t raw;
    uint8_t stable;
    uint8_t samples;
} KeyState;

static volatile uint32_t g_milliseconds;
static KeyState g_keys[KEY_COUNT];
static const uint32_t g_keyPins[KEY_COUNT] = {
    KEYS_KEY1_PIN, KEYS_KEY2_PIN, KEYS_KEY3_PIN,
    KEYS_KEY4_PIN, KEYS_KEY5_PIN, KEYS_KEY6_PIN
};
static uint32_t g_leftPwm = DEFAULT_LEFT_PWM;
static uint32_t g_rightPwm = DEFAULT_RIGHT_PWM;
static bool g_running;
static WheelSpeed g_leftSpeed;
static WheelSpeed g_rightSpeed;

void SysTick_Handler(void)
{
    g_milliseconds++;
}

static uint8_t key_is_pressed(uint32_t pin)
{
    return (DL_GPIO_readPins(KEYS_PORT, pin) == 0U) ? 1U : 0U;
}

static uint32_t pwm_increase(uint32_t pwm)
{
    if (pwm >= PWM_TEST_MAX) {
        return PWM_TEST_MAX;
    }
    return pwm + PWM_STEP;
}

static uint32_t pwm_decrease(uint32_t pwm)
{
    if (pwm <= PWM_STEP) {
        return 0U;
    }
    return pwm - PWM_STEP;
}

static void apply_pwm(void)
{
    if (!g_running) {
        TB6612_Motor_Stop();
        return;
    }

    if (g_leftPwm == 0U) {
        AO_Stop();
    } else {
        AO_Control(LEFT_FORWARD_DIR, g_leftPwm);
    }
    if (g_rightPwm == 0U) {
        BO_Stop();
    } else {
        BO_Control(RIGHT_FORWARD_DIR, g_rightPwm);
    }
}

static void handle_key(KeyId key)
{
    switch (key) {
        case KEY_1:
            g_leftPwm = pwm_increase(g_leftPwm);
            break;
        case KEY_2:
            g_leftPwm = pwm_decrease(g_leftPwm);
            break;
        case KEY_3:
            g_rightPwm = pwm_increase(g_rightPwm);
            break;
        case KEY_4:
            g_rightPwm = pwm_decrease(g_rightPwm);
            break;
        case KEY_5:
            Encoder_Reset();
            g_running = true;
            break;
        case KEY_6:
            g_running = false;
            break;
        default:
            break;
    }
    apply_pwm();
}

static void keys_init(void)
{
    uint32_t i;

    for (i = 0U; i < KEY_COUNT; i++) {
        uint8_t pressed = key_is_pressed(g_keyPins[i]);
        g_keys[i].raw = pressed;
        g_keys[i].stable = pressed;
        g_keys[i].samples = KEY_DEBOUNCE_SAMPLES;
    }
}

static void keys_poll(void)
{
    uint32_t i;

    for (i = 0U; i < KEY_COUNT; i++) {
        uint8_t pressed = key_is_pressed(g_keyPins[i]);
        KeyState *state = &g_keys[i];

        if (pressed != state->raw) {
            state->raw = pressed;
            state->samples = 1U;
            continue;
        }
        if (state->samples < KEY_DEBOUNCE_SAMPLES) {
            state->samples++;
        }
        if ((state->samples == KEY_DEBOUNCE_SAMPLES) &&
            (state->stable != pressed)) {
            state->stable = pressed;
            if (pressed != 0U) {
                handle_key((KeyId) i);
            }
        }
    }
}

static void format_speed_line(char *text, size_t size, char wheel,
    uint32_t pwm, int32_t rps_milli)
{
    char sign = '+';
    uint32_t speed;

    if (rps_milli < 0) {
        sign = '-';
        rps_milli = -rps_milli;
    }
    speed = (uint32_t) rps_milli;
    (void) snprintf(text, size, "%c P:%02lu S:%c%lu.%02lu", wheel,
        (unsigned long) pwm, sign,
        (unsigned long) (speed / 1000U),
        (unsigned long) ((speed % 1000U) / 10U));
}

static void oled_show_status(void)
{
    char line[22];

    OLED_Clear();
    OLED_DrawRectangle(0, 0, 127, 63, 1);
    (void) snprintf(line, sizeof(line), "OPEN PWM %s",
        g_running ? "RUN" : "STOP");
    OLED_ShowString(8, 3, line, 8, 1);

    format_speed_line(line, sizeof(line), 'L', g_leftPwm,
        g_leftSpeed.rps_milli);
    OLED_ShowString(8, 15, line, 8, 1);
    format_speed_line(line, sizeof(line), 'R', g_rightPwm,
        g_rightSpeed.rps_milli);
    OLED_ShowString(8, 27, line, 8, 1);

    (void) snprintf(line, sizeof(line), "D:%+04ld/%+04ld",
        (long) g_leftSpeed.delta, (long) g_rightSpeed.delta);
    OLED_ShowString(8, 39, line, 8, 1);
    (void) snprintf(line, sizeof(line), "C:%lu T:%luMS",
        (unsigned long) ENCODER_COUNTS_PER_REV,
        (unsigned long) SPEED_SAMPLE_MS);
    OLED_ShowString(8, 51, line, 8, 1);
    OLED_Refresh();
}

int main(void)
{
    uint32_t last_speed_sample;
    uint32_t last_oled;
    uint32_t last_key_poll;

    SYSCFG_DL_init();
    (void) SysTick_Config(CPUCLK_FREQ / 1000U);
    TB6612_Motor_Stop();
    DL_TimerA_startCounter(PWM_0_INST);
    Encoder_Init();
    keys_init();

    OLED_Init();
    OLED_ColorTurn(0);
    OLED_DisplayTurn(0);
    oled_show_status();

    last_speed_sample = g_milliseconds;
    last_oled = g_milliseconds;
    last_key_poll = g_milliseconds;

    while (1) {
        uint32_t now = g_milliseconds;

        if ((now - last_key_poll) >= KEY_POLL_MS) {
            last_key_poll = now;
            keys_poll();
        }
        if ((now - last_speed_sample) >= SPEED_SAMPLE_MS) {
            uint32_t elapsed_ms = now - last_speed_sample;
            last_speed_sample = now;
            g_leftSpeed = Encoder_UpdateSpeed(ENCODER_LEFT, elapsed_ms);
            g_rightSpeed = Encoder_UpdateSpeed(ENCODER_RIGHT, elapsed_ms);
        }
        if ((now - last_oled) >= OLED_UPDATE_MS) {
            last_oled = now;
            oled_show_status();
        }
    }
}
