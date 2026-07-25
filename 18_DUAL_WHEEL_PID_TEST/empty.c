#include "ti_msp_dl_config.h"
#include "bsp_encoder.h"
#include "bsp_tb6612.h"
#include "oled.h"
#include "speed_pi.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define CONTROL_PERIOD_MS            (50U)
#define OLED_UPDATE_MS               (200U)
#define KEY_POLL_MS                  (10U)
#define KEY_DEBOUNCE_SAMPLES         (3U)
#define DEFAULT_TARGET_RPS_MILLI     (800)
#define TARGET_STEP_RPS_MILLI        (200)
#define MIN_TARGET_RPS_MILLI         (200)
#define MAX_TARGET_RPS_MILLI         (3000)
#define SETPOINT_RAMP_PPS            (50)
#define SPEED_FILTER_DIVISOR         (4)

#define LEFT_FORWARD_DIR             (0U)
#define LEFT_REVERSE_DIR             (1U)
#define RIGHT_FORWARD_DIR            (1U)
#define RIGHT_REVERSE_DIR            (0U)

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
static SpeedPI g_leftController;
static SpeedPI g_rightController;
static WheelSpeed g_leftSpeed;
static WheelSpeed g_rightSpeed;
static int32_t g_leftFilteredPps;
static int32_t g_rightFilteredPps;
static int32_t g_commandRpsMilli = DEFAULT_TARGET_RPS_MILLI;
static int32_t g_rampedTargetPps;
static bool g_running;
static bool g_forward = true;

void SysTick_Handler(void)
{
    g_milliseconds++;
}

static int32_t abs_i32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static int32_t clamp_i32(int32_t value, int32_t minimum, int32_t maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static uint8_t key_is_pressed(uint32_t pin)
{
    return (DL_GPIO_readPins(KEYS_PORT, pin) == 0U) ? 1U : 0U;
}

static void reset_controllers(void)
{
    SpeedPI_Reset(&g_leftController);
    SpeedPI_Reset(&g_rightController);
    g_rampedTargetPps = 0;
}

static void stop_motors(void)
{
    g_running = false;
    reset_controllers();
    TB6612_Motor_Stop();
}

static void handle_key(KeyId key)
{
    switch (key) {
        case KEY_1:
            reset_controllers();
            g_running = true;
            break;
        case KEY_2:
            g_commandRpsMilli = clamp_i32(
                g_commandRpsMilli + TARGET_STEP_RPS_MILLI,
                MIN_TARGET_RPS_MILLI, MAX_TARGET_RPS_MILLI);
            break;
        case KEY_3:
            g_commandRpsMilli = clamp_i32(
                g_commandRpsMilli - TARGET_STEP_RPS_MILLI,
                MIN_TARGET_RPS_MILLI, MAX_TARGET_RPS_MILLI);
            break;
        case KEY_4:
            stop_motors();
            break;
        case KEY_5:
            stop_motors();
            g_forward = !g_forward;
            break;
        case KEY_6:
            reset_controllers();
            break;
        default:
            break;
    }
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

static int32_t target_rps_to_pps(int32_t rps_milli)
{
    return (rps_milli * (int32_t) ENCODER_COUNTS_PER_REV) /
        SPEED_DECIMAL_SCALE;
}

static int32_t filter_speed(int32_t filtered, int32_t measured)
{
    return filtered + ((measured - filtered) / SPEED_FILTER_DIVISOR);
}

static void apply_motor_pwm(uint32_t left_pwm, uint32_t right_pwm)
{
    uint8_t left_dir = g_forward ? LEFT_FORWARD_DIR : LEFT_REVERSE_DIR;
    uint8_t right_dir = g_forward ? RIGHT_FORWARD_DIR : RIGHT_REVERSE_DIR;

    if (left_pwm == 0U) {
        AO_Stop();
    } else {
        AO_Control(left_dir, left_pwm);
    }
    if (right_pwm == 0U) {
        BO_Stop();
    } else {
        BO_Control(right_dir, right_pwm);
    }
}

static void control_update(uint32_t elapsed_ms)
{
    int32_t command_pps = target_rps_to_pps(g_commandRpsMilli);
    int32_t left_measured;
    int32_t right_measured;
    uint32_t left_pwm;
    uint32_t right_pwm;

    g_leftSpeed = Encoder_UpdateSpeed(ENCODER_LEFT, elapsed_ms);
    g_rightSpeed = Encoder_UpdateSpeed(ENCODER_RIGHT, elapsed_ms);
    left_measured = abs_i32(g_leftSpeed.pps);
    right_measured = abs_i32(g_rightSpeed.pps);
    g_leftFilteredPps = filter_speed(g_leftFilteredPps, left_measured);
    g_rightFilteredPps = filter_speed(g_rightFilteredPps, right_measured);

    if (!g_running) {
        apply_motor_pwm(0U, 0U);
        return;
    }

    if (g_rampedTargetPps < command_pps) {
        g_rampedTargetPps = clamp_i32(
            g_rampedTargetPps + SETPOINT_RAMP_PPS, 0, command_pps);
    } else if (g_rampedTargetPps > command_pps) {
        g_rampedTargetPps = clamp_i32(
            g_rampedTargetPps - SETPOINT_RAMP_PPS, command_pps,
            MAX_TARGET_RPS_MILLI * (int32_t) ENCODER_COUNTS_PER_REV /
                SPEED_DECIMAL_SCALE);
    }

    left_pwm = SpeedPI_Update(&g_leftController, g_rampedTargetPps,
        g_leftFilteredPps, elapsed_ms);
    right_pwm = SpeedPI_Update(&g_rightController, g_rampedTargetPps,
        g_rightFilteredPps, elapsed_ms);
    apply_motor_pwm(left_pwm, right_pwm);
}

static void format_rps(char *text, size_t size, const char *label,
    int32_t pps, uint32_t pwm)
{
    uint32_t rps_milli = ((uint32_t) abs_i32(pps) * SPEED_DECIMAL_SCALE) /
        ENCODER_COUNTS_PER_REV;

    (void) snprintf(text, size, "%s:%lu.%02lu P:%02lu", label,
        (unsigned long) (rps_milli / 1000U),
        (unsigned long) ((rps_milli % 1000U) / 10U),
        (unsigned long) pwm);
}

static void oled_show_status(void)
{
    char line[22];

    OLED_Clear();
    OLED_DrawRectangle(0, 0, 127, 63, 1);
    (void) snprintf(line, sizeof(line), "PI %s %c T:%ld.%01ld",
        g_running ? "RUN" : "STOP", g_forward ? 'F' : 'R',
        (long) (g_commandRpsMilli / 1000),
        (long) ((g_commandRpsMilli % 1000) / 100));
    OLED_ShowString(4, 3, line, 8, 1);

    format_rps(line, sizeof(line), "L", g_leftFilteredPps,
        g_leftController.output_percent);
    OLED_ShowString(8, 15, line, 8, 1);
    format_rps(line, sizeof(line), "R", g_rightFilteredPps,
        g_rightController.output_percent);
    OLED_ShowString(8, 27, line, 8, 1);

    (void) snprintf(line, sizeof(line), "E L:%+04ld R:%+04ld",
        (long) g_leftController.error, (long) g_rightController.error);
    OLED_ShowString(8, 39, line, 8, 1);
    (void) snprintf(line, sizeof(line), "CPR:%lu DT:%luMS",
        (unsigned long) ENCODER_COUNTS_PER_REV,
        (unsigned long) CONTROL_PERIOD_MS);
    OLED_ShowString(8, 51, line, 8, 1);
    OLED_Refresh();
}

int main(void)
{
    uint32_t last_control;
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

    last_control = g_milliseconds;
    last_oled = g_milliseconds;
    last_key_poll = g_milliseconds;

    while (1) {
        uint32_t now = g_milliseconds;

        if ((now - last_key_poll) >= KEY_POLL_MS) {
            last_key_poll = now;
            keys_poll();
        }
        if ((now - last_control) >= CONTROL_PERIOD_MS) {
            uint32_t elapsed_ms = now - last_control;
            last_control = now;
            control_update(elapsed_ms);
        }
        if ((now - last_oled) >= OLED_UPDATE_MS) {
            last_oled = now;
            oled_show_status();
        }
    }
}
