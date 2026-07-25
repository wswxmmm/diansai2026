#include "ti_msp_dl_config.h"
#include "bsp_encoder.h"
#include "bsp_tb6612.h"
#include "oled.h"
#include "speed_pid.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define CONTROL_PERIOD_MS             (50U)
#define OLED_UPDATE_MS                (200U)
#define KEY_POLL_MS                   (10U)
#define KEY_DEBOUNCE_SAMPLES          (3U)

#define DEFAULT_TARGET_SPEED          (40)
#define MIN_TARGET_SPEED              (0)
#define MAX_TARGET_SPEED              (100)
#define TARGET_SPEED_STEP             (5)
#define AUTO_LOW_SPEED                (30)
#define AUTO_HIGH_SPEED               (60)
#define AUTO_STEP_PERIOD_MS           (3000U)
#define TARGET_RAMP_PPS_PER_SECOND    (1000)
#define SPEED_FILTER_DIVISOR          (3)

#define KP_STEP_X1000                 (25)
#define KI_STEP_X1000                 (25)
#define KD_STEP_X1000                 (1)
#define KP_MAX_X1000                  (2000)
#define KI_MAX_X1000                  (2000)
#define KD_MAX_X1000                  (100)

#define LEFT_FORWARD_DIR              (0U)
#define RIGHT_FORWARD_DIR             (1U)

typedef enum {
    KEY_1 = 0,
    KEY_2,
    KEY_3,
    KEY_4,
    KEY_5,
    KEY_6,
    KEY_COUNT
} KeyId;

typedef enum {
    TUNE_TARGET = 0,
    TUNE_KP,
    TUNE_KI,
    TUNE_KD,
    TUNE_COUNT
} TuneItem;

typedef struct {
    uint8_t raw;
    uint8_t stable;
    uint8_t samples;
} KeyState;

volatile int32_t g_tuneTargetSpeed = DEFAULT_TARGET_SPEED;
volatile int32_t g_tuneKpX1000 = 200;
volatile int32_t g_tuneKiX1000 = 100;
volatile int32_t g_tuneKdX1000 = 0;

static volatile uint32_t g_milliseconds;
static KeyState g_keys[KEY_COUNT];
static const uint32_t g_keyPins[KEY_COUNT] = {
    KEYS_KEY1_PIN, KEYS_KEY2_PIN, KEYS_KEY3_PIN,
    KEYS_KEY4_PIN, KEYS_KEY5_PIN, KEYS_KEY6_PIN
};
static SpeedPID g_leftController;
static SpeedPID g_rightController;
static WheelSpeed g_leftSpeed;
static WheelSpeed g_rightSpeed;
static int32_t g_leftFilteredPps;
static int32_t g_rightFilteredPps;
static int32_t g_rampedTargetPps;
static TuneItem g_selected = TUNE_TARGET;
static bool g_running;
static bool g_autoStep;
static bool g_autoHigh;
static uint32_t g_lastAutoStep;

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

static void reset_control_state(void)
{
    SpeedPID_Reset(&g_leftController);
    SpeedPID_Reset(&g_rightController);
    Encoder_Reset();
    g_leftFilteredPps = 0;
    g_rightFilteredPps = 0;
    g_rampedTargetPps = 0;
}

static void start_control(void)
{
    reset_control_state();
    g_running = true;
}

static void stop_control(void)
{
    g_running = false;
    reset_control_state();
    TB6612_Motor_Stop();
}

static void adjust_selected(int32_t direction)
{
    switch (g_selected) {
        case TUNE_TARGET:
            g_tuneTargetSpeed = clamp_i32(
                g_tuneTargetSpeed + direction * TARGET_SPEED_STEP,
                MIN_TARGET_SPEED, MAX_TARGET_SPEED);
            g_autoStep = false;
            break;
        case TUNE_KP:
            g_tuneKpX1000 = clamp_i32(
                g_tuneKpX1000 + direction * KP_STEP_X1000,
                0, KP_MAX_X1000);
            reset_control_state();
            break;
        case TUNE_KI:
            g_tuneKiX1000 = clamp_i32(
                g_tuneKiX1000 + direction * KI_STEP_X1000,
                0, KI_MAX_X1000);
            reset_control_state();
            break;
        case TUNE_KD:
            g_tuneKdX1000 = clamp_i32(
                g_tuneKdX1000 + direction * KD_STEP_X1000,
                0, KD_MAX_X1000);
            reset_control_state();
            break;
        default:
            break;
    }
}

static void handle_key(KeyId key)
{
    switch (key) {
        case KEY_1:
            g_selected = (TuneItem) (((uint32_t) g_selected + 1U) %
                (uint32_t) TUNE_COUNT);
            break;
        case KEY_2:
            adjust_selected(1);
            break;
        case KEY_3:
            adjust_selected(-1);
            break;
        case KEY_4:
            if (g_running) {
                stop_control();
            } else {
                start_control();
            }
            break;
        case KEY_5:
            g_autoStep = !g_autoStep;
            if (g_autoStep) {
                g_autoHigh = false;
                g_tuneTargetSpeed = AUTO_LOW_SPEED;
                g_lastAutoStep = g_milliseconds;
                if (!g_running) {
                    start_control();
                }
            }
            break;
        case KEY_6:
            reset_control_state();
            g_lastAutoStep = g_milliseconds;
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

static int32_t filter_speed(int32_t filtered, int32_t measured)
{
    return filtered + ((measured - filtered) / SPEED_FILTER_DIVISOR);
}

static int32_t target_speed_to_pps(int32_t target_speed)
{
    return (target_speed * 1000) / (int32_t) CONTROL_PERIOD_MS;
}

static void update_ramped_target(int32_t command_pps, uint32_t elapsed_ms)
{
    int32_t step = (TARGET_RAMP_PPS_PER_SECOND * (int32_t) elapsed_ms) / 1000;

    if (step < 1) {
        step = 1;
    }
    if (g_rampedTargetPps < command_pps) {
        g_rampedTargetPps = clamp_i32(g_rampedTargetPps + step,
            0, command_pps);
    } else if (g_rampedTargetPps > command_pps) {
        g_rampedTargetPps = clamp_i32(g_rampedTargetPps - step,
            command_pps, MAX_TARGET_SPEED * 1000 /
                (int32_t) CONTROL_PERIOD_MS);
    }
}

static void apply_output(uint32_t left_permille, uint32_t right_permille)
{
    if (left_permille == 0U) {
        AO_Stop();
    } else {
        AO_ControlPermille(LEFT_FORWARD_DIR, left_permille);
    }
    if (right_permille == 0U) {
        BO_Stop();
    } else {
        BO_ControlPermille(RIGHT_FORWARD_DIR, right_permille);
    }
}

static void control_update(uint32_t elapsed_ms)
{
    SpeedPIDGains gains;
    int32_t command_pps;
    uint32_t left_output;
    uint32_t right_output;

    g_leftSpeed = Encoder_UpdateSpeed(ENCODER_LEFT, elapsed_ms);
    g_rightSpeed = Encoder_UpdateSpeed(ENCODER_RIGHT, elapsed_ms);
    g_leftFilteredPps = filter_speed(g_leftFilteredPps,
        g_leftSpeed.pps);
    g_rightFilteredPps = filter_speed(g_rightFilteredPps,
        g_rightSpeed.pps);

    if (!g_running) {
        apply_output(0U, 0U);
        return;
    }

    gains.kp_x1000 = clamp_i32(g_tuneKpX1000, 0, KP_MAX_X1000);
    gains.ki_x1000 = clamp_i32(g_tuneKiX1000, 0, KI_MAX_X1000);
    gains.kd_x1000 = clamp_i32(g_tuneKdX1000, 0, KD_MAX_X1000);
    command_pps = target_speed_to_pps(clamp_i32(g_tuneTargetSpeed,
        MIN_TARGET_SPEED, MAX_TARGET_SPEED));
    update_ramped_target(command_pps, elapsed_ms);

    left_output = SpeedPID_Update(&g_leftController, &gains,
        g_rampedTargetPps, g_leftFilteredPps, elapsed_ms);
    right_output = SpeedPID_Update(&g_rightController, &gains,
        g_rampedTargetPps, g_rightFilteredPps, elapsed_ms);
    apply_output(left_output, right_output);
}

static void format_wheel(char *text, size_t size, char wheel,
    int32_t target_speed, int32_t filtered_pps)
{
    char sign = (filtered_pps < 0) ? '-' : '+';
    uint32_t actual_speed = (((uint32_t) abs_i32(filtered_pps) *
        CONTROL_PERIOD_MS) + 500U) / 1000U;

    (void) snprintf(text, size, "%c T:%03ld A:%c%03lu", wheel,
        (long) target_speed, sign, (unsigned long) actual_speed);
}

static void format_selected(char *text, size_t size)
{
    switch (g_selected) {
        case TUNE_TARGET:
            (void) snprintf(text, size, ">TGT:%03ld CNT",
                (long) g_tuneTargetSpeed);
            break;
        case TUNE_KP:
            (void) snprintf(text, size, ">KP:%ld.%03ld",
                (long) (g_tuneKpX1000 / 1000),
                (long) (g_tuneKpX1000 % 1000));
            break;
        case TUNE_KI:
            (void) snprintf(text, size, ">KI:%ld.%03ld",
                (long) (g_tuneKiX1000 / 1000),
                (long) (g_tuneKiX1000 % 1000));
            break;
        case TUNE_KD:
            (void) snprintf(text, size, ">KD:%ld.%03ld",
                (long) (g_tuneKdX1000 / 1000),
                (long) (g_tuneKdX1000 % 1000));
            break;
        default:
            text[0] = '\0';
            break;
    }
}

static void oled_show_status(void)
{
    char line[22];

    OLED_Clear();
    OLED_DrawRectangle(0, 0, 127, 63, 1);
    (void) snprintf(line, sizeof(line), "PID %s %c %luMS",
        g_running ? "RUN" : "STOP", g_autoStep ? 'A' : 'M',
        (unsigned long) CONTROL_PERIOD_MS);
    OLED_ShowString(4, 2, line, 8, 1);

    format_wheel(line, sizeof(line), 'L', g_tuneTargetSpeed,
        g_leftFilteredPps);
    OLED_ShowString(6, 12, line, 8, 1);
    format_wheel(line, sizeof(line), 'R', g_tuneTargetSpeed,
        g_rightFilteredPps);
    OLED_ShowString(6, 22, line, 8, 1);

    (void) snprintf(line, sizeof(line), "O L:%lu.%lu R:%lu.%lu",
        (unsigned long) (g_leftController.output_permille / 10U),
        (unsigned long) (g_leftController.output_permille % 10U),
        (unsigned long) (g_rightController.output_permille / 10U),
        (unsigned long) (g_rightController.output_permille % 10U));
    OLED_ShowString(6, 32, line, 8, 1);

    format_selected(line, sizeof(line));
    OLED_ShowString(6, 42, line, 8, 1);
    (void) snprintf(line, sizeof(line), "P%ld I%ld D%ld",
        (long) g_tuneKpX1000, (long) g_tuneKiX1000,
        (long) g_tuneKdX1000);
    OLED_ShowString(6, 52, line, 8, 1);
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
    g_lastAutoStep = g_milliseconds;

    while (1) {
        uint32_t now = g_milliseconds;

        if ((now - last_key_poll) >= KEY_POLL_MS) {
            last_key_poll = now;
            keys_poll();
        }
        if (g_autoStep &&
            ((now - g_lastAutoStep) >= AUTO_STEP_PERIOD_MS)) {
            g_lastAutoStep = now;
            g_autoHigh = !g_autoHigh;
            g_tuneTargetSpeed = g_autoHigh ?
                AUTO_HIGH_SPEED : AUTO_LOW_SPEED;
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
