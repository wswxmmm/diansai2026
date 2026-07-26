#include "ti_msp_dl_config.h"
#include "bsp_encoder.h"
#include "bsp_tb6612.h"
#include "oled.h"
#include "pid_uart.h"
#include "speed_pid.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define CONTROL_PERIOD_MS             (10U)
#define DRIVE_UPDATE_MS               (1U)
#define TELEMETRY_PERIOD_MS           (50U)
#define OLED_UPDATE_MS                (200U)
#define KEY_POLL_MS                   (10U)
#define KEY_DEBOUNCE_SAMPLES          (3U)
#define REMOTE_WATCHDOG_MS            (2000U)
#define SERIAL_COMMAND_SIZE           (64U)
#define SERIAL_MESSAGE_SIZE           (192U)

#define DEFAULT_TARGET_SPEED          (40)
#define MIN_TARGET_SPEED              (0)
#define MAX_TARGET_SPEED              (150)
#define TARGET_SPEED_STEP             (5)
#define AUTO_LOW_SPEED                (30)
#define AUTO_HIGH_SPEED               (60)
#define AUTO_STEP_PERIOD_MS           (3000U)
#define STRAIGHT_TEST_TARGET_SPEED     (135)
#define STRAIGHT_TEST_DURATION_MS      (10000U)
#define TARGET_RAMP_PPS_PER_SECOND    (10000)
#define TARGET_PPS_PER_UNIT           (200)
#define SPEED_FILTER_DIVISOR          (5)
#define DRIVE_DITHER_MIN_PERMILLE     (50U)

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
volatile int32_t g_tuneKpX1000 = 20;
volatile int32_t g_tuneKiX1000 = 30;
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
static uint32_t g_leftDriveAccumulator;
static uint32_t g_rightDriveAccumulator;
static uint32_t g_leftRequestedOutput;
static uint32_t g_rightRequestedOutput;
static TuneItem g_selected = TUNE_TARGET;
static bool g_running;
static bool g_autoStep;
static bool g_autoHigh;
static uint32_t g_lastAutoStep;
static bool g_remoteWatchdog;
static uint32_t g_lastRemoteCommandMs;
static bool g_straightTest;
static bool g_straightTestDone;
static uint32_t g_straightTestStartMs;
static int32_t g_straightResultLeftPps;
static int32_t g_straightResultRightPps;

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

static bool parse_i32(const char *text, int32_t *value)
{
    int32_t result = 0;
    int32_t sign   = 1;
    bool hasDigit  = false;

    if ((text == 0) || (value == 0)) {
        return false;
    }
    if (*text == '-') {
        sign = -1;
        text++;
    }
    while ((*text >= '0') && (*text <= '9')) {
        hasDigit = true;
        result   = (result * 10) + (int32_t) (*text - '0');
        text++;
    }
    if (!hasDigit || (*text != '\0')) {
        return false;
    }

    *value = result * sign;
    return true;
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
    g_leftDriveAccumulator = 0U;
    g_rightDriveAccumulator = 0U;
    g_leftRequestedOutput = 0U;
    g_rightRequestedOutput = 0U;
}

static void start_control(void)
{
    reset_control_state();
    g_straightTest = false;
    g_straightTestDone = false;
    g_running = true;
}

static void stop_control(void)
{
    g_running = false;
    g_straightTest = false;
    g_straightTestDone = false;
    reset_control_state();
    TB6612_Motor_Stop();
}

static void start_straight_test(void)
{
    g_autoStep = false;
    g_tuneTargetSpeed = STRAIGHT_TEST_TARGET_SPEED;
    start_control();
    g_straightTest = true;
    g_straightTestStartMs = g_milliseconds;
}

static void complete_straight_test(void)
{
    g_straightResultLeftPps = g_leftFilteredPps;
    g_straightResultRightPps = g_rightFilteredPps;
    stop_control();
    g_straightTestDone = true;
}

static void serial_send_status(void)
{
    char message[SERIAL_MESSAGE_SIZE];

    (void) snprintf(message, sizeof(message),
        "STATUS,%lu,%u,%u,%ld,%ld,%ld,%ld\r\n",
        (unsigned long) g_milliseconds,
        g_running ? 1U : 0U,
        g_autoStep ? 1U : 0U,
        (long) g_tuneTargetSpeed,
        (long) g_tuneKpX1000,
        (long) g_tuneKiX1000,
        (long) g_tuneKdX1000);
    PID_UART_SendString(message);
}

static void serial_handle_command(const char *command)
{
    int32_t value;

    g_lastRemoteCommandMs = g_milliseconds;

    if (strcmp(command, "PING") == 0) {
        PID_UART_SendString("PONG\r\n");
        return;
    }
    if (strcmp(command, "GET") == 0) {
        serial_send_status();
        return;
    }
    if (strcmp(command, "RUN") == 0) {
        g_remoteWatchdog = true;
        start_control();
        PID_UART_SendString("OK,RUN\r\n");
        return;
    }
    if (strcmp(command, "STOP") == 0) {
        g_remoteWatchdog = false;
        stop_control();
        PID_UART_SendString("OK,STOP\r\n");
        return;
    }
    if (strcmp(command, "RESET") == 0) {
        reset_control_state();
        g_lastAutoStep = g_milliseconds;
        PID_UART_SendString("OK,RESET\r\n");
        return;
    }
    if (strcmp(command, "AUTO,1") == 0) {
        g_remoteWatchdog    = true;
        g_autoStep          = true;
        g_autoHigh          = false;
        g_tuneTargetSpeed   = AUTO_LOW_SPEED;
        g_lastAutoStep      = g_milliseconds;
        start_control();
        PID_UART_SendString("OK,AUTO,1\r\n");
        return;
    }
    if (strcmp(command, "AUTO,0") == 0) {
        g_autoStep = false;
        PID_UART_SendString("OK,AUTO,0\r\n");
        return;
    }
    if ((strncmp(command, "SET,TARGET,", 11U) == 0) &&
        parse_i32(&command[11], &value)) {
        g_tuneTargetSpeed = clamp_i32(
            value, MIN_TARGET_SPEED, MAX_TARGET_SPEED);
        g_autoStep = false;
        PID_UART_SendString("OK,SET,TARGET\r\n");
        return;
    }
    if ((strncmp(command, "SET,KP,", 7U) == 0) &&
        parse_i32(&command[7], &value)) {
        g_tuneKpX1000 = clamp_i32(value, 0, KP_MAX_X1000);
        reset_control_state();
        PID_UART_SendString("OK,SET,KP\r\n");
        return;
    }
    if ((strncmp(command, "SET,KI,", 7U) == 0) &&
        parse_i32(&command[7], &value)) {
        g_tuneKiX1000 = clamp_i32(value, 0, KI_MAX_X1000);
        reset_control_state();
        PID_UART_SendString("OK,SET,KI\r\n");
        return;
    }
    if ((strncmp(command, "SET,KD,", 7U) == 0) &&
        parse_i32(&command[7], &value)) {
        g_tuneKdX1000 = clamp_i32(value, 0, KD_MAX_X1000);
        reset_control_state();
        PID_UART_SendString("OK,SET,KD\r\n");
        return;
    }

    PID_UART_SendString("ERR,COMMAND\r\n");
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
            g_remoteWatchdog = false;
            if (g_running) {
                stop_control();
            } else {
                start_straight_test();
            }
            break;
        case KEY_5:
            g_remoteWatchdog = false;
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
            g_straightTestDone = false;
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
    return target_speed * TARGET_PPS_PER_UNIT;
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
            command_pps, MAX_TARGET_SPEED * TARGET_PPS_PER_UNIT);
    }
}

static uint32_t dither_drive_output(uint32_t requested,
    uint32_t *accumulator)
{
    if (requested == 0U) {
        *accumulator = 0U;
        return 0U;
    }
    if (requested >= DRIVE_DITHER_MIN_PERMILLE) {
        *accumulator = 0U;
        return requested;
    }

    *accumulator += requested;
    if (*accumulator >= DRIVE_DITHER_MIN_PERMILLE) {
        *accumulator -= DRIVE_DITHER_MIN_PERMILLE;
        return DRIVE_DITHER_MIN_PERMILLE;
    }
    return 0U;
}

static void apply_output(uint32_t left_permille, uint32_t right_permille)
{
    if (!g_running) {
        TB6612_Motor_Stop();
        return;
    }

    left_permille = dither_drive_output(left_permille,
        &g_leftDriveAccumulator);
    right_permille = dither_drive_output(right_permille,
        &g_rightDriveAccumulator);

    if (left_permille == 0U) {
        AO_Coast();
    } else {
        AO_ControlPermille(LEFT_FORWARD_DIR, left_permille);
    }
    if (right_permille == 0U) {
        BO_Coast();
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
        abs_i32(g_leftSpeed.pps));
    g_rightFilteredPps = filter_speed(g_rightFilteredPps,
        abs_i32(g_rightSpeed.pps));

    if (!g_running) {
        g_leftRequestedOutput = 0U;
        g_rightRequestedOutput = 0U;
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
    g_leftRequestedOutput = left_output;
    g_rightRequestedOutput = right_output;
}

static void serial_send_telemetry(void)
{
    char message[SERIAL_MESSAGE_SIZE];
    int32_t leftError  = g_rampedTargetPps - g_leftFilteredPps;
    int32_t rightError = g_rampedTargetPps - g_rightFilteredPps;

    (void) snprintf(message, sizeof(message),
        "DATA,%lu,%u,%u,%ld,%ld,%ld,%ld,%ld,%ld,%lu,%lu,%ld,%ld,%ld\r\n",
        (unsigned long) g_milliseconds,
        g_running ? 1U : 0U,
        g_autoStep ? 1U : 0U,
        (long) g_tuneTargetSpeed,
        (long) g_rampedTargetPps,
        (long) g_leftFilteredPps,
        (long) g_rightFilteredPps,
        (long) leftError,
        (long) rightError,
        (unsigned long) g_leftController.output_permille,
        (unsigned long) g_rightController.output_permille,
        (long) g_tuneKpX1000,
        (long) g_tuneKiX1000,
        (long) g_tuneKdX1000);
    PID_UART_SendString(message);
}

static void format_wheel(char *text, size_t size, char wheel,
    int32_t target_speed, int32_t filtered_pps)
{
    char sign = (filtered_pps < 0) ? '-' : '+';
    uint32_t actual_speed = ((uint32_t) abs_i32(filtered_pps) +
        (TARGET_PPS_PER_UNIT / 2U)) / TARGET_PPS_PER_UNIT;

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
    int32_t displayLeftPps = g_leftFilteredPps;
    int32_t displayRightPps = g_rightFilteredPps;

    OLED_Clear();
    OLED_DrawRectangle(0, 0, 127, 63, 1);
    if (g_straightTest) {
        uint32_t elapsed = g_milliseconds - g_straightTestStartMs;
        uint32_t remaining = (elapsed < STRAIGHT_TEST_DURATION_MS) ?
            STRAIGHT_TEST_DURATION_MS - elapsed : 0U;
        (void) snprintf(line, sizeof(line), "LINE RUN %lu.%luS",
            (unsigned long) (remaining / 1000U),
            (unsigned long) ((remaining % 1000U) / 100U));
    } else if (g_straightTestDone) {
        (void) snprintf(line, sizeof(line), "LINE TEST DONE");
        displayLeftPps = g_straightResultLeftPps;
        displayRightPps = g_straightResultRightPps;
    } else {
        (void) snprintf(line, sizeof(line), "PID %s %c %luMS",
            g_running ? "RUN" : "STOP", g_autoStep ? 'A' : 'M',
            (unsigned long) CONTROL_PERIOD_MS);
    }
    OLED_ShowString(4, 2, line, 8, 1);

    format_wheel(line, sizeof(line), 'L', g_tuneTargetSpeed,
        displayLeftPps);
    OLED_ShowString(6, 12, line, 8, 1);
    format_wheel(line, sizeof(line), 'R', g_tuneTargetSpeed,
        displayRightPps);
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
    uint32_t last_drive;
    uint32_t last_telemetry;
    uint32_t last_oled;
    uint32_t last_key_poll;
    char serialCommand[SERIAL_COMMAND_SIZE];

    SYSCFG_DL_init();
    (void) SysTick_Config(CPUCLK_FREQ / 1000U);
    PID_UART_Init();
    PID_UART_SendString("READY,PID_TUNER,RTT_UART\r\n");
    TB6612_Motor_Stop();
    DL_TimerA_startCounter(PWM_0_INST);
    Encoder_Init();
    keys_init();

    OLED_Init();
    OLED_ColorTurn(0);
    OLED_DisplayTurn(0);
    oled_show_status();

    last_control = g_milliseconds;
    last_drive = g_milliseconds;
    last_telemetry = g_milliseconds;
    last_oled = g_milliseconds;
    last_key_poll = g_milliseconds;
    g_lastAutoStep = g_milliseconds;
    g_remoteWatchdog = false;
    g_lastRemoteCommandMs = g_milliseconds;
    serial_send_status();

    while (1) {
        uint32_t now = g_milliseconds;

        while (PID_UART_ReadLine(serialCommand, sizeof(serialCommand))) {
            serial_handle_command(serialCommand);
        }

        /* Command handling may cross a SysTick boundary. Refresh now before
         * subtracting the command timestamp to avoid unsigned underflow. */
        now = g_milliseconds;

        if (g_remoteWatchdog && g_running &&
            ((now - g_lastRemoteCommandMs) >= REMOTE_WATCHDOG_MS)) {
            g_remoteWatchdog = false;
            stop_control();
            PID_UART_SendString("FAULT,REMOTE_TIMEOUT\r\n");
        }

        if (g_straightTest &&
            ((now - g_straightTestStartMs) >= STRAIGHT_TEST_DURATION_MS)) {
            complete_straight_test();
        }

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
        if ((now - last_drive) >= DRIVE_UPDATE_MS) {
            last_drive = now;
            apply_output(g_leftRequestedOutput, g_rightRequestedOutput);
        }
        if ((now - last_telemetry) >= TELEMETRY_PERIOD_MS) {
            last_telemetry = now;
            serial_send_telemetry();
        }
        if ((now - last_oled) >= OLED_UPDATE_MS) {
            last_oled = now;
            oled_show_status();
        }
    }
}
