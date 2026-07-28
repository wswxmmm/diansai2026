#include "ti_msp_dl_config.h"
#include "bsp_encoder.h"
#include "bsp_gray.h"
#include "bsp_mpu6050_sw.h"
#include "bsp_tb6612.h"
#include "closed_loop_drive.h"
#include "hc04.h"
#include "oled.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define KEY_POLL_MS                  (10U)
#define KEY_DEBOUNCE_SAMPLES         (3U)
#define CONTROL_PERIOD_MS            (10U)
#define OLED_UPDATE_MS               (200U)
#define REMOTE_LINE_BUFFER_SIZE      (32U)
#define REMOTE_DUPLICATE_WINDOW_MS   (1000U)
#define REMOTE_LINK_TIMEOUT_MS       (2500U)

#define GRAY_LINE_BLACK_THRESHOLD    (800U)
#define GRAY_CORNER_BLACK_THRESHOLD  (600U)
#define CORNER_BLACK_COUNT           (5U)
#define CORNER_CONFIRM_SAMPLES       (5U)
#define CORNER_FALLBACK_SAMPLES      (2U)
#define CORNER_CANDIDATE_HOLD_MS     (200U)
#define CORNER_LOCKOUT_MS            (600U)
#define CORNER_RECOVERY_MS           (300U)
#define LINE_LOST_GRACE_MS           (180U)
#define SEARCH_LINE_SAMPLES          (3U)

#define FOLLOW_BASE_SPEED            (17)
#define FOLLOW_MAX_SPEED             (25)
#define TURN_OUTER_SPEED             (20)
#define TURN_INNER_SPEED             (5)
#define TURN_SEARCH_OUTER_SPEED      (16)
#define TURN_SEARCH_INNER_SPEED      (4)
#define RECOVERY_BASE_SPEED          (12)
#define RECOVERY_MAX_SPEED           (17)
#define RECOVERY_MAX_CORRECTION      (4)
#define LINE_LOST_SPEED              (12)
#define LINE_ERROR_DIVISOR           (40)
#define MAX_STEERING_CORRECTION      (8)

#define TURN_REACQUIRE_MIN_MDEG      (55000U)
#define TURN_REACQUIRE_SAMPLES       (3U)
#define TURN_SLOWDOWN_MDEG           (70000U)
#define TURN_SEARCH_LIMIT_MDEG       (130000U)
#define TURN_LINE_LEAVE_MDEG         (15000U)
#define TURN_LINE_LEAVE_FALLBACK_MS  (150U)
#define TURN_TIMEOUT_MS              (6000U)
#define TURN_REACQUIRE_FALLBACK_MS   (500U)
#define GYRO_DEADBAND_MDPS           (500)
#define DEFAULT_TURN_RIGHT           (1)

#define STRAIGHT_TEST_DURATION_MS     (10000U)
#define STRAIGHT_TEST_BASE_SPEED      (12)
#define STRAIGHT_HEADING_DIVISOR      (3000)
#define STRAIGHT_MAX_CORRECTION       (4)
#define STRAIGHT_HEADING_LIMIT_MDEG   (30000)
#define STRAIGHT_MPU_FAILURE_LIMIT    (20U)
#define STRAIGHT_GYRO_CORRECTION_SIGN (1)

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

typedef enum {
    CAR_IDLE = 0,
    CAR_FOLLOW,
    CAR_TURN,
    CAR_RECOVER,
    CAR_SEARCH,
    CAR_STRAIGHT_TEST,
    CAR_DONE,
    CAR_ERROR
} CarState;

typedef struct {
    uint8_t raw;
    uint8_t stable;
    uint8_t samples;
} KeyState;

typedef struct {
    char data[REMOTE_LINE_BUFFER_SIZE];
    uint8_t length;
} RemoteLineBuffer;

static volatile uint32_t g_milliseconds;
static KeyState g_keyState[KEY_COUNT];
static uint16_t g_grayAdc[GRAY_SENSOR_CHANNELS];
static WheelSpeed g_leftSpeed;
static WheelSpeed g_rightSpeed;
static CarState g_carState = CAR_IDLE;
static bool g_mpuReady;
static uint8_t g_targetLaps;
static bool g_runContinuous;
static uint32_t g_cornerCount;
static uint8_t g_cornerConfirm;
static uint32_t g_cornerCandidateUntil;
static int32_t g_lastLineError;
static int8_t g_turnDirection = DEFAULT_TURN_RIGHT;
static uint32_t g_stateStartedMs;
static uint32_t g_cornerLockoutUntil;
static uint32_t g_turnLastMs;
static uint32_t g_turnAngleMdeg;
static bool g_turnLeftLine;
static uint8_t g_turnReacquireConfirm;
static uint8_t g_mpuReadFailures;
static bool g_lineLostActive;
static uint32_t g_lineLostStartedMs;
static uint8_t g_searchLineConfirm;
static bool g_searchCompletesCorner;
static RemoteLineBuffer g_remoteLine;
static uint32_t g_lastRemoteSequence;
static uint32_t g_lastRemoteStartMs;
static bool g_remoteSequenceValid;
static uint32_t g_remoteReceiveCount;
static uint32_t g_lastRemoteReceiveMs;
static uint32_t g_straightLastMs;
static int32_t g_straightHeadingMdeg;
static uint8_t g_straightMpuReadFailures;

static const uint32_t g_keyPins[KEY_COUNT] = {
    KEYS_KEY1_PIN,
    KEYS_KEY2_PIN,
    KEYS_KEY3_PIN,
    KEYS_KEY4_PIN,
    KEYS_KEY5_PIN,
    KEYS_KEY6_PIN
};

static const int16_t g_lineWeights[GRAY_SENSOR_CHANNELS] = {
    -350, -250, -150, -50, 50, 150, 250, 350
};

void SysTick_Handler(void)
{
    g_milliseconds++;
}

static uint8_t key_is_pressed(uint32_t pin)
{
    return (DL_GPIO_readPins(KEYS_PORT, pin) == 0U) ? 1U : 0U;
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

static void apply_wheel_speeds(int32_t left, int32_t right)
{
    ClosedLoopDrive_SetTargets(left, right);
}

static void apply_turn_arc(bool search_speed)
{
    int32_t outer = search_speed ? TURN_SEARCH_OUTER_SPEED : TURN_OUTER_SPEED;
    int32_t inner = search_speed ? TURN_SEARCH_INNER_SPEED : TURN_INNER_SPEED;

    if (g_turnDirection > 0) {
        apply_wheel_speeds(outer, inner);
    } else {
        apply_wheel_speeds(inner, outer);
    }
}

static bool time_reached(uint32_t now, uint32_t deadline)
{
    return ((int32_t) (now - deadline) >= 0);
}

static uint8_t analyze_line(int32_t *line_error)
{
    int32_t weighted_sum = 0;
    uint8_t black_count = 0U;
    uint8_t i;

    for (i = 0U; i < GRAY_SENSOR_CHANNELS; i++) {
        if (g_grayAdc[i] <= GRAY_LINE_BLACK_THRESHOLD) {
            weighted_sum += g_lineWeights[i];
            black_count++;
        }
    }

    if ((line_error != 0) && (black_count != 0U)) {
        *line_error = weighted_sum / (int32_t) black_count;
    }
    return black_count;
}

static bool center_line_is_black(void)
{
    return (g_grayAdc[3] <= GRAY_LINE_BLACK_THRESHOLD) ||
        (g_grayAdc[4] <= GRAY_LINE_BLACK_THRESHOLD);
}

static uint8_t count_corner_black(void)
{
    uint8_t black_count = 0U;
    uint8_t i;

    for (i = 0U; i < GRAY_SENSOR_CHANNELS; i++) {
        if (g_grayAdc[i] <= GRAY_CORNER_BLACK_THRESHOLD) {
            black_count++;
        }
    }
    return black_count;
}

static void stop_run(CarState state, uint32_t now)
{
    apply_wheel_speeds(0, 0);
    g_carState = state;
    g_stateStartedMs = now;
    g_cornerConfirm = 0U;
    g_cornerCandidateUntil = 0U;
    g_lineLostActive = false;
}

static void start_run(uint8_t laps, bool continuous, uint32_t now)
{
    if (!g_mpuReady) {
        stop_run(CAR_ERROR, now);
        return;
    }

    g_targetLaps = laps;
    g_runContinuous = continuous;
    g_cornerCount = 0U;
    g_cornerConfirm = 0U;
    g_cornerCandidateUntil = 0U;
    g_lastLineError = 0;
    g_turnAngleMdeg = 0U;
    g_cornerLockoutUntil = now + CORNER_LOCKOUT_MS;
    g_lineLostActive = false;
    g_carState = CAR_FOLLOW;
    g_stateStartedMs = now;
}

static void start_straight_test(uint32_t now)
{
    if (!g_mpuReady) {
        stop_run(CAR_ERROR, now);
        return;
    }

    apply_wheel_speeds(0, 0);
    g_targetLaps = 0U;
    g_runContinuous = false;
    g_cornerCount = 0U;
    g_straightHeadingMdeg = 0;
    g_straightMpuReadFailures = 0U;
    g_straightLastMs = now;
    g_carState = CAR_STRAIGHT_TEST;
    g_stateStartedMs = now;
    apply_wheel_speeds(STRAIGHT_TEST_BASE_SPEED,
        STRAIGHT_TEST_BASE_SPEED);
}

static bool remote_line_starts_with(const char *text, const char *prefix)
{
    while (*prefix != '\0') {
        if (*text++ != *prefix++) {
            return false;
        }
    }
    return true;
}

static bool remote_parse_unsigned(const char *text, uint32_t *value)
{
    uint32_t result = 0U;
    bool has_digit = false;

    while ((*text >= '0') && (*text <= '9')) {
        has_digit = true;
        result = (result * 10U) + (uint32_t) (*text - '0');
        text++;
    }
    if (!has_digit) {
        return false;
    }
    *value = result;
    return true;
}

static bool remote_line_push(uint8_t data)
{
    if (data == (uint8_t) '\r') {
        return false;
    }
    if (data == (uint8_t) '\n') {
        g_remoteLine.data[g_remoteLine.length] = '\0';
        return true;
    }
    if (g_remoteLine.length < (REMOTE_LINE_BUFFER_SIZE - 1U)) {
        g_remoteLine.data[g_remoteLine.length++] = (char) data;
    } else {
        g_remoteLine.length = 0U;
    }
    return false;
}

static void remote_send_unsigned(uint32_t value)
{
    uint8_t digits[10];
    uint8_t count = 0U;

    do {
        digits[count++] = (uint8_t) ('0' + (value % 10U));
        value /= 10U;
    } while (value > 0U);
    while (count > 0U) {
        HC04_SendByte(digits[--count]);
    }
}

static void remote_send_ack(uint32_t sequence)
{
    HC04_SendString("ACK START 2 ");
    remote_send_unsigned(sequence);
    HC04_SendString("\r\n");
}

static void remote_send_link_ack(uint32_t sequence)
{
    HC04_SendString("LINK ACK ");
    remote_send_unsigned(sequence);
    HC04_SendString("\r\n");
}

static void remote_mark_link_active(uint32_t now)
{
    g_remoteReceiveCount++;
    g_lastRemoteReceiveMs = now;
}

static void remote_process_byte(uint8_t data, uint32_t now)
{
    static const char start_prefix[] = "START 2 ";
    static const char link_prefix[] = "LINK PING ";
    uint32_t sequence;
    bool duplicate;

    if (!remote_line_push(data)) {
        return;
    }
    if (remote_line_starts_with(g_remoteLine.data, start_prefix) &&
        remote_parse_unsigned(
            &g_remoteLine.data[sizeof(start_prefix) - 1U],
            &sequence)) {
        remote_mark_link_active(now);
        duplicate = g_remoteSequenceValid &&
            (sequence == g_lastRemoteSequence) &&
            ((now - g_lastRemoteStartMs) < REMOTE_DUPLICATE_WINDOW_MS);
        if (!duplicate) {
            g_lastRemoteSequence = sequence;
            g_lastRemoteStartMs = now;
            g_remoteSequenceValid = true;
            start_run(2U, false, now);
        }
        remote_send_ack(sequence);
    } else if (remote_line_starts_with(g_remoteLine.data, link_prefix) &&
        remote_parse_unsigned(
            &g_remoteLine.data[sizeof(link_prefix) - 1U],
            &sequence)) {
        remote_mark_link_active(now);
        remote_send_link_ack(sequence);
    }
    g_remoteLine.length = 0U;
}

static void remote_poll(uint32_t now)
{
    uint8_t data;

    while (HC04_ReadByte(&data)) {
        remote_process_byte(data, now);
    }
}

static void handle_key_press(KeyId key, uint32_t now)
{
    switch (key) {
        case KEY_1:
            start_run(1U, false, now);
            break;
        case KEY_2:
            start_run(2U, false, now);
            break;
        case KEY_3:
            start_run(0U, true, now);
            break;
        case KEY_4:
            g_targetLaps = 0U;
            g_runContinuous = false;
            stop_run(CAR_IDLE, now);
            break;
        case KEY_6:
            start_straight_test(now);
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
        g_keyState[i].raw = pressed;
        g_keyState[i].stable = pressed;
        g_keyState[i].samples = KEY_DEBOUNCE_SAMPLES;
    }
}

static void keys_poll(uint32_t now)
{
    uint32_t i;

    for (i = 0U; i < KEY_COUNT; i++) {
        uint8_t pressed = key_is_pressed(g_keyPins[i]);
        KeyState *state = &g_keyState[i];

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
                handle_key_press((KeyId) i, now);
            }
        }
    }
}

static void begin_corner(int32_t line_error, uint32_t now)
{
    if (line_error < -60) {
        g_turnDirection = -1;
    } else if (line_error > 60) {
        g_turnDirection = 1;
    } else if (g_lastLineError < -60) {
        g_turnDirection = -1;
    } else if (g_lastLineError > 60) {
        g_turnDirection = 1;
    } else {
        g_turnDirection = DEFAULT_TURN_RIGHT;
    }

    g_carState = CAR_TURN;
    g_stateStartedMs = now;
    g_turnLastMs = now;
    g_turnAngleMdeg = 0U;
    g_turnLeftLine = false;
    g_turnReacquireConfirm = 0U;
    g_mpuReadFailures = 0U;
    g_cornerConfirm = 0U;
    g_cornerCandidateUntil = 0U;
    g_lineLostActive = false;
    apply_turn_arc(false);
}

static void begin_search(int32_t line_error, bool completes_corner,
    uint32_t now)
{
    if (line_error < -60) {
        g_turnDirection = -1;
    } else if (line_error > 60) {
        g_turnDirection = 1;
    } else if (g_lastLineError < -60) {
        g_turnDirection = -1;
    } else if (g_lastLineError > 60) {
        g_turnDirection = 1;
    } else {
        g_turnDirection = DEFAULT_TURN_RIGHT;
    }

    g_carState = CAR_SEARCH;
    g_stateStartedMs = now;
    g_searchLineConfirm = 0U;
    g_searchCompletesCorner = completes_corner;
    g_cornerConfirm = 0U;
    g_cornerCandidateUntil = 0U;
    g_lineLostActive = false;
    apply_turn_arc(true);
}

static void finish_corner(uint32_t now)
{
    g_cornerCount++;
    g_cornerLockoutUntil = now + CORNER_LOCKOUT_MS;
    g_cornerCandidateUntil = 0U;
    g_lineLostActive = false;

    if (!g_runContinuous &&
        (g_cornerCount >= ((uint32_t) g_targetLaps * 4U))) {
        apply_wheel_speeds(0, 0);
        g_carState = CAR_DONE;
    } else {
        apply_wheel_speeds(RECOVERY_BASE_SPEED, RECOVERY_BASE_SPEED);
        g_carState = CAR_RECOVER;
    }
    g_stateStartedMs = now;
}

static void update_turn(uint32_t now)
{
    int32_t gyro_mdps;
    uint32_t elapsed_ms = now - g_turnLastMs;
    bool angle_ready;

    g_turnLastMs = now;
    if (!MPU6050_SW_ReadGyroZDpsMilli(&gyro_mdps)) {
        if (g_mpuReadFailures < UINT8_MAX) {
            g_mpuReadFailures++;
        }
    } else {
        g_mpuReadFailures = 0U;
        if (gyro_mdps < 0) {
            gyro_mdps = -gyro_mdps;
        }
        if (gyro_mdps < GYRO_DEADBAND_MDPS) {
            gyro_mdps = 0;
        }
        g_turnAngleMdeg += ((uint32_t) gyro_mdps * elapsed_ms) / 1000U;
    }

    if (((g_turnAngleMdeg >= TURN_LINE_LEAVE_MDEG) ||
            ((now - g_stateStartedMs) >= TURN_LINE_LEAVE_FALLBACK_MS)) &&
        !center_line_is_black()) {
        g_turnLeftLine = true;
    }

    if (g_turnAngleMdeg >= TURN_SLOWDOWN_MDEG) {
        apply_turn_arc(true);
    }

    angle_ready = (g_turnAngleMdeg >= TURN_REACQUIRE_MIN_MDEG) ||
        ((now - g_stateStartedMs) >= TURN_REACQUIRE_FALLBACK_MS);
    if (angle_ready &&
        g_turnLeftLine && center_line_is_black()) {
        if (g_turnReacquireConfirm < TURN_REACQUIRE_SAMPLES) {
            g_turnReacquireConfirm++;
        }
    } else {
        g_turnReacquireConfirm = 0U;
    }

    if (g_turnReacquireConfirm >= TURN_REACQUIRE_SAMPLES) {
        finish_corner(now);
    } else if (g_turnAngleMdeg >= TURN_SEARCH_LIMIT_MDEG) {
        begin_search((g_turnDirection > 0) ? 100 : -100, true, now);
    } else if ((now - g_stateStartedMs) >= TURN_TIMEOUT_MS) {
        begin_search((g_turnDirection > 0) ? 100 : -100, true, now);
    }
}

static void update_straight_test(uint32_t now)
{
    uint32_t elapsed_ms = now - g_straightLastMs;
    int32_t gyro_mdps;
    int32_t correction;
    int64_t heading_delta;

    g_straightLastMs = now;
    if ((now - g_stateStartedMs) >= STRAIGHT_TEST_DURATION_MS) {
        stop_run(CAR_DONE, now);
        return;
    }

    if (!MPU6050_SW_ReadGyroZDpsMilli(&gyro_mdps)) {
        if (g_straightMpuReadFailures < UINT8_MAX) {
            g_straightMpuReadFailures++;
        }
        if (g_straightMpuReadFailures >= STRAIGHT_MPU_FAILURE_LIMIT) {
            stop_run(CAR_ERROR, now);
            return;
        }
    } else {
        g_straightMpuReadFailures = 0U;
        if ((gyro_mdps > -GYRO_DEADBAND_MDPS) &&
            (gyro_mdps < GYRO_DEADBAND_MDPS)) {
            gyro_mdps = 0;
        }
        heading_delta = ((int64_t) gyro_mdps * elapsed_ms) / 1000;
        g_straightHeadingMdeg = clamp_i32(
            g_straightHeadingMdeg + (int32_t) heading_delta,
            -STRAIGHT_HEADING_LIMIT_MDEG,
            STRAIGHT_HEADING_LIMIT_MDEG);
    }

    correction = clamp_i32(
        (g_straightHeadingMdeg / STRAIGHT_HEADING_DIVISOR) *
            STRAIGHT_GYRO_CORRECTION_SIGN,
        -STRAIGHT_MAX_CORRECTION, STRAIGHT_MAX_CORRECTION);
    apply_wheel_speeds(
        STRAIGHT_TEST_BASE_SPEED + correction,
        STRAIGHT_TEST_BASE_SPEED - correction);
}

static void control_update(uint32_t now)
{
    int32_t line_error = g_lastLineError;
    uint8_t black_count;
    uint8_t corner_black_count;

    GraySensor_Read(g_grayAdc);
    black_count = analyze_line(&line_error);
    corner_black_count = count_corner_black();

    switch (g_carState) {
        case CAR_FOLLOW:
            if (black_count == 0U) {
                if (time_reached(now, g_cornerLockoutUntil) &&
                    (g_cornerConfirm >= CORNER_FALLBACK_SAMPLES) &&
                    !time_reached(now, g_cornerCandidateUntil)) {
                    begin_corner(g_lastLineError, now);
                    break;
                }

                if (!g_lineLostActive) {
                    g_lineLostActive = true;
                    g_lineLostStartedMs = now;
                }
                if ((now - g_lineLostStartedMs) <= LINE_LOST_GRACE_MS) {
                    int32_t correction = clamp_i32(
                        g_lastLineError / LINE_ERROR_DIVISOR,
                        -MAX_STEERING_CORRECTION, MAX_STEERING_CORRECTION);
                    apply_wheel_speeds(
                        clamp_i32(LINE_LOST_SPEED + correction, 0, FOLLOW_MAX_SPEED),
                        clamp_i32(LINE_LOST_SPEED - correction, 0, FOLLOW_MAX_SPEED));
                } else {
                    begin_search(g_lastLineError, false, now);
                }
                break;
            }

            g_lineLostActive = false;
            g_lastLineError = line_error;
            if (time_reached(now, g_cornerLockoutUntil) &&
                (corner_black_count >= CORNER_BLACK_COUNT)) {
                g_cornerCandidateUntil = now + CORNER_CANDIDATE_HOLD_MS;
                if (g_cornerConfirm < CORNER_CONFIRM_SAMPLES) {
                    g_cornerConfirm++;
                }
                if (g_cornerConfirm >= CORNER_CONFIRM_SAMPLES) {
                    begin_corner(line_error, now);
                    break;
                }
            } else {
                int32_t correction = clamp_i32(
                    line_error / LINE_ERROR_DIVISOR,
                    -MAX_STEERING_CORRECTION, MAX_STEERING_CORRECTION);
                int32_t left = clamp_i32(FOLLOW_BASE_SPEED + correction,
                    0, FOLLOW_MAX_SPEED);
                int32_t right = clamp_i32(FOLLOW_BASE_SPEED - correction,
                    0, FOLLOW_MAX_SPEED);

                if (time_reached(now, g_cornerCandidateUntil)) {
                    g_cornerConfirm = 0U;
                }
                apply_wheel_speeds(left, right);
            }
            break;

        case CAR_TURN:
            update_turn(now);
            break;

        case CAR_RECOVER:
            if (black_count == 0U) {
                if (!g_lineLostActive) {
                    g_lineLostActive = true;
                    g_lineLostStartedMs = now;
                }
                if ((now - g_lineLostStartedMs) <= LINE_LOST_GRACE_MS) {
                    apply_wheel_speeds(RECOVERY_BASE_SPEED, RECOVERY_BASE_SPEED);
                } else {
                    begin_search(g_lastLineError, false, now);
                }
                break;
            }

            g_lineLostActive = false;
            g_lastLineError = line_error;
            {
                int32_t correction = clamp_i32(
                    line_error / LINE_ERROR_DIVISOR,
                    -RECOVERY_MAX_CORRECTION, RECOVERY_MAX_CORRECTION);
                int32_t left = clamp_i32(RECOVERY_BASE_SPEED + correction,
                    0, RECOVERY_MAX_SPEED);
                int32_t right = clamp_i32(RECOVERY_BASE_SPEED - correction,
                    0, RECOVERY_MAX_SPEED);

                apply_wheel_speeds(left, right);
            }
            if ((now - g_stateStartedMs) >= CORNER_RECOVERY_MS) {
                g_carState = CAR_FOLLOW;
                g_stateStartedMs = now;
            }
            break;

        case CAR_SEARCH:
            if (black_count == 0U) {
                g_searchLineConfirm = 0U;
                apply_turn_arc(true);
                break;
            }

            g_lastLineError = line_error;
            {
                int32_t correction = clamp_i32(
                    line_error / LINE_ERROR_DIVISOR,
                    -RECOVERY_MAX_CORRECTION, RECOVERY_MAX_CORRECTION);
                apply_wheel_speeds(
                    clamp_i32(RECOVERY_BASE_SPEED + correction,
                        0, RECOVERY_MAX_SPEED),
                    clamp_i32(RECOVERY_BASE_SPEED - correction,
                        0, RECOVERY_MAX_SPEED));
            }

            if (center_line_is_black()) {
                if (g_searchLineConfirm < SEARCH_LINE_SAMPLES) {
                    g_searchLineConfirm++;
                }
            } else {
                g_searchLineConfirm = 0U;
            }

            if (g_searchLineConfirm >= SEARCH_LINE_SAMPLES) {
                if (g_searchCompletesCorner) {
                    finish_corner(now);
                } else {
                    g_carState = CAR_RECOVER;
                    g_stateStartedMs = now;
                    g_lineLostActive = false;
                }
            }
            break;

        case CAR_STRAIGHT_TEST:
            update_straight_test(now);
            break;

        case CAR_IDLE:
        case CAR_DONE:
        case CAR_ERROR:
        default:
            apply_wheel_speeds(0, 0);
            break;
    }
}

static const char *state_name(void)
{
    switch (g_carState) {
        case CAR_FOLLOW: return "RUN";
        case CAR_TURN: return "TURN";
        case CAR_RECOVER: return "LOCK";
        case CAR_SEARCH: return "FIND";
        case CAR_STRAIGHT_TEST: return "LINE";
        case CAR_DONE: return "DONE";
        case CAR_ERROR: return "ERR";
        default: return "IDLE";
    }
}

static char bluetooth_status_marker(void)
{
    if (g_remoteReceiveCount == 0U) {
        return '?';
    }
    if ((g_milliseconds - g_lastRemoteReceiveMs) <=
        REMOTE_LINK_TIMEOUT_MS) {
        return '+';
    }
    return '-';
}

static void oled_show_gray(void)
{
    char line[22];
    uint32_t row;
    uint32_t completed_laps = g_cornerCount / 4U;
    uint32_t corner_in_lap = g_cornerCount % 4U;

    OLED_Clear();
    OLED_DrawRectangle(0, 0, 127, 63, 1);
    if (g_carState == CAR_STRAIGHT_TEST) {
        (void) snprintf(line, sizeof(line), "B%c Y:%ld LINE",
            bluetooth_status_marker(),
            (long) (g_straightHeadingMdeg / 1000));
    } else if (g_runContinuous) {
        (void) snprintf(line, sizeof(line), "B%c L%lu/* C%lu %s",
            bluetooth_status_marker(),
            (unsigned long) completed_laps,
            (unsigned long) corner_in_lap, state_name());
    } else if (g_targetLaps != 0U) {
        (void) snprintf(line, sizeof(line), "B%c L%lu/%u C%lu %s",
            bluetooth_status_marker(),
            (unsigned long) completed_laps, g_targetLaps,
            (unsigned long) corner_in_lap, state_name());
    } else {
        (void) snprintf(line, sizeof(line), "B%c L%lu/- C%lu %s",
            bluetooth_status_marker(),
            (unsigned long) completed_laps,
            (unsigned long) corner_in_lap, state_name());
    }
    OLED_ShowString(4, 3, line, 8, 1);

    for (row = 0U; row < 4U; row++) {
        uint32_t channel0 = row * 2U;
        uint32_t channel1 = channel0 + 1U;
        (void) snprintf(line, sizeof(line), "%lu:%4u %lu:%4u",
            (unsigned long) channel0, g_grayAdc[channel0],
            (unsigned long) channel1, g_grayAdc[channel1]);
        OLED_ShowString(8, (uint8_t) (15U + (row * 12U)), line, 8, 1);
    }
    OLED_Refresh();
}

static void oled_show_mpu_init(void)
{
    OLED_Clear();
    OLED_ShowString(10, 8, "MPU CAL", 16, 1);
    OLED_ShowString(16, 38, "KEEP STILL", 8, 1);
    OLED_Refresh();
}

int main(void)
{
    uint32_t last_key_poll;
    uint32_t last_control;
    uint32_t last_oled_update;

    SYSCFG_DL_init();
    (void) SysTick_Config(CPUCLK_FREQ / 1000U);
    DL_TimerA_startCounter(PWM_0_INST);
    ClosedLoopDrive_Init();
    HC04_Init();
    (void) HC04_EnsureRole(HC04_ROLE_SLAVE);

    OLED_Init();
    OLED_ColorTurn(0);
    OLED_DisplayTurn(0);
    oled_show_mpu_init();

    g_mpuReady = MPU6050_SW_Init() && MPU6050_SW_CalibrateGyroZ();
    g_carState = g_mpuReady ? CAR_IDLE : CAR_ERROR;
    keys_init();
    GraySensor_Read(g_grayAdc);
    oled_show_gray();

    last_key_poll = g_milliseconds;
    last_control = g_milliseconds;
    last_oled_update = g_milliseconds;

    while (1) {
        uint32_t now = g_milliseconds;

        remote_poll(now);
        if ((now - last_key_poll) >= KEY_POLL_MS) {
            last_key_poll = now;
            keys_poll(now);
        }
        if ((now - last_control) >= CONTROL_PERIOD_MS) {
            uint32_t elapsed_ms = now - last_control;
            last_control = now;
            control_update(now);
            ClosedLoopDrive_Update(elapsed_ms);
            g_leftSpeed = ClosedLoopDrive_GetLeftSpeed();
            g_rightSpeed = ClosedLoopDrive_GetRightSpeed();
        }
        if ((now - last_oled_update) >= OLED_UPDATE_MS) {
            last_oled_update = now;
            oled_show_gray();
        }
    }
}
