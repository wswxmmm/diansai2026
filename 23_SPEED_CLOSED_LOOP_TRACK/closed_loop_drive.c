#include "closed_loop_drive.h"
#include "bsp_tb6612.h"
#include "speed_pid.h"
#include <stdbool.h>

#define TARGET_RAMP_PPS_PER_SECOND (50000)
#define SPEED_FILTER_DIVISOR       (5)
#define DRIVE_DITHER_MIN_PERMILLE  (50U)

#define TUNED_KP_X1000             (20)
#define TUNED_KI_X1000             (30)
#define TUNED_KD_X1000             (0)

#define LEFT_FORWARD_DIR           (0U)
#define LEFT_REVERSE_DIR           (1U)
#define RIGHT_FORWARD_DIR          (1U)
#define RIGHT_REVERSE_DIR          (0U)

typedef struct {
    SpeedPID controller;
    WheelSpeed speed;
    int32_t filtered_pps;
    int32_t target_speed_units;
    int32_t ramped_target_pps;
    int8_t direction;
    uint32_t drive_accumulator;
    uint32_t requested_output;
} ClosedLoopWheel;

static ClosedLoopWheel g_leftWheel;
static ClosedLoopWheel g_rightWheel;

static const SpeedPIDGains g_tunedGains = {
    TUNED_KP_X1000,
    TUNED_KI_X1000,
    TUNED_KD_X1000
};

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

static int8_t speed_direction(int32_t speed_units)
{
    if (speed_units > 0) {
        return 1;
    }
    if (speed_units < 0) {
        return -1;
    }
    return 0;
}

static void reset_wheel_control(ClosedLoopWheel *wheel, bool clear_filter)
{
    SpeedPID_Reset(&wheel->controller);
    wheel->ramped_target_pps = 0;
    wheel->drive_accumulator = 0U;
    wheel->requested_output = 0U;
    if (clear_filter) {
        wheel->filtered_pps = 0;
    }
}

static int32_t filter_speed(int32_t filtered, int32_t measured)
{
    return filtered + ((measured - filtered) / SPEED_FILTER_DIVISOR);
}

static void update_ramped_target(ClosedLoopWheel *wheel,
    int32_t command_pps, uint32_t elapsed_ms)
{
    int32_t step = (TARGET_RAMP_PPS_PER_SECOND * (int32_t) elapsed_ms) /
        1000;

    if (step < 1) {
        step = 1;
    }
    if (wheel->ramped_target_pps < command_pps) {
        wheel->ramped_target_pps = clamp_i32(
            wheel->ramped_target_pps + step, 0, command_pps);
    } else if (wheel->ramped_target_pps > command_pps) {
        /* Apply reductions immediately so the inside wheel still slows
         * promptly when project 16 enters a corner. */
        wheel->ramped_target_pps = command_pps;
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

static uint32_t update_wheel_control(ClosedLoopWheel *wheel,
    uint32_t elapsed_ms)
{
    int32_t command_pps;

    wheel->filtered_pps = filter_speed(wheel->filtered_pps,
        abs_i32(wheel->speed.pps));
    if ((wheel->direction == 0) || (wheel->target_speed_units == 0)) {
        wheel->requested_output = 0U;
        return 0U;
    }

    command_pps = abs_i32(wheel->target_speed_units) *
        CLOSED_LOOP_PPS_PER_UNIT;
    update_ramped_target(wheel, command_pps, elapsed_ms);
    wheel->requested_output = SpeedPID_Update(&wheel->controller,
        &g_tunedGains, wheel->ramped_target_pps,
        wheel->filtered_pps, elapsed_ms);

    return dither_drive_output(wheel->requested_output,
        &wheel->drive_accumulator);
}

static void set_wheel_target(ClosedLoopWheel *wheel, int32_t speed_units)
{
    int8_t new_direction;

    speed_units = clamp_i32(speed_units,
        -CLOSED_LOOP_MAX_SPEED_UNITS, CLOSED_LOOP_MAX_SPEED_UNITS);
    new_direction = speed_direction(speed_units);
    if (new_direction != wheel->direction) {
        reset_wheel_control(wheel, true);
    }
    wheel->target_speed_units = speed_units;
    wheel->direction = new_direction;
    if (new_direction == 0) {
        reset_wheel_control(wheel, false);
    }
}

void ClosedLoopDrive_Init(void)
{
    g_leftWheel.target_speed_units = 0;
    g_leftWheel.direction = 0;
    g_leftWheel.filtered_pps = 0;
    g_leftWheel.speed.count = 0;
    g_leftWheel.speed.delta = 0;
    g_leftWheel.speed.pps = 0;
    g_leftWheel.speed.rps_milli = 0;
    g_rightWheel.target_speed_units = 0;
    g_rightWheel.direction = 0;
    g_rightWheel.filtered_pps = 0;
    g_rightWheel.speed.count = 0;
    g_rightWheel.speed.delta = 0;
    g_rightWheel.speed.pps = 0;
    g_rightWheel.speed.rps_milli = 0;
    reset_wheel_control(&g_leftWheel, true);
    reset_wheel_control(&g_rightWheel, true);
    Encoder_Init();
    TB6612_Motor_Stop();
}

void ClosedLoopDrive_Stop(void)
{
    set_wheel_target(&g_leftWheel, 0);
    set_wheel_target(&g_rightWheel, 0);
    Encoder_Reset();
    TB6612_Motor_Stop();
}

void ClosedLoopDrive_SetTargets(int32_t left_speed_units,
    int32_t right_speed_units)
{
    int8_t old_left_direction = g_leftWheel.direction;
    int8_t old_right_direction = g_rightWheel.direction;

    set_wheel_target(&g_leftWheel, left_speed_units);
    set_wheel_target(&g_rightWheel, right_speed_units);

    if ((g_leftWheel.direction == 0) ||
        (g_leftWheel.direction != old_left_direction)) {
        AO_Stop();
    }
    if ((g_rightWheel.direction == 0) ||
        (g_rightWheel.direction != old_right_direction)) {
        BO_Stop();
    }
}

void ClosedLoopDrive_Update(uint32_t elapsed_ms)
{
    uint32_t left_output;
    uint32_t right_output;

    if (elapsed_ms == 0U) {
        elapsed_ms = 1U;
    }
    g_leftWheel.speed = Encoder_UpdateSpeed(ENCODER_LEFT, elapsed_ms);
    g_rightWheel.speed = Encoder_UpdateSpeed(ENCODER_RIGHT, elapsed_ms);
    left_output = update_wheel_control(&g_leftWheel, elapsed_ms);
    right_output = update_wheel_control(&g_rightWheel, elapsed_ms);

    if (g_leftWheel.direction == 0) {
        AO_Stop();
    } else if (left_output == 0U) {
        AO_Coast();
    } else {
        AO_ControlPermille((g_leftWheel.direction > 0) ?
            LEFT_FORWARD_DIR : LEFT_REVERSE_DIR, left_output);
    }

    if (g_rightWheel.direction == 0) {
        BO_Stop();
    } else if (right_output == 0U) {
        BO_Coast();
    } else {
        BO_ControlPermille((g_rightWheel.direction > 0) ?
            RIGHT_FORWARD_DIR : RIGHT_REVERSE_DIR, right_output);
    }
}

WheelSpeed ClosedLoopDrive_GetLeftSpeed(void)
{
    return g_leftWheel.speed;
}

WheelSpeed ClosedLoopDrive_GetRightSpeed(void)
{
    return g_rightWheel.speed;
}
