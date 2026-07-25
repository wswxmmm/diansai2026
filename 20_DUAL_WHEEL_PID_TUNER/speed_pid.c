#include "speed_pid.h"

#define SPEED_PID_INTEGRAL_LIMIT_MS      (3000000)
#define SPEED_PID_DERIVATIVE_LIMIT       (100000)
#define SPEED_PID_DERIVATIVE_FILTER_DIV  (4)

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

static int32_t clamp_i64_to_i32(int64_t value, int32_t minimum,
    int32_t maximum)
{
    if (value < (int64_t) minimum) {
        return minimum;
    }
    if (value > (int64_t) maximum) {
        return maximum;
    }
    return (int32_t) value;
}

static int32_t calculate_output(const SpeedPID *controller,
    const SpeedPIDGains *gains, int32_t target_pps,
    int32_t integral_error_ms)
{
    int64_t feedforward;
    int64_t proportional;
    int64_t integral;
    int64_t derivative;

    feedforward = ((int64_t) target_pps * 1000) /
        SPEED_PID_EXPECTED_MAX_PPS;
    proportional = ((int64_t) gains->kp_x1000 *
        controller->error_pps) / SPEED_PID_GAIN_SCALE;
    integral = ((int64_t) gains->ki_x1000 * integral_error_ms) /
        (SPEED_PID_GAIN_SCALE * 1000LL);
    derivative = ((int64_t) gains->kd_x1000 *
        controller->derivative_pps_per_s) / SPEED_PID_GAIN_SCALE;

    return clamp_i64_to_i32(feedforward + proportional + integral - derivative,
        -SPEED_PID_MAX_OUTPUT_PERMILLE,
        SPEED_PID_MAX_OUTPUT_PERMILLE * 2);
}

void SpeedPID_Reset(SpeedPID *controller)
{
    if (controller == 0) {
        return;
    }

    controller->error_pps = 0;
    controller->integral_error_ms = 0;
    controller->derivative_pps_per_s = 0;
    controller->previous_measured_pps = 0;
    controller->output_permille = 0U;
    controller->initialized = 0U;
}

uint32_t SpeedPID_Update(SpeedPID *controller,
    const SpeedPIDGains *gains, int32_t target_pps,
    int32_t measured_pps, uint32_t elapsed_ms)
{
    int32_t raw_derivative;
    int32_t candidate_integral;
    int32_t candidate_output;
    int32_t output;
    int64_t derivative_numerator;
    int64_t integral_candidate_64;

    if ((controller == 0) || (gains == 0)) {
        return 0U;
    }
    if (target_pps <= 0) {
        SpeedPID_Reset(controller);
        return 0U;
    }
    if (elapsed_ms == 0U) {
        elapsed_ms = 1U;
    }

    controller->error_pps = target_pps - measured_pps;
    if (controller->initialized == 0U) {
        controller->previous_measured_pps = measured_pps;
        controller->derivative_pps_per_s = 0;
        controller->initialized = 1U;
    } else {
        derivative_numerator = (int64_t)
            (measured_pps - controller->previous_measured_pps) * 1000;
        raw_derivative = clamp_i64_to_i32(
            derivative_numerator / elapsed_ms,
            -SPEED_PID_DERIVATIVE_LIMIT, SPEED_PID_DERIVATIVE_LIMIT);
        controller->derivative_pps_per_s +=
            (raw_derivative - controller->derivative_pps_per_s) /
                SPEED_PID_DERIVATIVE_FILTER_DIV;
        controller->previous_measured_pps = measured_pps;
    }

    integral_candidate_64 = (int64_t) controller->integral_error_ms +
        ((int64_t) controller->error_pps * elapsed_ms);
    candidate_integral = clamp_i64_to_i32(integral_candidate_64,
        -SPEED_PID_INTEGRAL_LIMIT_MS, SPEED_PID_INTEGRAL_LIMIT_MS);
    candidate_output = calculate_output(controller, gains, target_pps,
        candidate_integral);

    if (!((candidate_output >= SPEED_PID_MAX_OUTPUT_PERMILLE &&
              controller->error_pps > 0) ||
            (candidate_output <= SPEED_PID_MIN_OUTPUT_PERMILLE &&
              controller->error_pps < 0))) {
        controller->integral_error_ms = candidate_integral;
    }

    output = calculate_output(controller, gains, target_pps,
        controller->integral_error_ms);
    output = clamp_i32(output, SPEED_PID_MIN_OUTPUT_PERMILLE,
        SPEED_PID_MAX_OUTPUT_PERMILLE);
    controller->output_permille = (uint32_t) output;

    return controller->output_permille;
}
