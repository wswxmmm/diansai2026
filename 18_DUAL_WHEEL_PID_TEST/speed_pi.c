#include "speed_pi.h"

#define SPEED_PI_NOMINAL_PERIOD_MS   (50U)

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

void SpeedPI_Reset(SpeedPI *controller)
{
    if (controller == 0) {
        return;
    }
    controller->integral = 0;
    controller->error = 0;
    controller->output_percent = 0U;
}

uint32_t SpeedPI_Update(SpeedPI *controller, int32_t target_pps,
    int32_t measured_pps, uint32_t elapsed_ms)
{
    int32_t feedforward;
    int32_t proportional;
    int32_t integral_term;
    int32_t output;
    int64_t integral_delta;

    if ((controller == 0) || (target_pps <= 0)) {
        SpeedPI_Reset(controller);
        return 0U;
    }
    if (elapsed_ms == 0U) {
        elapsed_ms = SPEED_PI_NOMINAL_PERIOD_MS;
    }

    controller->error = target_pps - measured_pps;
    integral_delta = ((int64_t) controller->error * elapsed_ms) /
        SPEED_PI_NOMINAL_PERIOD_MS;
    controller->integral = clamp_i32(
        controller->integral + (int32_t) integral_delta,
        -SPEED_PI_INTEGRAL_LIMIT, SPEED_PI_INTEGRAL_LIMIT);

    feedforward = ((target_pps * 100) +
        (SPEED_PI_EXPECTED_MAX_PPS / 2)) / SPEED_PI_EXPECTED_MAX_PPS;
    proportional = controller->error / SPEED_PI_KP_DIVISOR;
    integral_term = controller->integral / SPEED_PI_KI_DIVISOR;
    output = feedforward + proportional + integral_term;
    output = clamp_i32(output, 0, SPEED_PI_MAX_PWM_PERCENT);
    if ((output > 0) && (output < SPEED_PI_MIN_PWM_PERCENT)) {
        output = SPEED_PI_MIN_PWM_PERCENT;
    }

    controller->output_percent = (uint32_t) output;
    return controller->output_percent;
}
