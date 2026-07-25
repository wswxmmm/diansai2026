#ifndef SPEED_PID_H
#define SPEED_PID_H

#include <stdint.h>

#define SPEED_PID_GAIN_SCALE             (1000)
#define SPEED_PID_EXPECTED_MAX_PPS       (5000)
#define SPEED_PID_MIN_OUTPUT_PERMILLE    (80)
#define SPEED_PID_MAX_OUTPUT_PERMILLE    (600)

typedef struct {
    int32_t kp_x1000;
    int32_t ki_x1000;
    int32_t kd_x1000;
} SpeedPIDGains;

typedef struct {
    int32_t error_pps;
    int32_t integral_error_ms;
    int32_t derivative_pps_per_s;
    int32_t previous_measured_pps;
    uint32_t output_permille;
    uint8_t initialized;
} SpeedPID;

void SpeedPID_Reset(SpeedPID *controller);
uint32_t SpeedPID_Update(SpeedPID *controller,
    const SpeedPIDGains *gains, int32_t target_pps,
    int32_t measured_pps, uint32_t elapsed_ms);

#endif
