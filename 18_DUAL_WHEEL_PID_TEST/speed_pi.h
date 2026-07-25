#ifndef SPEED_PI_H
#define SPEED_PI_H

#include <stdint.h>

#define SPEED_PI_KP_DIVISOR          (100)
#define SPEED_PI_KI_DIVISOR          (2000)
#define SPEED_PI_INTEGRAL_LIMIT      (20000)
#define SPEED_PI_EXPECTED_MAX_PPS    (5000)
#define SPEED_PI_MIN_PWM_PERCENT     (10)
#define SPEED_PI_MAX_PWM_PERCENT     (60)

typedef struct {
    int32_t integral;
    int32_t error;
    uint32_t output_percent;
} SpeedPI;

void SpeedPI_Reset(SpeedPI *controller);
uint32_t SpeedPI_Update(SpeedPI *controller, int32_t target_pps,
    int32_t measured_pps, uint32_t elapsed_ms);

#endif
