#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

#define GMR_ENCODER_PPR            (500U)
#define ENCODER_DECODE_EDGES       (2U)
#define ENCODER_COUNTS_PER_REV     (GMR_ENCODER_PPR * ENCODER_DECODE_EDGES)
#define SPEED_DECIMAL_SCALE        (1000)

typedef enum {
    ENCODER_LEFT = 0,
    ENCODER_RIGHT = 1,
    ENCODER_COUNT = 2
} EncoderId;

typedef struct {
    int32_t count;
    int32_t delta;
    int32_t pps;
    int32_t rps_milli;
} WheelSpeed;

void Encoder_Init(void);
void Encoder_Reset(void);
WheelSpeed Encoder_UpdateSpeed(EncoderId id, uint32_t elapsed_ms);
int32_t Encoder_GetCount(EncoderId id);

#endif
