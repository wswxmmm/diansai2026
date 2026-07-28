#ifndef CLOSED_LOOP_DRIVE_H
#define CLOSED_LOOP_DRIVE_H

#include "bsp_encoder.h"
#include <stdint.h>

#define CLOSED_LOOP_MAX_SPEED_UNITS (150)
#define CLOSED_LOOP_PPS_PER_UNIT     (1000)

void ClosedLoopDrive_Init(void);
void ClosedLoopDrive_Stop(void);
void ClosedLoopDrive_SetTargets(int32_t left_speed_units,
    int32_t right_speed_units);
void ClosedLoopDrive_Update(uint32_t elapsed_ms);
WheelSpeed ClosedLoopDrive_GetLeftSpeed(void);
WheelSpeed ClosedLoopDrive_GetRightSpeed(void);

#endif
