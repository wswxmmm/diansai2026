#ifndef EMM_MULTI_MOTOR_H
#define EMM_MULTI_MOTOR_H

#include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stdint.h>

#define EMM_MULTI_MAX_FRAME_SIZE 13U

extern volatile uint8_t g_emmMultiLastFrame[EMM_MULTI_MAX_FRAME_SIZE];
extern volatile uint8_t g_emmMultiLastFrameLength;
extern volatile uint32_t g_emmMultiFrameCount;
extern volatile uint32_t g_emmMultiSendErrorCount;

void EmmMulti_Init(void);
bool EmmMulti_Enable(uint8_t address, bool enable);
bool EmmMulti_QueueVelocity(uint8_t address, uint8_t direction,
                            uint16_t rpm, uint8_t acceleration);
bool EmmMulti_QueuePosition(uint8_t address, uint8_t direction,
                            uint16_t rpm, uint8_t acceleration,
                            uint32_t pulses);
bool EmmMulti_QueueStop(uint8_t address);
bool EmmMulti_TriggerSynchronous(void);
bool EmmMulti_SendRaw(const uint8_t *data, uint8_t length);

#endif
