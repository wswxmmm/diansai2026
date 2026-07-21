#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    MOTOR_PORT_1 = 0, /* UART0: PA10 TX, PA11 RX */
    MOTOR_PORT_2,     /* UART1: PA8 TX, PA9 RX */
    MOTOR_PORT_COUNT
} MotorPort;

typedef enum {
    MOTOR_RESPONSE_NONE = 0,
    MOTOR_RESPONSE_ACK,
    MOTOR_RESPONSE_POSITION,
    MOTOR_RESPONSE_UNKNOWN
} MotorResponseType;

typedef struct {
    MotorResponseType type;
    uint8_t address;
    uint8_t function;
    uint8_t status;
    uint8_t length;
    float position_degrees;
} MotorResponse;

void Motor_Init(void);
void Motor_ClearResponse(MotorPort port);

bool Motor_Enable(MotorPort port, uint8_t address, bool enable);
bool Motor_SetVelocity(MotorPort port, uint8_t address, uint8_t direction,
                       uint16_t acceleration, float rpm);
bool Motor_QueueVelocity(MotorPort port, uint8_t address, uint8_t direction,
                         uint16_t acceleration, float rpm);
bool Motor_MoveRelativeDegrees(MotorPort port, uint8_t address,
                               uint8_t direction, float rpm,
                               uint16_t acceleration, float degrees);
bool Motor_QueueMoveRelativeDegrees(MotorPort port, uint8_t address,
                                    uint8_t direction, float rpm,
                                    uint16_t acceleration, float degrees);
bool Motor_Stop(MotorPort port, uint8_t address);
bool Motor_QueueStop(MotorPort port, uint8_t address);
bool Motor_TriggerSynchronous(MotorPort port);
bool Motor_RequestPosition(MotorPort port, uint8_t address);

bool Motor_SendRaw(MotorPort port, const uint8_t *data, uint8_t length);

/* Each UART has an independent receive buffer. */
bool Motor_TakeResponse(MotorPort port, MotorResponse *response);

#endif
