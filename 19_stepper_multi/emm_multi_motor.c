#include "emm_multi_motor.h"

#include <stddef.h>

#define EMM_MULTI_FRAME_END           0x6BU
#define EMM_MULTI_TX_TIMEOUT          100000U
#define EMM_MULTI_SYNC_FLAG           0x01U
/* Vendor Emm example: 0 = relative to the previous target position. */
#define EMM_MULTI_RELATIVE_MODE       0x00U

volatile uint8_t g_emmMultiLastFrame[EMM_MULTI_MAX_FRAME_SIZE];
volatile uint8_t g_emmMultiLastFrameLength;
volatile uint32_t g_emmMultiFrameCount;
volatile uint32_t g_emmMultiSendErrorCount;

static bool send_byte(uint8_t data)
{
    uint32_t timeout = EMM_MULTI_TX_TIMEOUT;

    while (DL_UART_isBusy(UART_BUS_INST)) {
        if (timeout-- == 0U) {
            return false;
        }
    }

    DL_UART_Main_transmitData(UART_BUS_INST, data);
    return true;
}

void EmmMulti_Init(void)
{
    uint8_t i;

    g_emmMultiLastFrameLength = 0U;
    g_emmMultiFrameCount = 0U;
    g_emmMultiSendErrorCount = 0U;
    for (i = 0U; i < EMM_MULTI_MAX_FRAME_SIZE; i++) {
        g_emmMultiLastFrame[i] = 0U;
    }
}

bool EmmMulti_SendRaw(const uint8_t *data, uint8_t length)
{
    uint8_t i;

    if ((data == NULL) || (length == 0U) ||
        (length > EMM_MULTI_MAX_FRAME_SIZE)) {
        g_emmMultiSendErrorCount++;
        return false;
    }

    g_emmMultiLastFrameLength = length;
    for (i = 0U; i < length; i++) {
        g_emmMultiLastFrame[i] = data[i];
    }

    for (i = 0U; i < length; i++) {
        if (!send_byte(data[i])) {
            g_emmMultiSendErrorCount++;
            return false;
        }
    }

    g_emmMultiFrameCount++;
    return true;
}

bool EmmMulti_Enable(uint8_t address, bool enable)
{
    const uint8_t command[6] = {
        address, 0xF3U, 0xABU, (uint8_t)enable, 0x00U,
        EMM_MULTI_FRAME_END
    };

    if (address == 0U) {
        return false;
    }
    return EmmMulti_SendRaw(command, (uint8_t)sizeof(command));
}

bool EmmMulti_QueueVelocity(uint8_t address, uint8_t direction,
                            uint16_t rpm, uint8_t acceleration)
{
    uint8_t command[8];

    if ((address == 0U) || (direction > 1U) || (rpm > 5000U)) {
        return false;
    }

    command[0] = address;
    command[1] = 0xF6U;
    command[2] = direction;
    command[3] = (uint8_t)(rpm >> 8);
    command[4] = (uint8_t)rpm;
    command[5] = acceleration;
    command[6] = EMM_MULTI_SYNC_FLAG;
    command[7] = EMM_MULTI_FRAME_END;

    return EmmMulti_SendRaw(command, (uint8_t)sizeof(command));
}

bool EmmMulti_QueuePosition(uint8_t address, uint8_t direction,
                            uint16_t rpm, uint8_t acceleration,
                            uint32_t pulses)
{
    uint8_t command[13];

    if ((address == 0U) || (direction > 1U) ||
        (rpm > 5000U) || (pulses == 0U)) {
        return false;
    }

    command[0]  = address;
    command[1]  = 0xFDU;
    command[2]  = direction;
    command[3]  = (uint8_t)(rpm >> 8);
    command[4]  = (uint8_t)rpm;
    command[5]  = acceleration;
    command[6]  = (uint8_t)(pulses >> 24);
    command[7]  = (uint8_t)(pulses >> 16);
    command[8]  = (uint8_t)(pulses >> 8);
    command[9]  = (uint8_t)pulses;
    command[10] = EMM_MULTI_RELATIVE_MODE;
    command[11] = EMM_MULTI_SYNC_FLAG;
    command[12] = EMM_MULTI_FRAME_END;

    return EmmMulti_SendRaw(command, (uint8_t)sizeof(command));
}

bool EmmMulti_QueueStop(uint8_t address)
{
    const uint8_t command[5] = {
        address, 0xFEU, 0x98U, EMM_MULTI_SYNC_FLAG,
        EMM_MULTI_FRAME_END
    };

    if (address == 0U) {
        return false;
    }
    return EmmMulti_SendRaw(command, (uint8_t)sizeof(command));
}

bool EmmMulti_TriggerSynchronous(void)
{
    const uint8_t command[4] = {
        0x00U, 0xFFU, 0x66U, EMM_MULTI_FRAME_END
    };

    return EmmMulti_SendRaw(command, (uint8_t)sizeof(command));
}
