#include "motor_driver.h"
#include "motor_driver_config.h"

#include <stddef.h>

#define MOTOR_FRAME_END             0x6BU
#define MOTOR_RX_BUFFER_SIZE        16U
#define MOTOR_TX_TIMEOUT_COUNT      100000U

#if ((MOTOR_PROTOCOL != MOTOR_PROTOCOL_X) && \
     (MOTOR_PROTOCOL != MOTOR_PROTOCOL_EMM))
#error "MOTOR_PROTOCOL must be MOTOR_PROTOCOL_X or MOTOR_PROTOCOL_EMM"
#endif

static volatile uint8_t g_rxBuffer[MOTOR_PORT_COUNT][MOTOR_RX_BUFFER_SIZE];
static volatile uint8_t g_rxCount[MOTOR_PORT_COUNT];
static volatile uint8_t g_rxExpected[MOTOR_PORT_COUNT];
static volatile bool g_rxReady[MOTOR_PORT_COUNT];

static bool port_is_valid(MotorPort port)
{
    return ((port == MOTOR_PORT_1) || (port == MOTOR_PORT_2));
}

static void port_disable_irq(MotorPort port)
{
    if (port == MOTOR_PORT_1) {
        NVIC_DisableIRQ(UART_0_INST_INT_IRQN);
    } else {
        NVIC_DisableIRQ(UART_1_INST_INT_IRQN);
    }
}

static void port_enable_irq(MotorPort port)
{
    if (port == MOTOR_PORT_1) {
        NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
    } else {
        NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
    }
}

static void port_clear_pending_irq(MotorPort port)
{
    if (port == MOTOR_PORT_1) {
        NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    } else {
        NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
    }
}

static bool port_uart_is_busy(MotorPort port)
{
    if (port == MOTOR_PORT_1) {
        return DL_UART_isBusy(UART_0_INST);
    }
    return DL_UART_isBusy(UART_1_INST);
}

static void port_transmit(MotorPort port, uint8_t data)
{
    if (port == MOTOR_PORT_1) {
        DL_UART_Main_transmitData(UART_0_INST, data);
    } else {
        DL_UART_Main_transmitData(UART_1_INST, data);
    }
}

static uint8_t response_length_for_function(uint8_t function)
{
    if (function == 0x36U) {
        return 8U;
    }

    return 4U;
}

static bool send_byte(MotorPort port, uint8_t data)
{
    uint32_t timeout = MOTOR_TX_TIMEOUT_COUNT;

    while (port_uart_is_busy(port)) {
        if (timeout-- == 0U) {
            return false;
        }
    }

    port_transmit(port, data);
    return true;
}

void Motor_ClearResponse(MotorPort port)
{
    if (!port_is_valid(port)) {
        return;
    }

    port_disable_irq(port);
    g_rxCount[port]    = 0U;
    g_rxExpected[port] = 0U;
    g_rxReady[port]    = false;
    port_clear_pending_irq(port);
    port_enable_irq(port);
}

void Motor_Init(void)
{
    MotorPort port;

    for (port = MOTOR_PORT_1; port < MOTOR_PORT_COUNT; port++) {
        g_rxCount[port]    = 0U;
        g_rxExpected[port] = 0U;
        g_rxReady[port]    = false;
        port_clear_pending_irq(port);
        port_enable_irq(port);
    }
}

bool Motor_SendRaw(MotorPort port, const uint8_t *data, uint8_t length)
{
    uint8_t i;

    if (!port_is_valid(port) || (data == NULL) || (length == 0U)) {
        return false;
    }

    Motor_ClearResponse(port);

    for (i = 0U; i < length; i++) {
        if (!send_byte(port, data[i])) {
            return false;
        }
    }

    return true;
}

bool Motor_Enable(MotorPort port, uint8_t address, bool enable)
{
    const uint8_t command[6] = {
        address, 0xF3U, 0xABU, (uint8_t)enable, 0x00U, MOTOR_FRAME_END
    };

    return Motor_SendRaw(port, command, (uint8_t)sizeof(command));
}

static bool motor_set_velocity(MotorPort port, uint8_t address,
                               uint8_t direction, uint16_t acceleration,
                               float rpm, bool synchronous)
{
    if ((direction > 1U) || (rpm < 0.0f)) {
        return false;
    }

#if (MOTOR_PROTOCOL == MOTOR_PROTOCOL_X)
    {
        uint32_t velocity;
        uint8_t command[9];

        if (rpm > 3000.0f) {
            return false;
        }

        velocity = (uint32_t)((rpm * 10.0f) + 0.5f);

        command[0] = address;
        command[1] = 0xF6U;
        command[2] = direction;
        command[3] = (uint8_t)(acceleration >> 8);
        command[4] = (uint8_t)acceleration;
        command[5] = (uint8_t)(velocity >> 8);
        command[6] = (uint8_t)velocity;
        command[7] = (uint8_t)synchronous;
        command[8] = MOTOR_FRAME_END;

        return Motor_SendRaw(port, command, (uint8_t)sizeof(command));
    }
#else
    {
        uint32_t velocity;
        uint8_t command[8];

        if ((rpm > 5000.0f) || (acceleration > 255U)) {
            return false;
        }

        velocity = (uint32_t)(rpm + 0.5f);

        command[0] = address;
        command[1] = 0xF6U;
        command[2] = direction;
        command[3] = (uint8_t)(velocity >> 8);
        command[4] = (uint8_t)velocity;
        command[5] = (uint8_t)acceleration;
        command[6] = (uint8_t)synchronous;
        command[7] = MOTOR_FRAME_END;

        return Motor_SendRaw(port, command, (uint8_t)sizeof(command));
    }
#endif
}

bool Motor_SetVelocity(MotorPort port, uint8_t address, uint8_t direction,
                       uint16_t acceleration, float rpm)
{
    return motor_set_velocity(port, address, direction, acceleration, rpm,
                              false);
}

bool Motor_QueueVelocity(MotorPort port, uint8_t address, uint8_t direction,
                         uint16_t acceleration, float rpm)
{
    return motor_set_velocity(port, address, direction, acceleration, rpm,
                              true);
}

bool Motor_Stop(MotorPort port, uint8_t address)
{
    const uint8_t command[5] = {
        address, 0xFEU, 0x98U, 0x00U, MOTOR_FRAME_END
    };

    return Motor_SendRaw(port, command, (uint8_t)sizeof(command));
}

bool Motor_QueueStop(MotorPort port, uint8_t address)
{
    const uint8_t command[5] = {
        address, 0xFEU, 0x98U, 0x01U, MOTOR_FRAME_END
    };

    return Motor_SendRaw(port, command, (uint8_t)sizeof(command));
}

bool Motor_TriggerSynchronous(MotorPort port)
{
    const uint8_t command[4] = { 0x00U, 0xFFU, 0x66U, MOTOR_FRAME_END };

    return Motor_SendRaw(port, command, (uint8_t)sizeof(command));
}

static bool motor_move_relative_degrees(MotorPort port, uint8_t address,
                                        uint8_t direction, float rpm,
                                        uint16_t acceleration, float degrees,
                                        bool synchronous)
{
#if (MOTOR_PROTOCOL == MOTOR_PROTOCOL_X)
    uint32_t position;
    uint32_t velocity;
    uint8_t command[16];

    if ((direction > 1U) || (rpm < 0.0f) || (rpm > 3000.0f) ||
        (degrees < 0.0f) || (degrees > 429496704.0f)) {
        return false;
    }

    velocity = (uint32_t)((rpm * 10.0f) + 0.5f);
    position = (uint32_t)((degrees * 10.0f) + 0.5f);

    command[0]  = address;
    command[1]  = 0xFDU;
    command[2]  = direction;
    command[3]  = (uint8_t)(acceleration >> 8);
    command[4]  = (uint8_t)acceleration;
    command[5]  = (uint8_t)(acceleration >> 8);
    command[6]  = (uint8_t)acceleration;
    command[7]  = (uint8_t)(velocity >> 8);
    command[8]  = (uint8_t)velocity;
    command[9]  = (uint8_t)(position >> 24);
    command[10] = (uint8_t)(position >> 16);
    command[11] = (uint8_t)(position >> 8);
    command[12] = (uint8_t)position;
    command[13] = 0x00U;
    command[14] = (uint8_t)synchronous;
    command[15] = MOTOR_FRAME_END;

    return Motor_SendRaw(port, command, (uint8_t)sizeof(command));
#else
    uint32_t pulses;
    uint8_t command[13];

    if ((direction > 1U) || (rpm < 0.0f) || (rpm > 5000.0f) ||
        (acceleration > 255U) || (degrees < 0.0f)) {
        return false;
    }

    pulses = (uint32_t)(((degrees * (float)MOTOR_PULSES_PER_REV) /
                         360.0f) + 0.5f);

    command[0]  = address;
    command[1]  = 0xFDU;
    command[2]  = direction;
    command[3]  = (uint8_t)(((uint16_t)rpm) >> 8);
    command[4]  = (uint8_t)((uint16_t)rpm);
    command[5]  = (uint8_t)acceleration;
    command[6]  = (uint8_t)(pulses >> 24);
    command[7]  = (uint8_t)(pulses >> 16);
    command[8]  = (uint8_t)(pulses >> 8);
    command[9]  = (uint8_t)pulses;
    command[10] = 0x00U;
    command[11] = (uint8_t)synchronous;
    command[12] = MOTOR_FRAME_END;

    return Motor_SendRaw(port, command, (uint8_t)sizeof(command));
#endif
}

bool Motor_MoveRelativeDegrees(MotorPort port, uint8_t address,
                               uint8_t direction, float rpm,
                               uint16_t acceleration, float degrees)
{
    return motor_move_relative_degrees(port, address, direction, rpm,
                                       acceleration, degrees, false);
}

bool Motor_QueueMoveRelativeDegrees(MotorPort port, uint8_t address,
                                    uint8_t direction, float rpm,
                                    uint16_t acceleration, float degrees)
{
    return motor_move_relative_degrees(port, address, direction, rpm,
                                       acceleration, degrees, true);
}

bool Motor_RequestPosition(MotorPort port, uint8_t address)
{
    const uint8_t command[3] = { address, 0x36U, MOTOR_FRAME_END };

    return Motor_SendRaw(port, command, (uint8_t)sizeof(command));
}

bool Motor_TakeResponse(MotorPort port, MotorResponse *response)
{
    uint8_t frame[MOTOR_RX_BUFFER_SIZE];
    uint8_t count;
    uint8_t i;

    if (!port_is_valid(port) || (response == NULL)) {
        return false;
    }

    port_disable_irq(port);
    if (!g_rxReady[port]) {
        port_enable_irq(port);
        return false;
    }

    count = g_rxCount[port];
    for (i = 0U; i < count; i++) {
        frame[i] = g_rxBuffer[port][i];
    }

    g_rxCount[port]    = 0U;
    g_rxExpected[port] = 0U;
    g_rxReady[port]    = false;
    port_enable_irq(port);

    response->type             = MOTOR_RESPONSE_UNKNOWN;
    response->address          = frame[0];
    response->function         = (count > 1U) ? frame[1] : 0U;
    response->status           = (count > 2U) ? frame[2] : 0U;
    response->length           = count;
    response->position_degrees = 0.0f;

    if ((count < 3U) || (frame[count - 1U] != MOTOR_FRAME_END)) {
        return true;
    }

    if ((frame[1] == 0x36U) && (count == 8U)) {
        uint32_t magnitude =
            ((uint32_t)frame[3] << 24) |
            ((uint32_t)frame[4] << 16) |
            ((uint32_t)frame[5] << 8)  |
            (uint32_t)frame[6];

        response->position_degrees = (float)magnitude * 0.1f;
        if (frame[2] != 0U) {
            response->position_degrees = -response->position_degrees;
        }
        response->type = MOTOR_RESPONSE_POSITION;
    } else if (count == 4U) {
        response->type = MOTOR_RESPONSE_ACK;
    }

    return true;
}

static void store_received_byte(MotorPort port, uint8_t data)
{
    if (g_rxReady[port]) {
        return;
    }

    if (g_rxCount[port] >= MOTOR_RX_BUFFER_SIZE) {
        g_rxCount[port]    = 0U;
        g_rxExpected[port] = 0U;
    }

    g_rxBuffer[port][g_rxCount[port]++] = data;

    if (g_rxCount[port] == 2U) {
        g_rxExpected[port] =
            response_length_for_function(g_rxBuffer[port][1]);
    }

    if ((g_rxExpected[port] != 0U) &&
        (g_rxCount[port] >= g_rxExpected[port])) {
        g_rxReady[port] = true;
    }
}

void UART_0_INST_IRQHandler(void)
{
    if (DL_UART_getPendingInterrupt(UART_0_INST) == DL_UART_IIDX_RX) {
        store_received_byte(
            MOTOR_PORT_1,
            (uint8_t)DL_UART_Main_receiveData(UART_0_INST));
    }
}

void UART_1_INST_IRQHandler(void)
{
    if (DL_UART_getPendingInterrupt(UART_1_INST) == DL_UART_IIDX_RX) {
        store_received_byte(
            MOTOR_PORT_2,
            (uint8_t)DL_UART_Main_receiveData(UART_1_INST));
    }
}
