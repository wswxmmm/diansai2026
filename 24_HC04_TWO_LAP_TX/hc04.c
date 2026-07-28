#include "hc04.h"
#include "ti_msp_dl_config.h"

#define HC04_RX_BUFFER_SIZE       (128U)
#define HC04_RX_BUFFER_MASK       (HC04_RX_BUFFER_SIZE - 1U)
#define HC04_CPU_CYCLES_PER_MS    (32000U)

static volatile uint8_t g_rxBuffer[HC04_RX_BUFFER_SIZE];
static volatile uint8_t g_rxHead;
static volatile uint8_t g_rxTail;
static volatile uint32_t g_droppedBytes;

static void HC04_DelayMs(uint32_t milliseconds)
{
    while (milliseconds-- > 0U) {
        DL_Common_delayCycles(HC04_CPU_CYCLES_PER_MS);
    }
}

static void HC04_ClearReceiveBuffer(void)
{
    g_rxTail = g_rxHead;
}

static bool HC04_WaitForText(const char *expected, uint32_t timeoutMs)
{
    uint32_t matched = 0U;
    uint8_t data;

    if ((expected == 0) || (*expected == '\0')) {
        return false;
    }

    while (timeoutMs-- > 0U) {
        while (HC04_ReadByte(&data)) {
            if (data == (uint8_t) expected[matched]) {
                matched++;
                if (expected[matched] == '\0') {
                    return true;
                }
            } else {
                matched = (data == (uint8_t) expected[0]) ? 1U : 0U;
            }
        }
        HC04_DelayMs(1U);
    }

    return false;
}

static bool HC04_SendCommand(const char *command,
    const char *expected, uint32_t timeoutMs)
{
    HC04_ClearReceiveBuffer();
    HC04_SendString(command);
    return HC04_WaitForText(expected, timeoutMs);
}

void HC04_Init(void)
{
    g_rxHead = 0U;
    g_rxTail = 0U;
    g_droppedBytes = 0U;
    NVIC_ClearPendingIRQ(HC04_UART_INST_INT_IRQN);
    NVIC_EnableIRQ(HC04_UART_INST_INT_IRQN);
}

bool HC04_EnsureRole(HC04_Role role)
{
    const char *roleCommand;
    const char *roleResponse;

    if (role == HC04_ROLE_SPP_MASTER) {
        roleCommand = "AT+ROLE=M";
        roleResponse = "SppMaster";
    } else {
        roleCommand = "AT+ROLE=S";
        roleResponse = "Slave";
    }

    HC04_DelayMs(400U);
    if (!HC04_SendCommand("AT", "OK", 300U)) {
        return false;
    }
    if (HC04_SendCommand("AT+ROLE=?", roleResponse, 300U)) {
        return true;
    }

    (void) HC04_SendCommand(roleCommand, roleResponse, 500U);
    HC04_DelayMs(600U);
    if (!HC04_SendCommand("AT", "OK", 300U)) {
        return false;
    }
    return HC04_SendCommand("AT+ROLE=?", roleResponse, 300U);
}

bool HC04_ClearPairing(void)
{
    bool cleared = HC04_SendCommand("AT+CLEAR", "OK", 500U);

    if (cleared) {
        HC04_DelayMs(800U);
    }
    return cleared;
}

void HC04_SendByte(uint8_t data)
{
    while (DL_UART_isBusy(HC04_UART_INST)) {
    }
    DL_UART_Main_transmitData(HC04_UART_INST, data);
}

void HC04_SendString(const char *text)
{
    while ((text != 0) && (*text != '\0')) {
        HC04_SendByte((uint8_t) *text++);
    }
}

bool HC04_ReadByte(uint8_t *data)
{
    if ((data == 0) || (g_rxHead == g_rxTail)) {
        return false;
    }
    *data = g_rxBuffer[g_rxTail];
    g_rxTail = (uint8_t) ((g_rxTail + 1U) & HC04_RX_BUFFER_MASK);
    return true;
}

uint32_t HC04_GetDroppedCount(void)
{
    return g_droppedBytes;
}

void HC04_UART_INST_IRQHandler(void)
{
    uint8_t next;
    uint8_t data;

    if (DL_UART_getPendingInterrupt(HC04_UART_INST) != DL_UART_IIDX_RX) {
        return;
    }

    data = DL_UART_Main_receiveData(HC04_UART_INST);
    next = (uint8_t) ((g_rxHead + 1U) & HC04_RX_BUFFER_MASK);
    if (next == g_rxTail) {
        g_droppedBytes++;
    } else {
        g_rxBuffer[g_rxHead] = data;
        g_rxHead = next;
    }
}
