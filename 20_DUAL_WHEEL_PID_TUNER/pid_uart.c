#include "pid_uart.h"
#include "ti_msp_dl_config.h"
#include "third_party/segger_rtt/SEGGER_RTT.h"
#include <stdint.h>

#define PID_UART_RX_BUFFER_SIZE (128U)
#define PID_UART_RX_BUFFER_MASK (PID_UART_RX_BUFFER_SIZE - 1U)

static volatile uint8_t g_rxBuffer[PID_UART_RX_BUFFER_SIZE];
static volatile uint8_t g_rxHead;
static volatile uint8_t g_rxTail;

static bool PID_UART_ReadByte(uint8_t *data)
{
    if (data == 0) {
        return false;
    }

    if (SEGGER_RTT_Read(0U, data, 1U) == 1U) {
        return true;
    }

    if (g_rxHead == g_rxTail) {
        return false;
    }

    *data    = g_rxBuffer[g_rxTail];
    g_rxTail = (uint8_t) ((g_rxTail + 1U) & PID_UART_RX_BUFFER_MASK);
    return true;
}

void PID_UART_Init(void)
{
    g_rxHead = 0U;
    g_rxTail = 0U;
    SEGGER_RTT_Init();
    NVIC_ClearPendingIRQ(DEBUG_UART_INST_INT_IRQN);
    NVIC_EnableIRQ(DEBUG_UART_INST_INT_IRQN);
}

void PID_UART_SendString(const char *text)
{
    const char *uartText = text;

    if (text == 0) {
        return;
    }

    (void) SEGGER_RTT_WriteString(0U, text);

    while (*uartText != '\0') {
        while (DL_UART_isBusy(DEBUG_UART_INST)) {
        }
        DL_UART_Main_transmitData(DEBUG_UART_INST, (uint8_t) *uartText++);
    }
}

bool PID_UART_ReadLine(char *line, size_t size)
{
    static size_t length;
    uint8_t data;

    if ((line == 0) || (size < 2U)) {
        return false;
    }

    while (PID_UART_ReadByte(&data)) {
        if (data == (uint8_t) '\r') {
            continue;
        }
        if (data == (uint8_t) '\n') {
            if (length == 0U) {
                continue;
            }
            line[length] = '\0';
            length       = 0U;
            return true;
        }

        if (length < (size - 1U)) {
            line[length++] = (char) data;
        } else {
            length = 0U;
        }
    }

    return false;
}

void DEBUG_UART_INST_IRQHandler(void)
{
    if (DL_UART_getPendingInterrupt(DEBUG_UART_INST) == DL_UART_IIDX_RX) {
        uint8_t data = DL_UART_Main_receiveData(DEBUG_UART_INST);
        uint8_t next =
            (uint8_t) ((g_rxHead + 1U) & PID_UART_RX_BUFFER_MASK);

        if (next != g_rxTail) {
            g_rxBuffer[g_rxHead] = data;
            g_rxHead             = next;
        }
    }
}
