#include "rpi_uart_link.h"
#include "ti_msp_dl_config.h"

#include <stddef.h>

#define RPI_RX_BUFFER_SIZE 192U
#define RPI_TX_TIMEOUT 100000U

static volatile uint8_t g_rxBuffer[RPI_RX_BUFFER_SIZE];
static volatile uint16_t g_rxHead;
static volatile uint16_t g_rxTail;
static volatile bool g_rxOverflow;
static volatile bool g_emergencyStop;

static bool receive_byte(uint8_t *value)
{
    uint16_t tail = g_rxTail;

    if ((value == NULL) || (tail == g_rxHead)) {
        return false;
    }
    *value = g_rxBuffer[tail];
    g_rxTail = (uint16_t)((tail + 1U) % RPI_RX_BUFFER_SIZE);
    return true;
}

static void send_byte(uint8_t value)
{
    uint32_t timeout = RPI_TX_TIMEOUT;

    while (DL_UART_isBusy(UART_RPI_INST)) {
        if (timeout-- == 0U) {
            return;
        }
    }
    DL_UART_Main_transmitData(UART_RPI_INST, value);
}

void RpiLink_Init(void)
{
    uint16_t i;

    g_rxHead = 0U;
    g_rxTail = 0U;
    g_rxOverflow = false;
    g_emergencyStop = false;
    for (i = 0U; i < RPI_RX_BUFFER_SIZE; i++) {
        g_rxBuffer[i] = 0U;
    }

    NVIC_ClearPendingIRQ(UART_RPI_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_RPI_INST_INT_IRQN);
}

bool RpiLink_ReadLine(char *line, uint16_t capacity)
{
    static uint16_t length = 0U;
    static bool discard = false;
    static char pending[RPI_LINK_LINE_SIZE];
    uint8_t byte;

    if ((line == NULL) || (capacity == 0U)) {
        return false;
    }

    while (receive_byte(&byte)) {
        if (byte == '\r') {
            continue;
        }
        if (byte == '\n') {
            if (discard) {
                discard = false;
                length = 0U;
                continue;
            }
            if (length >= capacity) {
                length = 0U;
                return false;
            }
            pending[length] = '\0';
            {
                uint16_t i;
                for (i = 0U; i <= length; i++) {
                    line[i] = pending[i];
                }
            }
            length = 0U;
            return true;
        }
        if (discard) {
            continue;
        }
        if (length < (RPI_LINK_LINE_SIZE - 1U)) {
            pending[length++] = (char)byte;
        } else {
            length = 0U;
            discard = true;
            g_rxOverflow = true;
        }
    }
    return false;
}

void RpiLink_SendString(const char *text)
{
    while ((text != NULL) && (*text != '\0')) {
        send_byte((uint8_t)*text++);
    }
}

void RpiLink_SendUnsigned(uint32_t value)
{
    char digits[10];
    uint8_t length = 0U;

    do {
        digits[length++] = (char)('0' + (value % 10U));
        value /= 10U;
    } while ((value != 0U) && (length < (uint8_t)sizeof(digits)));

    while (length > 0U) {
        send_byte((uint8_t)digits[--length]);
    }
}

void RpiLink_SendSigned(int32_t value)
{
    if (value < 0) {
        send_byte((uint8_t)'-');
        RpiLink_SendUnsigned((uint32_t)(-(value + 1)) + 1U);
    } else {
        RpiLink_SendUnsigned((uint32_t)value);
    }
}

bool RpiLink_TakeEmergencyStop(void)
{
    bool requested = g_emergencyStop;
    g_emergencyStop = false;
    return requested;
}

bool RpiLink_TakeOverflow(void)
{
    bool overflow = g_rxOverflow;
    g_rxOverflow = false;
    return overflow;
}

void UART_RPI_INST_IRQHandler(void)
{
    switch (DL_UART_getPendingInterrupt(UART_RPI_INST)) {
        case DL_UART_IIDX_RX:
        {
            uint8_t byte = DL_UART_Main_receiveData(UART_RPI_INST);
            uint16_t next;

            /* A single '!' is the low-latency stop byte, even during motion. */
            if (byte == (uint8_t)'!') {
                g_emergencyStop = true;
                break;
            }

            next = (uint16_t)((g_rxHead + 1U) % RPI_RX_BUFFER_SIZE);
            if (next == g_rxTail) {
                g_rxOverflow = true;
            } else {
                g_rxBuffer[g_rxHead] = byte;
                g_rxHead = next;
            }
            break;
        }
        default:
            break;
    }
}
