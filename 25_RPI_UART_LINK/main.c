#include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define UART_RX_BUFFER_SIZE 128U
#define UART_LINE_SIZE       96U

static volatile uint8_t g_rx_buffer[UART_RX_BUFFER_SIZE];
static volatile uint16_t g_rx_head = 0U;
static volatile uint16_t g_rx_tail = 0U;
static volatile bool g_rx_overflow = false;

static void uart_send_char(char ch)
{
    while (DL_UART_isBusy(UART_0_INST) == true) {
    }
    DL_UART_Main_transmitData(UART_0_INST, (uint8_t) ch);
}

static void uart_send_string(const char *text)
{
    while ((text != NULL) && (*text != '\0')) {
        uart_send_char(*text++);
    }
}

static bool uart_receive_char(uint8_t *value)
{
    uint16_t tail = g_rx_tail;

    if (tail == g_rx_head) {
        return false;
    }

    *value = g_rx_buffer[tail];
    g_rx_tail = (uint16_t) ((tail + 1U) % UART_RX_BUFFER_SIZE);
    return true;
}

static void process_command(char *line)
{
    if (strcmp(line, "PING") == 0) {
        uart_send_string("PONG\r\n");
    } else if (strncmp(line, "PING ", 5U) == 0) {
        uart_send_string("PONG ");
        uart_send_string(line + 5);
        uart_send_string("\r\n");
    } else if (strncmp(line, "ECHO ", 5U) == 0) {
        uart_send_string("ECHO ");
        uart_send_string(line + 5);
        uart_send_string("\r\n");
    } else if (strcmp(line, "STATUS") == 0) {
        uart_send_string("STATUS OK MSPM0G3507 UART3 115200 FW5\r\n");
    } else if (strcmp(line, "HELP") == 0) {
        uart_send_string("COMMANDS PING [token] | ECHO text | STATUS | HELP\r\n");
    } else if (line[0] != '\0') {
        uart_send_string("ERR UNKNOWN_COMMAND\r\n");
    }
}

int main(void)
{
    char line[UART_LINE_SIZE];
    uint16_t line_length = 0U;
    uint16_t heartbeat_ticks = 0U;
    bool discard_line = false;
    uint8_t byte;

    SYSCFG_DL_init();

    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);

    uart_send_string("READY MSPM0G3507 UART3 115200 FW5\r\n");

    while (1) {
        if (g_rx_overflow) {
            g_rx_overflow = false;
            line_length = 0U;
            discard_line = true;
            uart_send_string("ERR RX_OVERFLOW\r\n");
        }

        while (uart_receive_char(&byte)) {
            if (byte == '\r') {
                continue;
            }

            if (byte == '\n') {
                if (!discard_line) {
                    line[line_length] = '\0';
                    process_command(line);
                }
                line_length = 0U;
                discard_line = false;
                continue;
            }

            if (discard_line) {
                continue;
            }

            if (line_length < (UART_LINE_SIZE - 1U)) {
                line[line_length++] = (char) byte;
            } else {
                line_length = 0U;
                discard_line = true;
                uart_send_string("ERR LINE_TOO_LONG\r\n");
            }
        }

        /* A periodic beacon makes the physical TX path easy to diagnose. */
        DL_Common_delayCycles(32000U);
        heartbeat_ticks++;
        if ((heartbeat_ticks == 500U) || (heartbeat_ticks == 1000U)) {
            DL_GPIO_togglePins(LED_PORT, LED_LED0_PIN);
        }
        if (heartbeat_ticks >= 1000U) {
            heartbeat_ticks = 0U;
            uart_send_string("HEARTBEAT MSPM0G3507\r\n");
        }
    }
}

void UART_0_INST_IRQHandler(void)
{
    switch (DL_UART_getPendingInterrupt(UART_0_INST)) {
        case DL_UART_IIDX_RX:
        {
            uint8_t byte = DL_UART_Main_receiveData(UART_0_INST);
            uint16_t next = (uint16_t) ((g_rx_head + 1U) % UART_RX_BUFFER_SIZE);

            if (next == g_rx_tail) {
                g_rx_overflow = true;
            } else {
                g_rx_buffer[g_rx_head] = byte;
                g_rx_head = next;
            }
            break;
        }
        default:
            break;
    }
}
