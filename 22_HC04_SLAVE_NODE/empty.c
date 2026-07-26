#include "hc04.h"
#include "oled.h"
#include "ti_msp_dl_config.h"
#include <stdbool.h>
#include <stdint.h>

#define LINE_BUFFER_SIZE       (64U)
#define CPU_CYCLES_PER_MS      (32000U)
#define SCREEN_INTERVAL_MS     (200U)
#define LINK_TIMEOUT_MS        (3000U)

typedef struct {
    char data[LINE_BUFFER_SIZE];
    uint8_t length;
} LineBuffer;

static LineBuffer g_receiveLine;
static uint32_t g_systemMs;
static uint32_t g_receiveCount;
static uint32_t g_replyCount;
static uint32_t g_lastPingSequence;
static uint32_t g_lastReceiveMs;
static bool g_oledReady;
static bool g_roleReady;

static void Debug_SendByte(uint8_t data)
{
    while (DL_UART_isBusy(DEBUG_UART_INST)) {
    }
    DL_UART_Main_transmitData(DEBUG_UART_INST, data);
}

static void Debug_SendString(const char *text)
{
    while ((text != 0) && (*text != '\0')) {
        Debug_SendByte((uint8_t) *text++);
    }
}

static void SendUnsigned(void (*sendByte)(uint8_t), uint32_t value)
{
    uint8_t digits[10];
    uint8_t count = 0U;

    do {
        digits[count++] = (uint8_t) ('0' + (value % 10U));
        value /= 10U;
    } while (value > 0U);

    while (count > 0U) {
        sendByte(digits[--count]);
    }
}

static bool LineStartsWith(const char *text, const char *prefix)
{
    while (*prefix != '\0') {
        if (*text++ != *prefix++) {
            return false;
        }
    }
    return true;
}

static bool ParseUnsigned(const char *text, uint32_t *value)
{
    uint32_t result = 0U;
    bool hasDigit = false;

    while ((*text >= '0') && (*text <= '9')) {
        hasDigit = true;
        result = (result * 10U) + (uint32_t) (*text - '0');
        text++;
    }
    if (!hasDigit) {
        return false;
    }
    *value = result;
    return true;
}

static bool LineBuffer_Push(LineBuffer *line, uint8_t data)
{
    if (data == (uint8_t) '\r') {
        return false;
    }
    if (data == (uint8_t) '\n') {
        line->data[line->length] = '\0';
        return true;
    }
    if (line->length < (LINE_BUFFER_SIZE - 1U)) {
        line->data[line->length++] = (char) data;
    } else {
        line->length = 0U;
    }
    return false;
}

static void Screen_ShowConfig(void)
{
    if (!g_oledReady) {
        return;
    }
    OLED_Clear();
    OLED_ShowString(20, 3, "HC04 SLAVE", 8, 1);
    OLED_ShowString(28, 22, "CONFIG", 16, 1);
    OLED_ShowString(27, 48, "ROLE: SLAVE", 8, 1);
    OLED_Refresh();
}

static void Screen_Update(void)
{
    bool linkOk = (g_receiveCount > 0U) &&
        ((g_systemMs - g_lastReceiveMs) <= LINK_TIMEOUT_MS);

    if (!g_oledReady) {
        return;
    }
    OLED_Clear();
    OLED_ShowString(20, 1, "HC04 SLAVE", 8, 1);
    if (linkOk) {
        OLED_ShowString(20, 11, "LINK OK", 16, 1);
    } else if (g_receiveCount == 0U) {
        OLED_ShowString(20, 11, "WAITING", 16, 1);
    } else {
        OLED_ShowString(20, 11, "TIMEOUT", 16, 1);
    }

    OLED_ShowString(2, 31, "RX:", 8, 1);
    OLED_ShowNum(20, 31, g_receiveCount % 100000U, 5, 8, 1);
    OLED_ShowString(62, 31, "TX:", 8, 1);
    OLED_ShowNum(80, 31, g_replyCount % 100000U, 5, 8, 1);
    OLED_ShowString(2, 42, "LAST PING:", 8, 1);
    OLED_ShowNum(62, 42, g_lastPingSequence % 100000U, 5, 8, 1);
    OLED_ShowString(2, 53,
        g_roleReady ? "ROLE:S U2 9600" : "ROLE:S AT SKIP", 8, 1);
    OLED_Refresh();
}

static void Slave_SendAck(uint32_t sequence)
{
    HC04_SendString("SLAVE ACK ");
    SendUnsigned(HC04_SendByte, sequence);
    HC04_SendString("\r\n");
    g_replyCount++;

    Debug_SendString("[TX] SLAVE ACK ");
    SendUnsigned(Debug_SendByte, sequence);
    Debug_SendString("\r\n");
}

static void ProcessReceive(uint8_t data)
{
    static const char prefix[] = "MASTER PING ";

    if (!LineBuffer_Push(&g_receiveLine, data)) {
        return;
    }

    Debug_SendString("[RX] ");
    Debug_SendString(g_receiveLine.data);
    Debug_SendString("\r\n");
    if (LineStartsWith(g_receiveLine.data, prefix) &&
        ParseUnsigned(&g_receiveLine.data[sizeof(prefix) - 1U],
            &g_lastPingSequence)) {
        g_receiveCount++;
        g_lastReceiveMs = g_systemMs;
        Slave_SendAck(g_lastPingSequence);
    }
    g_receiveLine.length = 0U;
}

int main(void)
{
    uint32_t screenElapsedMs = 0U;
    uint8_t data;

    SYSCFG_DL_init();
    HC04_Init();
    g_oledReady = OLED_Init();
    if (g_oledReady) {
        OLED_ColorTurn(0U);
        OLED_DisplayTurn(0U);
    }

    Debug_SendString("HC-04 slave node boot\r\n");
    Screen_ShowConfig();
    g_roleReady = HC04_EnsureRole(HC04_ROLE_SLAVE);
    Debug_SendString(g_roleReady ?
        "SLAVE ROLE: ready\r\n" :
        "SLAVE ROLE: AT unavailable, continuing\r\n");
    Screen_Update();

    while (1) {
        while (HC04_ReadByte(&data)) {
            ProcessReceive(data);
        }

        DL_Common_delayCycles(CPU_CYCLES_PER_MS);
        g_systemMs++;
        screenElapsedMs++;
        if (screenElapsedMs >= SCREEN_INTERVAL_MS) {
            screenElapsedMs = 0U;
            Screen_Update();
        }
    }
}
