#include "hc04.h"
#include "ti_msp_dl_config.h"
#include <stdbool.h>
#include <stdint.h>

#define KEY_POLL_MS             (10U)
#define KEY_DEBOUNCE_SAMPLES    (3U)
#define SEND_RETRY_COUNT        (10U)
#define SEND_RETRY_INTERVAL_MS  (100U)
#define LINK_PING_INTERVAL_MS   (1000U)
#define RECEIVE_LINE_SIZE       (32U)

typedef struct {
    uint8_t raw;
    uint8_t stable;
    uint8_t samples;
} KeyState;

typedef struct {
    char data[RECEIVE_LINE_SIZE];
    uint8_t length;
} LineBuffer;

static volatile uint32_t g_milliseconds;
static KeyState g_startKey;
static LineBuffer g_receiveLine;
static uint32_t g_sequence;
static uint32_t g_nextSendMs;
static uint8_t g_retriesRemaining;
static uint32_t g_linkSequence;
static uint32_t g_nextLinkPingMs;

void SysTick_Handler(void)
{
    g_milliseconds++;
}

static bool time_reached(uint32_t now, uint32_t deadline)
{
    return ((int32_t) (now - deadline) >= 0);
}

static uint8_t start_key_is_pressed(void)
{
    return (DL_GPIO_readPins(START_KEY_PORT,
        START_KEY_BUTTON_PIN) == 0U) ? 1U : 0U;
}

static void send_unsigned(uint32_t value)
{
    uint8_t digits[10];
    uint8_t count = 0U;

    do {
        digits[count++] = (uint8_t) ('0' + (value % 10U));
        value /= 10U;
    } while (value > 0U);
    while (count > 0U) {
        HC04_SendByte(digits[--count]);
    }
}

static void send_start_command(void)
{
    HC04_SendString("START 2 ");
    send_unsigned(g_sequence);
    HC04_SendString("\r\n");
}

static void send_link_ping(void)
{
    g_linkSequence++;
    if (g_linkSequence == 0U) {
        g_linkSequence = 1U;
    }
    HC04_SendString("LINK PING ");
    send_unsigned(g_linkSequence);
    HC04_SendString("\r\n");
}

static void schedule_start_command(uint32_t now)
{
    g_sequence++;
    if (g_sequence == 0U) {
        g_sequence = 1U;
    }
    g_retriesRemaining = SEND_RETRY_COUNT;
    g_nextSendMs = now;
}

static void key_init(void)
{
    uint8_t pressed = start_key_is_pressed();

    g_startKey.raw = pressed;
    g_startKey.stable = pressed;
    g_startKey.samples = KEY_DEBOUNCE_SAMPLES;
}

static void key_poll(uint32_t now)
{
    uint8_t pressed = start_key_is_pressed();

    if (pressed == g_startKey.raw) {
        if (g_startKey.samples < KEY_DEBOUNCE_SAMPLES) {
            g_startKey.samples++;
        }
    } else {
        g_startKey.raw = pressed;
        g_startKey.samples = 1U;
    }

    if ((g_startKey.samples >= KEY_DEBOUNCE_SAMPLES) &&
        (g_startKey.stable != g_startKey.raw)) {
        g_startKey.stable = g_startKey.raw;
        if (g_startKey.stable != 0U) {
            schedule_start_command(now);
        }
    }
}

static bool line_starts_with(const char *text, const char *prefix)
{
    while (*prefix != '\0') {
        if (*text++ != *prefix++) {
            return false;
        }
    }
    return true;
}

static bool parse_unsigned(const char *text, uint32_t *value)
{
    uint32_t result = 0U;
    bool has_digit = false;

    while ((*text >= '0') && (*text <= '9')) {
        has_digit = true;
        result = (result * 10U) + (uint32_t) (*text - '0');
        text++;
    }
    if (!has_digit) {
        return false;
    }
    *value = result;
    return true;
}

static bool line_push(uint8_t data)
{
    if (data == (uint8_t) '\r') {
        return false;
    }
    if (data == (uint8_t) '\n') {
        g_receiveLine.data[g_receiveLine.length] = '\0';
        return true;
    }
    if (g_receiveLine.length < (RECEIVE_LINE_SIZE - 1U)) {
        g_receiveLine.data[g_receiveLine.length++] = (char) data;
    } else {
        g_receiveLine.length = 0U;
    }
    return false;
}

static void process_receive_byte(uint8_t data)
{
    static const char prefix[] = "ACK START 2 ";
    uint32_t sequence;

    if (!line_push(data)) {
        return;
    }
    if (line_starts_with(g_receiveLine.data, prefix) &&
        parse_unsigned(&g_receiveLine.data[sizeof(prefix) - 1U],
            &sequence) &&
        (sequence == g_sequence)) {
        g_retriesRemaining = 0U;
    }
    g_receiveLine.length = 0U;
}

static void receive_poll(void)
{
    uint8_t data;

    while (HC04_ReadByte(&data)) {
        process_receive_byte(data);
    }
}

int main(void)
{
    uint32_t last_key_poll;
    bool role_ready;

    SYSCFG_DL_init();
    (void) SysTick_Config(CPUCLK_FREQ / 1000U);
    HC04_Init();
    role_ready = HC04_EnsureRole(HC04_ROLE_SPP_MASTER);
    if (role_ready) {
        (void) HC04_ClearPairing();
    }
    key_init();
    last_key_poll = g_milliseconds;
    g_nextLinkPingMs = g_milliseconds;

    while (1) {
        uint32_t now = g_milliseconds;

        receive_poll();
        if ((now - last_key_poll) >= KEY_POLL_MS) {
            last_key_poll = now;
            key_poll(now);
        }
        if ((g_retriesRemaining > 0U) &&
            time_reached(now, g_nextSendMs)) {
            send_start_command();
            g_retriesRemaining--;
            g_nextSendMs = now + SEND_RETRY_INTERVAL_MS;
        }
        if (time_reached(now, g_nextLinkPingMs)) {
            send_link_ping();
            g_nextLinkPingMs = now + LINK_PING_INTERVAL_MS;
        }
    }
}
