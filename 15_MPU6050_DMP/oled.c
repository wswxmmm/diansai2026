#include "oled.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#define OLED_I2C_ADDR     (0x3CU)
#define OLED_CMD          (0x00U)
#define OLED_DATA         (0x40U)
#define OLED_PAGE_COUNT   (OLED_HEIGHT / 8U)
#define OLED_CHUNK_BYTES  (7U)
#define OLED_SCALE(size)  (((size) >= 16U) ? 2U : 1U)

static uint8_t g_oledBuffer[OLED_WIDTH * OLED_PAGE_COUNT];

static bool OLED_I2C_Write(const uint8_t *data, uint8_t len)
{
    while (!(DL_I2C_getControllerStatus(I2C_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
    }

    (void) DL_I2C_fillControllerTXFIFO(I2C_INST, (uint8_t *) data, len);
    DL_I2C_startControllerTransfer(
        I2C_INST, OLED_I2C_ADDR, DL_I2C_CONTROLLER_DIRECTION_TX, len);
    delay_cycles(1000);

    while (DL_I2C_getControllerStatus(I2C_INST) & DL_I2C_CONTROLLER_STATUS_BUSY) {
    }

    return (DL_I2C_getControllerStatus(I2C_INST) & DL_I2C_CONTROLLER_STATUS_ERROR) == 0U;
}

static void OLED_WR_Byte(uint8_t data, uint8_t mode)
{
    uint8_t packet[2] = {mode, data};
    (void) OLED_I2C_Write(packet, 2);
}

static void OLED_WriteDataChunk(const uint8_t *data, uint8_t len)
{
    uint8_t packet[OLED_CHUNK_BYTES + 1U];

    packet[0] = OLED_DATA;
    memcpy(&packet[1], data, len);
    (void) OLED_I2C_Write(packet, (uint8_t) (len + 1U));
}

static const uint8_t *OLED_GetGlyph(char ch)
{
    static const uint8_t blank[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
    static const uint8_t dash[5]  = {0x08, 0x08, 0x08, 0x08, 0x08};
    static const uint8_t colon[5] = {0x00, 0x36, 0x36, 0x00, 0x00};
    static const uint8_t slash[5] = {0x20, 0x10, 0x08, 0x04, 0x02};
    static const uint8_t dot[5]   = {0x00, 0x60, 0x60, 0x00, 0x00};
    static const uint8_t digits[10][5] = {
        {0x3E, 0x51, 0x49, 0x45, 0x3E},
        {0x00, 0x42, 0x7F, 0x40, 0x00},
        {0x42, 0x61, 0x51, 0x49, 0x46},
        {0x21, 0x41, 0x45, 0x4B, 0x31},
        {0x18, 0x14, 0x12, 0x7F, 0x10},
        {0x27, 0x45, 0x45, 0x45, 0x39},
        {0x3C, 0x4A, 0x49, 0x49, 0x30},
        {0x01, 0x71, 0x09, 0x05, 0x03},
        {0x36, 0x49, 0x49, 0x49, 0x36},
        {0x06, 0x49, 0x49, 0x29, 0x1E},
    };
    static const uint8_t upper[26][5] = {
        {0x7E, 0x11, 0x11, 0x11, 0x7E},
        {0x7F, 0x49, 0x49, 0x49, 0x36},
        {0x3E, 0x41, 0x41, 0x41, 0x22},
        {0x7F, 0x41, 0x41, 0x22, 0x1C},
        {0x7F, 0x49, 0x49, 0x49, 0x41},
        {0x7F, 0x09, 0x09, 0x09, 0x01},
        {0x3E, 0x41, 0x49, 0x49, 0x7A},
        {0x7F, 0x08, 0x08, 0x08, 0x7F},
        {0x00, 0x41, 0x7F, 0x41, 0x00},
        {0x20, 0x40, 0x41, 0x3F, 0x01},
        {0x7F, 0x08, 0x14, 0x22, 0x41},
        {0x7F, 0x40, 0x40, 0x40, 0x40},
        {0x7F, 0x02, 0x0C, 0x02, 0x7F},
        {0x7F, 0x04, 0x08, 0x10, 0x7F},
        {0x3E, 0x41, 0x41, 0x41, 0x3E},
        {0x7F, 0x09, 0x09, 0x09, 0x06},
        {0x3E, 0x41, 0x51, 0x21, 0x5E},
        {0x7F, 0x09, 0x19, 0x29, 0x46},
        {0x46, 0x49, 0x49, 0x49, 0x31},
        {0x01, 0x01, 0x7F, 0x01, 0x01},
        {0x3F, 0x40, 0x40, 0x40, 0x3F},
        {0x1F, 0x20, 0x40, 0x20, 0x1F},
        {0x3F, 0x40, 0x38, 0x40, 0x3F},
        {0x63, 0x14, 0x08, 0x14, 0x63},
        {0x07, 0x08, 0x70, 0x08, 0x07},
        {0x61, 0x51, 0x49, 0x45, 0x43},
    };

    if ((ch >= 'a') && (ch <= 'z')) {
        ch = (char) (ch - 'a' + 'A');
    }
    if ((ch >= '0') && (ch <= '9')) {
        return digits[ch - '0'];
    }
    if ((ch >= 'A') && (ch <= 'Z')) {
        return upper[ch - 'A'];
    }
    if (ch == '-') {
        return dash;
    }
    if (ch == ':') {
        return colon;
    }
    if (ch == '/') {
        return slash;
    }
    if (ch == '.') {
        return dot;
    }

    return blank;
}

void OLED_Init(void)
{
    delay_cycles(CPUCLK_FREQ / 10U);

    OLED_WR_Byte(0xAE, OLED_CMD);
    OLED_WR_Byte(0x00, OLED_CMD);
    OLED_WR_Byte(0x10, OLED_CMD);
    OLED_WR_Byte(0x40, OLED_CMD);
    OLED_WR_Byte(0x81, OLED_CMD);
    OLED_WR_Byte(0xCF, OLED_CMD);
    OLED_WR_Byte(0xA1, OLED_CMD);
    OLED_WR_Byte(0xC8, OLED_CMD);
    OLED_WR_Byte(0xA6, OLED_CMD);
    OLED_WR_Byte(0xA8, OLED_CMD);
    OLED_WR_Byte(0x3F, OLED_CMD);
    OLED_WR_Byte(0xD3, OLED_CMD);
    OLED_WR_Byte(0x00, OLED_CMD);
    OLED_WR_Byte(0xD5, OLED_CMD);
    OLED_WR_Byte(0x80, OLED_CMD);
    OLED_WR_Byte(0xD9, OLED_CMD);
    OLED_WR_Byte(0xF1, OLED_CMD);
    OLED_WR_Byte(0xDA, OLED_CMD);
    OLED_WR_Byte(0x12, OLED_CMD);
    OLED_WR_Byte(0xDB, OLED_CMD);
    OLED_WR_Byte(0x40, OLED_CMD);
    OLED_WR_Byte(0x20, OLED_CMD);
    OLED_WR_Byte(0x02, OLED_CMD);
    OLED_WR_Byte(0x8D, OLED_CMD);
    OLED_WR_Byte(0x14, OLED_CMD);
    OLED_WR_Byte(0xA4, OLED_CMD);
    OLED_WR_Byte(0xA6, OLED_CMD);
    OLED_WR_Byte(0xAF, OLED_CMD);

    OLED_Clear();
    OLED_Refresh();
}

void OLED_Clear(void)
{
    memset(g_oledBuffer, 0, sizeof(g_oledBuffer));
}

void OLED_Refresh(void)
{
    uint8_t page;
    uint8_t col;

    for (page = 0; page < OLED_PAGE_COUNT; page++) {
        OLED_WR_Byte((uint8_t) (0xB0U + page), OLED_CMD);
        OLED_WR_Byte(0x00, OLED_CMD);
        OLED_WR_Byte(0x10, OLED_CMD);

        for (col = 0; col < OLED_WIDTH; col += OLED_CHUNK_BYTES) {
            uint8_t len = (uint8_t) ((OLED_WIDTH - col) >= OLED_CHUNK_BYTES ?
                             OLED_CHUNK_BYTES :
                             (OLED_WIDTH - col));
            OLED_WriteDataChunk(&g_oledBuffer[(page * OLED_WIDTH) + col], len);
        }
    }
}

void OLED_ColorTurn(uint8_t mode)
{
    OLED_WR_Byte(mode ? 0xA7U : 0xA6U, OLED_CMD);
}

void OLED_DisplayTurn(uint8_t mode)
{
    if (mode) {
        OLED_WR_Byte(0xC0, OLED_CMD);
        OLED_WR_Byte(0xA0, OLED_CMD);
    } else {
        OLED_WR_Byte(0xC8, OLED_CMD);
        OLED_WR_Byte(0xA1, OLED_CMD);
    }
}

void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t color)
{
    uint16_t index;
    uint8_t bit;

    if ((x >= OLED_WIDTH) || (y >= OLED_HEIGHT)) {
        return;
    }

    index = (uint16_t) x + ((uint16_t) (y / 8U) * OLED_WIDTH);
    bit = (uint8_t) (1U << (y % 8U));

    if (color) {
        g_oledBuffer[index] |= bit;
    } else {
        g_oledBuffer[index] &= (uint8_t) ~bit;
    }
}

void OLED_DrawRectangle(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t color)
{
    uint8_t x;
    uint8_t y;

    for (x = x0; x <= x1; x++) {
        OLED_DrawPoint(x, y0, color);
        OLED_DrawPoint(x, y1, color);
    }
    for (y = y0; y <= y1; y++) {
        OLED_DrawPoint(x0, y, color);
        OLED_DrawPoint(x1, y, color);
    }
}

void OLED_ShowChar(uint8_t x, uint8_t y, char ch, uint8_t size, uint8_t color)
{
    const uint8_t *glyph = OLED_GetGlyph(ch);
    uint8_t scale = OLED_SCALE(size);
    uint8_t col;
    uint8_t row;
    uint8_t sx;
    uint8_t sy;

    for (col = 0; col < 5U; col++) {
        for (row = 0; row < 7U; row++) {
            if ((glyph[col] & (1U << row)) != 0U) {
                for (sx = 0; sx < scale; sx++) {
                    for (sy = 0; sy < scale; sy++) {
                        OLED_DrawPoint((uint8_t) (x + (col * scale) + sx),
                            (uint8_t) (y + (row * scale) + sy), color);
                    }
                }
            }
        }
    }
}

void OLED_ShowString(uint8_t x, uint8_t y, const char *str, uint8_t size, uint8_t color)
{
    uint8_t scale = OLED_SCALE(size);

    while ((str != NULL) && (*str != '\0')) {
        OLED_ShowChar(x, y, *str++, size, color);
        x = (uint8_t) (x + (6U * scale));
        if (x > (OLED_WIDTH - (6U * scale))) {
            x = 0;
            y = (uint8_t) (y + (8U * scale));
        }
        if (y >= OLED_HEIGHT) {
            return;
        }
    }
}

void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size, uint8_t color)
{
    char text[12];
    (void) snprintf(text, sizeof(text), "%0*lu", len, (unsigned long) num);
    OLED_ShowString(x, y, text, size, color);
}

void OLED_ShowPicture(uint8_t x, uint8_t y, uint8_t width, uint8_t height, const uint8_t *picture, uint8_t color)
{
    uint8_t page;
    uint8_t col;

    if (picture == NULL) {
        return;
    }

    for (page = 0; page < (height / 8U); page++) {
        for (col = 0; col < width; col++) {
            uint8_t value = picture[(page * width) + col];
            uint8_t bit;
            for (bit = 0; bit < 8U; bit++) {
                if ((value & (1U << bit)) != 0U) {
                    OLED_DrawPoint((uint8_t) (x + col), (uint8_t) (y + (page * 8U) + bit), color);
                }
            }
        }
    }
}
