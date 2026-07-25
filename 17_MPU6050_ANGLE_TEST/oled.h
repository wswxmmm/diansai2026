#ifndef OLED_H
#define OLED_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

#define OLED_WIDTH  128U
#define OLED_HEIGHT 64U

void OLED_Init(void);
void OLED_Clear(void);
void OLED_Refresh(void);
void OLED_ColorTurn(uint8_t mode);
void OLED_DisplayTurn(uint8_t mode);
void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t color);
void OLED_DrawRectangle(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t color);
void OLED_ShowChar(uint8_t x, uint8_t y, char ch, uint8_t size, uint8_t color);
void OLED_ShowString(uint8_t x, uint8_t y, const char *str, uint8_t size, uint8_t color);
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size, uint8_t color);
void OLED_ShowPicture(uint8_t x, uint8_t y, uint8_t width, uint8_t height, const uint8_t *picture, uint8_t color);

#endif
