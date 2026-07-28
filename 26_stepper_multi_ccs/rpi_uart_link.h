#ifndef RPI_UART_LINK_H
#define RPI_UART_LINK_H

#include <stdbool.h>
#include <stdint.h>

#define RPI_LINK_LINE_SIZE 96U

void RpiLink_Init(void);
bool RpiLink_ReadLine(char *line, uint16_t capacity);
void RpiLink_SendString(const char *text);
void RpiLink_SendUnsigned(uint32_t value);
void RpiLink_SendSigned(int32_t value);
bool RpiLink_TakeEmergencyStop(void);
bool RpiLink_TakeOverflow(void);

#endif
