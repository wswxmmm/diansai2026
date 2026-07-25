#ifndef PID_UART_H
#define PID_UART_H

#include <stdbool.h>
#include <stddef.h>

void PID_UART_Init(void);
void PID_UART_SendString(const char *text);
bool PID_UART_ReadLine(char *line, size_t size);

#endif
