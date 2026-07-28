#ifndef HC04_H
#define HC04_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    HC04_ROLE_SLAVE,
    HC04_ROLE_SPP_MASTER
} HC04_Role;

void HC04_Init(void);
bool HC04_EnsureRole(HC04_Role role);
bool HC04_ClearPairing(void);
void HC04_SendByte(uint8_t data);
void HC04_SendString(const char *text);
bool HC04_ReadByte(uint8_t *data);
uint32_t HC04_GetDroppedCount(void);

#endif
