#ifndef BSP_W25Q128_H
#define BSP_W25Q128_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

#define SPI_CS(x) ((x) ? DL_GPIO_setPins(CS_PORT, CS_PIN_PIN) : DL_GPIO_clearPins(CS_PORT, CS_PIN_PIN))

uint16_t W25Q128_readID(void);
void W25Q128_write(uint8_t *buffer, uint32_t addr, uint16_t numbyte);
void W25Q128_read(uint8_t *buffer, uint32_t read_addr, uint16_t read_length);

#endif
