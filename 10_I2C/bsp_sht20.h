#ifndef BSP_SHT20_H
#define BSP_SHT20_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

#define SHT20_CMD_TEMP_NO_HOLD     (0xF3U)
#define SHT20_CMD_HUMIDITY_NO_HOLD (0xF5U)

#define SDA_OUT() do { \
    DL_GPIO_initDigitalOutput(I2C_SDA_IOMUX); \
    DL_GPIO_setPins(I2C_PORT, I2C_SDA_PIN); \
    DL_GPIO_enableOutput(I2C_PORT, I2C_SDA_PIN); \
} while (0)

#define SDA_IN() do { \
    DL_GPIO_initDigitalInput(I2C_SDA_IOMUX); \
} while (0)

#define SDA_GET() (((DL_GPIO_readPins(I2C_PORT, I2C_SDA_PIN) & I2C_SDA_PIN) != 0U) ? 1U : 0U)
#define SDA(x)    ((x) ? DL_GPIO_setPins(I2C_PORT, I2C_SDA_PIN) : DL_GPIO_clearPins(I2C_PORT, I2C_SDA_PIN))
#define SCL(x)    ((x) ? DL_GPIO_setPins(I2C_PORT, I2C_SCL_PIN) : DL_GPIO_clearPins(I2C_PORT, I2C_SCL_PIN))

float SHT20_Read(unsigned char command);

#endif
