#ifndef BSP_TB6612_H
#define BSP_TB6612_H

#include "ti_msp_dl_config.h"

#define TB6612_MAX_SPEED (999U)

#define AIN1_OUT(x) ((x) ? DL_GPIO_setPins(TB6612_PORT, TB6612_AIN1_PIN) : DL_GPIO_clearPins(TB6612_PORT, TB6612_AIN1_PIN))
#define AIN2_OUT(x) ((x) ? DL_GPIO_setPins(TB6612_PORT, TB6612_AIN2_PIN) : DL_GPIO_clearPins(TB6612_PORT, TB6612_AIN2_PIN))
#define BIN1_OUT(x) ((x) ? DL_GPIO_setPins(TB6612_PORT, TB6612_BIN1_PIN) : DL_GPIO_clearPins(TB6612_PORT, TB6612_BIN1_PIN))
#define BIN2_OUT(x) ((x) ? DL_GPIO_setPins(TB6612_PORT, TB6612_BIN2_PIN) : DL_GPIO_clearPins(TB6612_PORT, TB6612_BIN2_PIN))

void TB6612_Motor_Stop(void);
void AO_Control(uint8_t dir, uint32_t speed);
void BO_Control(uint8_t dir, uint32_t speed);

#endif