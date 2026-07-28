#ifndef BSP_MPU6050_SW_H
#define BSP_MPU6050_SW_H

#include "ti_msp_dl_config.h"
#include <stdbool.h>
#include <stdint.h>

bool MPU6050_SW_Init(void);
bool MPU6050_SW_CalibrateGyroZ(void);
bool MPU6050_SW_ReadGyroZDpsMilli(int32_t *gyro_z_mdps);
uint8_t MPU6050_SW_GetAddress(void);
uint8_t MPU6050_SW_GetWhoAmI(void);

#endif
