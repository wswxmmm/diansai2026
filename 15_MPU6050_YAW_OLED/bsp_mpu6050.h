#ifndef BSP_MPU6050_H
#define BSP_MPU6050_H

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>

#define MPU6050_ADDR                 (0x68U)
#define MPU6050_SAMPLE_MS            (20U)
#define MPU6050_CALIBRATION_SAMPLES  (500U)

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} MPU6050RawData;

bool MPU6050_Init(void);
uint8_t MPU6050_GetAddress(void);
uint8_t MPU6050_GetWhoAmI(void);
uint8_t MPU6050_GetInitStep(void);
bool MPU6050_ReadRaw(MPU6050RawData *data);
void MPU6050_CalibrateGyroZ(void);
void MPU6050_ResetYaw(void);
int32_t MPU6050_UpdateYawMilliDeg(void);
int32_t MPU6050_GetYawMilliDeg(void);

#endif