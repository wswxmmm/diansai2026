#ifndef MPU_PORT_H
#define MPU_PORT_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

int MPU_Write_Len(unsigned char addr, unsigned char reg, unsigned char len, unsigned char *buf);
int MPU_Read_Len(unsigned char addr, unsigned char reg, unsigned char len, unsigned char *buf);
void mget_ms(unsigned long *time);

int DMP_Init(void);
int DMP_Read_Data(float *pitch, float *roll, float *yaw);

#endif
