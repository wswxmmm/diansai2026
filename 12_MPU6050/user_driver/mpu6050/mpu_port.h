#ifndef _MPU_PORT_H_
#define _MPU_PORT_H_

#include "ti_msp_dl_config.h"
#include <stdint.h>

// 接线
// MPU6050_SCL 连接到 MSPM0 的 PA1
// MPU6050_SDA 连接到 MSPM0 的 PA0
// VCC 连接到 3.3V
// GND 连接到 GND
// ADDR 连接到 GND（MPU6050 的 I2C 地址为 0x68）二选一（默认）
// ADDR 连接到 VCC（MPU6050 的 I2C 地址为 0x69）二选一

// 官方库需要的 4 个底层函数声明
int MPU_Write_Len(unsigned char addr, unsigned char reg, unsigned char len, unsigned char *buf);
int MPU_Read_Len(unsigned char addr, unsigned char reg, unsigned char len, unsigned char *buf);
void mget_ms(unsigned long *time);

// 我们自己封装的 DMP 初始化和读取函数
int DMP_Init(void);
int DMP_Read_Data(float *pitch, float *roll, float *yaw);

#endif

