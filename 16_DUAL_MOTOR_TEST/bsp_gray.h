#ifndef BSP_GRAY_H
#define BSP_GRAY_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

#define GRAY_SENSOR_CHANNELS   (8U)
#define GRAY_ADC_MAX_VALUE     (4095U)

void GraySensor_Read(uint16_t values[GRAY_SENSOR_CHANNELS]);

#endif
