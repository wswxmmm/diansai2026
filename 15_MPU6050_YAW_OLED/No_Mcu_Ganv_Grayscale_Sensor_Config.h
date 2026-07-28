#ifndef NO_MCU_GANV_GRAYSCALE_SENSOR_CONFIG_H
#define NO_MCU_GANV_GRAYSCALE_SENSOR_CONFIG_H

#include <string.h>

#include "ADC.h"
#include "delay.h"
#include "ti_msp_dl_config.h"

#define Class 0

#define _14Bits 0
#define _12Bits 1
#define _10Bits 2
#define _8Bits  3

#define Sensor_Edition Class
#define Direction 1
#define Sensor_ADCbits _12Bits

#define Switch_Address_0(i) \
    ((i) ? (DL_GPIO_setPins(Gray_Address_PORT, Gray_Address_PIN_0_PIN)) : \
           (DL_GPIO_clearPins(Gray_Address_PORT, Gray_Address_PIN_0_PIN)))
#define Switch_Address_1(i) \
    ((i) ? (DL_GPIO_setPins(Gray_Address_PORT, Gray_Address_PIN_1_PIN)) : \
           (DL_GPIO_clearPins(Gray_Address_PORT, Gray_Address_PIN_1_PIN)))
#define Switch_Address_2(i) \
    ((i) ? (DL_GPIO_setPins(Gray_Address_PORT, Gray_Address_PIN_2_PIN)) : \
           (DL_GPIO_clearPins(Gray_Address_PORT, Gray_Address_PIN_2_PIN)))

#define Get_adc_of_user() adc_getValue()

typedef struct {
    unsigned short Analog_value[8];
    unsigned short Normal_value[8];
    unsigned short Calibrated_white[8];
    unsigned short Calibrated_black[8];
    unsigned short Gray_white[8];
    unsigned short Gray_black[8];
    double Normal_factor[8];
    double bits;
    unsigned char Digtal;
    unsigned char Time_out;
    unsigned char Tick;
    unsigned char ok;
} No_MCU_Sensor;

#ifdef __cplusplus
extern "C" {
#endif

void No_MCU_Ganv_Sensor_Init_Frist(No_MCU_Sensor *sensor);
void No_MCU_Ganv_Sensor_Init(No_MCU_Sensor *sensor,
    unsigned short *Calibrated_white, unsigned short *Calibrated_black);
void No_Mcu_Ganv_Sensor_Task_Without_tick(No_MCU_Sensor *sensor);
unsigned char Get_Digtal_For_User(No_MCU_Sensor *sensor);
unsigned char Get_Normalize_For_User(No_MCU_Sensor *sensor, unsigned short *result);
unsigned char Get_Anolog_Value(No_MCU_Sensor *sensor, unsigned short *result);

#ifdef __cplusplus
}
#endif

#endif
