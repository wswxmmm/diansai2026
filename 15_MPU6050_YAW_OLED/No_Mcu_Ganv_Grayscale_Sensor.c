#include "No_Mcu_Ganv_Grayscale_Sensor_Config.h"

void Get_Analog_value(unsigned short *result)
{
    unsigned char i;
    unsigned char j;
    unsigned int analog;

    for (i = 0; i < 8; i++) {
        analog = 0;

        Switch_Address_0(!(i & 0x01));
        Switch_Address_1(!(i & 0x02));
        Switch_Address_2(!(i & 0x04));
        delay_us(1);

        for (j = 0; j < 8; j++) {
            analog += Get_adc_of_user();
        }

        if (!Direction) {
            result[i] = (unsigned short) (analog / 8U);
        } else {
            result[7U - i] = (unsigned short) (analog / 8U);
        }
    }
}

void convertAnalogToDigital(unsigned short *adc_value, unsigned short *Gray_white,
    unsigned short *Gray_black, unsigned char *Digital)
{
    int i;

    for (i = 0; i < 8; i++) {
        if (adc_value[i] > Gray_white[i]) {
            *Digital |= (unsigned char) (1U << i);
        } else if (adc_value[i] < Gray_black[i]) {
            *Digital &= (unsigned char) ~(1U << i);
        }
    }
}

void normalizeAnalogValues(unsigned short *adc_value, double *Normal_factor,
    unsigned short *Calibrated_black, unsigned short *result, double bits)
{
    int i;

    for (i = 0; i < 8; i++) {
        unsigned short n;

        if (adc_value[i] < Calibrated_black[i]) {
            n = 0;
        } else {
            n = (unsigned short) ((adc_value[i] - Calibrated_black[i]) * Normal_factor[i]);
        }

        if (n > bits) {
            n = (unsigned short) bits;
        }

        result[i] = n;
    }
}

void No_MCU_Ganv_Sensor_Init_Frist(No_MCU_Sensor *sensor)
{
    memset(sensor->Calibrated_black, 0, sizeof(sensor->Calibrated_black));
    memset(sensor->Calibrated_white, 0, sizeof(sensor->Calibrated_white));
    memset(sensor->Normal_value, 0, sizeof(sensor->Normal_value));
    memset(sensor->Analog_value, 0, sizeof(sensor->Analog_value));

    for (int i = 0; i < 8; i++) {
        sensor->Normal_factor[i] = 0.0;
    }

    sensor->Digtal = 0;
    sensor->Time_out = 0;
    sensor->Tick = 0;
    sensor->ok = 0;
}

void No_MCU_Ganv_Sensor_Init(No_MCU_Sensor *sensor,
    unsigned short *Calibrated_white, unsigned short *Calibrated_black)
{
    unsigned short temp;

    No_MCU_Ganv_Sensor_Init_Frist(sensor);

    if (Sensor_ADCbits == _8Bits) {
        sensor->bits = 255.0;
    } else if (Sensor_ADCbits == _10Bits) {
        sensor->bits = 1024.0;
    } else if (Sensor_ADCbits == _12Bits) {
        sensor->bits = 4096.0;
    } else if (Sensor_ADCbits == _14Bits) {
        sensor->bits = 16384.0;
    }

    if (Sensor_Edition == Class) {
        sensor->Time_out = 1;
    } else {
        sensor->Time_out = 10;
    }

    for (int i = 0; i < 8; i++) {
        if (Calibrated_black[i] >= Calibrated_white[i]) {
            temp = Calibrated_white[i];
            Calibrated_white[i] = Calibrated_black[i];
            Calibrated_black[i] = temp;
        }

        sensor->Gray_white[i] = (unsigned short) ((Calibrated_white[i] * 2U + Calibrated_black[i]) / 3U);
        sensor->Gray_black[i] = (unsigned short) ((Calibrated_white[i] + Calibrated_black[i] * 2U) / 3U);
        sensor->Calibrated_black[i] = Calibrated_black[i];
        sensor->Calibrated_white[i] = Calibrated_white[i];

        if (((Calibrated_white[i] == 0U) && (Calibrated_black[i] == 0U)) ||
            (Calibrated_white[i] == Calibrated_black[i])) {
            sensor->Normal_factor[i] = 0.0;
            continue;
        }

        sensor->Normal_factor[i] =
            sensor->bits / ((double) Calibrated_white[i] - (double) Calibrated_black[i]);
    }

    sensor->ok = 1;
}

void No_Mcu_Ganv_Sensor_Task_Without_tick(No_MCU_Sensor *sensor)
{
    Get_Analog_value(sensor->Analog_value);
    convertAnalogToDigital(sensor->Analog_value,
        sensor->Gray_white, sensor->Gray_black, &sensor->Digtal);
    normalizeAnalogValues(sensor->Analog_value,
        sensor->Normal_factor, sensor->Calibrated_black,
        sensor->Normal_value, sensor->bits);
}

unsigned char Get_Digtal_For_User(No_MCU_Sensor *sensor)
{
    return sensor->Digtal;
}

unsigned char Get_Normalize_For_User(No_MCU_Sensor *sensor, unsigned short *result)
{
    if (!sensor->ok) {
        return 0;
    }

    memcpy(result, sensor->Normal_value, sizeof(sensor->Normal_value));
    return 1;
}

unsigned char Get_Anolog_Value(No_MCU_Sensor *sensor, unsigned short *result)
{
    memcpy(result, sensor->Analog_value, sizeof(sensor->Analog_value));
    if (!sensor->ok) {
        return 0;
    }

    return 1;
}
