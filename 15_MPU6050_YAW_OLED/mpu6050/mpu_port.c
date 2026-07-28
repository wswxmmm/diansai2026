#include "mpu_port.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include <math.h>
#include "ti_msp_dl_config.h"

#define Q30 1073741824.0f

volatile uint32_t sys_tick_ms = 0;
volatile int g_dmp_init_step = 0;
volatile int g_mpu_last_i2c_error = 0;
volatile unsigned char g_mpu_who68 = 0;
volatile unsigned char g_mpu_who69 = 0;

void mget_ms(unsigned long *time)
{
    if (time) {
        *time = sys_tick_ms;
    }
}

int MPU_Write_Len(unsigned char addr, unsigned char reg, unsigned char len, unsigned char *buf)
{
    volatile uint32_t timeout = 100000;

    while (!(DL_I2C_getControllerStatus(MPU_I2C_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--timeout == 0) {
            g_mpu_last_i2c_error = -1;
            return -1;
        }
    }

    DL_I2C_transmitControllerData(MPU_I2C_INST, reg);
    DL_I2C_startControllerTransfer(MPU_I2C_INST, addr, DL_I2C_CONTROLLER_DIRECTION_TX, len + 1);

    for (uint16_t i = 0; i < len; i++) {
        timeout = 100000;
        while (DL_I2C_isControllerTXFIFOFull(MPU_I2C_INST)) {
            if (--timeout == 0) {
                g_mpu_last_i2c_error = -2;
                return -1;
            }
        }
        DL_I2C_transmitControllerData(MPU_I2C_INST, buf[i]);
    }

    timeout = 100000;
    while (DL_I2C_getControllerStatus(MPU_I2C_INST) & DL_I2C_CONTROLLER_STATUS_BUSY) {
        if (--timeout == 0) {
            g_mpu_last_i2c_error = -3;
            return -1;
        }
    }

    timeout = 100000;
    while (!(DL_I2C_getControllerStatus(MPU_I2C_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--timeout == 0) {
            g_mpu_last_i2c_error = -4;
            return -1;
        }
    }

    if (DL_I2C_getControllerStatus(MPU_I2C_INST) & DL_I2C_CONTROLLER_STATUS_ERROR) {
        g_mpu_last_i2c_error = -12;
        DL_I2C_resetControllerTransfer(MPU_I2C_INST);
        DL_I2C_flushControllerTXFIFO(MPU_I2C_INST);
        DL_I2C_flushControllerRXFIFO(MPU_I2C_INST);
        return -1;
    }

    g_mpu_last_i2c_error = 0;
    return 0;
}

int MPU_Read_Len(unsigned char addr, unsigned char reg, unsigned char len, unsigned char *buf)
{
    volatile uint32_t timeout;

    timeout = 100000;
    while (!(DL_I2C_getControllerStatus(MPU_I2C_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--timeout == 0) {
            g_mpu_last_i2c_error = -5;
            return -1;
        }
    }

    DL_I2C_transmitControllerData(MPU_I2C_INST, reg);
    DL_I2C_startControllerTransfer(MPU_I2C_INST, addr, DL_I2C_CONTROLLER_DIRECTION_TX, 1);

    timeout = 100000;
    while (DL_I2C_getControllerStatus(MPU_I2C_INST) & DL_I2C_CONTROLLER_STATUS_BUSY) {
        if (--timeout == 0) {
            g_mpu_last_i2c_error = -6;
            return -1;
        }
    }

    timeout = 100000;
    while (!(DL_I2C_getControllerStatus(MPU_I2C_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--timeout == 0) {
            g_mpu_last_i2c_error = -7;
            return -1;
        }
    }

    if (DL_I2C_getControllerStatus(MPU_I2C_INST) & DL_I2C_CONTROLLER_STATUS_ERROR) {
        g_mpu_last_i2c_error = -13;
        DL_I2C_resetControllerTransfer(MPU_I2C_INST);
        DL_I2C_flushControllerTXFIFO(MPU_I2C_INST);
        DL_I2C_flushControllerRXFIFO(MPU_I2C_INST);
        return -1;
    }

    DL_I2C_startControllerTransfer(MPU_I2C_INST, addr, DL_I2C_CONTROLLER_DIRECTION_RX, len);

    for (uint16_t i = 0; i < len; i++) {
        timeout = 100000;
        while (DL_I2C_isControllerRXFIFOEmpty(MPU_I2C_INST)) {
            if (DL_I2C_getControllerStatus(MPU_I2C_INST) & DL_I2C_CONTROLLER_STATUS_ERROR) {
                g_mpu_last_i2c_error = -8;
                DL_I2C_resetControllerTransfer(MPU_I2C_INST);
                DL_I2C_flushControllerTXFIFO(MPU_I2C_INST);
                DL_I2C_flushControllerRXFIFO(MPU_I2C_INST);
                return -1;
            }
            if (--timeout == 0) {
                g_mpu_last_i2c_error = -9;
                return -1;
            }
        }
        buf[i] = DL_I2C_receiveControllerData(MPU_I2C_INST);
    }

    timeout = 100000;
    while (DL_I2C_getControllerStatus(MPU_I2C_INST) & DL_I2C_CONTROLLER_STATUS_BUSY) {
        if (--timeout == 0) {
            g_mpu_last_i2c_error = -10;
            return -1;
        }
    }

    timeout = 100000;
    while (!(DL_I2C_getControllerStatus(MPU_I2C_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--timeout == 0) {
            g_mpu_last_i2c_error = -11;
            return -1;
        }
    }

    g_mpu_last_i2c_error = 0;
    return 0;
}

static signed char gyro_orientation[9] = { 1, 0, 0,
                                           0, 1, 0,
                                           0, 0, 1 };

unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;
    if (row[0] > 0) b = 0;
    else if (row[0] < 0) b = 4;
    else if (row[1] > 0) b = 1;
    else if (row[1] < 0) b = 5;
    else if (row[2] > 0) b = 2;
    else if (row[2] < 0) b = 6;
    else b = 7;
    return b;
}

unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx)
{
    unsigned short scalar;
    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;
    return scalar;
}

int DMP_Init(void)
{
    int res;

    SysTick->LOAD = (CPUCLK_FREQ / 1000U) - 1U;
    SysTick->VAL = 0U;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
    __enable_irq();

    delay_cycles(CPUCLK_FREQ / 5U);

    g_dmp_init_step = 1;
    g_mpu_who68 = 0;
    g_mpu_who69 = 0;
    (void) MPU_Read_Len(0x68U, 0x75U, 1U, (unsigned char *) &g_mpu_who68);
    (void) MPU_Read_Len(0x69U, 0x75U, 1U, (unsigned char *) &g_mpu_who69);

    g_dmp_init_step = 2;
    res = mpu_init();
    if (res) return -102;

    g_dmp_init_step = 3;
    if (mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL)) return -103;
    if (mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL)) return -104;
    if (mpu_set_sample_rate(100)) return -105;

    g_dmp_init_step = 4;
    res = dmp_load_motion_driver_firmware();
    if (res) return -106;

    g_dmp_init_step = 5;
    if (dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation))) return -107;
    if (dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT |
                           DMP_FEATURE_SEND_RAW_ACCEL |
                           DMP_FEATURE_SEND_CAL_GYRO |
                           DMP_FEATURE_GYRO_CAL)) return -108;
    if (dmp_set_fifo_rate(100)) return -109;

    g_dmp_init_step = 6;
    res = mpu_set_dmp_state(1);
    if (res) return -110;

    g_dmp_init_step = 0;
    return 0;
}

int DMP_Read_Data(float *pitch, float *roll, float *yaw)
{
    short gyro[3], accel[3], sensors;
    unsigned char more;
    long quat[4];

    if (dmp_read_fifo(gyro, accel, quat, NULL, &sensors, &more) == 0) {
        if (sensors & INV_WXYZ_QUAT) {
            float q0 = quat[0] / Q30;
            float q1 = quat[1] / Q30;
            float q2 = quat[2] / Q30;
            float q3 = quat[3] / Q30;

            *pitch = asin(-2.0f * q1 * q3 + 2.0f * q0 * q2) * 57.29578f;
            *roll  = atan2(2.0f * q2 * q3 + 2.0f * q0 * q1,
                           -2.0f * q1 * q1 - 2.0f * q2 * q2 + 1.0f) * 57.29578f;
            *yaw   = atan2(2.0f * (q1 * q2 + q0 * q3),
                           q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.29578f;
            return 0;
        }
    }
    return -1;
}
