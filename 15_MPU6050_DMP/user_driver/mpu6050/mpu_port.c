#include "mpu_port.h"
#include "delay.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include <math.h>

#define I2C_TIMEOUT_LOOPS (100000U)

volatile uint32_t sys_tick_ms = 0;

static int wait_i2c_idle(void)
{
    volatile uint32_t timeout = I2C_TIMEOUT_LOOPS;

    while (!(DL_I2C_getControllerStatus(I2C_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--timeout == 0U) {
            return -1;
        }
    }

    return 0;
}

static int wait_i2c_not_busy(void)
{
    volatile uint32_t timeout = I2C_TIMEOUT_LOOPS;

    while (DL_I2C_getControllerStatus(I2C_INST) & DL_I2C_CONTROLLER_STATUS_BUSY) {
        if (DL_I2C_getControllerStatus(I2C_INST) & DL_I2C_CONTROLLER_STATUS_ERROR) {
            return -2;
        }
        if (--timeout == 0U) {
            return -1;
        }
    }

    return 0;
}

void mget_ms(unsigned long *time)
{
    if (time != 0) {
        *time = sys_tick_ms;
    }
}

int MPU_Write_Len(unsigned char addr, unsigned char reg, unsigned char len, unsigned char *buf)
{
    uint16_t i;
    volatile uint32_t timeout;

    if (wait_i2c_idle() != 0) {
        return -1;
    }

    DL_I2C_transmitControllerData(I2C_INST, reg);
    DL_I2C_startControllerTransfer(I2C_INST, addr, DL_I2C_CONTROLLER_DIRECTION_TX, (uint16_t) len + 1U);

    for (i = 0; i < len; i++) {
        timeout = I2C_TIMEOUT_LOOPS;
        while (DL_I2C_isControllerTXFIFOFull(I2C_INST)) {
            if (--timeout == 0U) {
                return -2;
            }
        }
        DL_I2C_transmitControllerData(I2C_INST, buf[i]);
    }

    if (wait_i2c_not_busy() != 0) {
        return -3;
    }

    return wait_i2c_idle();
}

int MPU_Read_Len(unsigned char addr, unsigned char reg, unsigned char len, unsigned char *buf)
{
    uint16_t i;
    volatile uint32_t timeout;

    if (wait_i2c_idle() != 0) {
        return -1;
    }

    DL_I2C_transmitControllerData(I2C_INST, reg);
    DL_I2C_startControllerTransfer(I2C_INST, addr, DL_I2C_CONTROLLER_DIRECTION_TX, 1U);

    if (wait_i2c_not_busy() != 0) {
        return -2;
    }
    if (wait_i2c_idle() != 0) {
        return -3;
    }

    DL_I2C_startControllerTransfer(I2C_INST, addr, DL_I2C_CONTROLLER_DIRECTION_RX, len);

    for (i = 0; i < len; i++) {
        timeout = I2C_TIMEOUT_LOOPS;
        while (DL_I2C_isControllerRXFIFOEmpty(I2C_INST)) {
            if (DL_I2C_getControllerStatus(I2C_INST) & DL_I2C_CONTROLLER_STATUS_ERROR) {
                return -4;
            }
            if (--timeout == 0U) {
                return -5;
            }
        }
        buf[i] = DL_I2C_receiveControllerData(I2C_INST);
    }

    if (wait_i2c_not_busy() != 0) {
        return -6;
    }

    return wait_i2c_idle();
}

static signed char gyro_orientation[9] = {
    1, 0, 0,
    0, 1, 0,
    0, 0, 1
};

unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0) {
        b = 0;
    } else if (row[0] < 0) {
        b = 4;
    } else if (row[1] > 0) {
        b = 1;
    } else if (row[1] < 0) {
        b = 5;
    } else if (row[2] > 0) {
        b = 2;
    } else if (row[2] < 0) {
        b = 6;
    } else {
        b = 7;
    }

    return b;
}

unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx)
{
    unsigned short scalar;

    scalar = inv_row_2_scale(mtx);
    scalar |= (unsigned short) (inv_row_2_scale(mtx + 3) << 3);
    scalar |= (unsigned short) (inv_row_2_scale(mtx + 6) << 6);

    return scalar;
}

int DMP_Init(void)
{
    int result;

    delay_ms(100);

    result = mpu_init();
    if (result != 0) {
        return result;
    }
    result = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    if (result != 0) {
        return result;
    }
    result = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    if (result != 0) {
        return result;
    }
    result = mpu_set_sample_rate(100);
    if (result != 0) {
        return result;
    }
    result = dmp_load_motion_driver_firmware();
    if (result != 0) {
        return result;
    }
    result = dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));
    if (result != 0) {
        return result;
    }
    result = dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
        DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL |
        DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL);
    if (result != 0) {
        return result;
    }
    result = dmp_set_fifo_rate(100);
    if (result != 0) {
        return result;
    }

    return mpu_set_dmp_state(1);
}

#define Q30_SCALE (1073741824.0f)

int DMP_Read_Data(float *pitch, float *roll, float *yaw)
{
    short gyro[3];
    short accel[3];
    short sensors;
    unsigned char more;
    long quat[4];

    if (dmp_read_fifo(gyro, accel, quat, 0, &sensors, &more) == 0) {
        if (sensors & INV_WXYZ_QUAT) {
            float q0 = quat[0] / Q30_SCALE;
            float q1 = quat[1] / Q30_SCALE;
            float q2 = quat[2] / Q30_SCALE;
            float q3 = quat[3] / Q30_SCALE;

            *pitch = asinf(-2.0f * q1 * q3 + 2.0f * q0 * q2) * 57.3f;
            *roll = atan2f(2.0f * q2 * q3 + 2.0f * q0 * q1,
                -2.0f * q1 * q1 - 2.0f * q2 * q2 + 1.0f) * 57.3f;
            *yaw = atan2f(2.0f * (q1 * q2 + q0 * q3),
                q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.3f;

            return 0;
        }
    }

    return -1;
}
