#include "bsp_mpu6050.h"
#include <stddef.h>

#define MPU6050_REG_SMPLRT_DIV       (0x19U)
#define MPU6050_REG_CONFIG           (0x1AU)
#define MPU6050_REG_GYRO_CONFIG      (0x1BU)
#define MPU6050_REG_ACCEL_CONFIG     (0x1CU)
#define MPU6050_REG_ACCEL_XOUT_H     (0x3BU)
#define MPU6050_REG_PWR_MGMT_1       (0x6BU)
#define MPU6050_REG_WHO_AM_I         (0x75U)

#define GYRO_2000DPS_LSB_PER_DPS_X10 (164) /* 16.4 LSB/(deg/s) */
#define I2C_TIMEOUT                  (200000U)

static uint8_t g_mpuAddr = 0x68U;
static uint8_t g_whoAmI = 0;
static uint8_t g_initStep = 0;
static int32_t g_gyroZBias = 0;
static int32_t g_yawMilliDeg = 0;

static void mpu_i2c_recover(void)
{
    DL_I2C_resetControllerTransfer(MPU_I2C_INST);
    DL_I2C_flushControllerTXFIFO(MPU_I2C_INST);
    DL_I2C_flushControllerRXFIFO(MPU_I2C_INST);
}

static bool wait_i2c_idle(void)
{
    uint32_t timeout = I2C_TIMEOUT;

    while (!(DL_I2C_getControllerStatus(MPU_I2C_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (timeout-- == 0U) {
            mpu_i2c_recover();
            return false;
        }
    }
    return true;
}

static bool wait_i2c_not_busy(void)
{
    uint32_t timeout = I2C_TIMEOUT;

    while (DL_I2C_getControllerStatus(MPU_I2C_INST) & DL_I2C_CONTROLLER_STATUS_BUSY) {
        if (timeout-- == 0U) {
            mpu_i2c_recover();
            return false;
        }
    }
    return true;
}

static bool wait_rx_data(void)
{
    uint32_t timeout = I2C_TIMEOUT;

    while (DL_I2C_isControllerRXFIFOEmpty(MPU_I2C_INST)) {
        if (timeout-- == 0U) {
            mpu_i2c_recover();
            return false;
        }
    }
    return true;
}

static bool i2c_ok(void)
{
    bool ok = (DL_I2C_getControllerStatus(MPU_I2C_INST) & DL_I2C_CONTROLLER_STATUS_ERROR) == 0U;
    if (!ok) {
        mpu_i2c_recover();
    }
    return ok;
}

static bool MPU6050_WriteRegAddr(uint8_t addr, uint8_t reg, uint8_t value)
{
    uint32_t timeout;

    mpu_i2c_recover();
    if (!wait_i2c_idle()) {
        return false;
    }

    DL_I2C_transmitControllerData(MPU_I2C_INST, reg);
    DL_I2C_startControllerTransfer(MPU_I2C_INST, addr,
        DL_I2C_CONTROLLER_DIRECTION_TX, 2U);
    delay_cycles(1000U);

    timeout = I2C_TIMEOUT;
    while (DL_I2C_isControllerTXFIFOFull(MPU_I2C_INST)) {
        if (timeout-- == 0U) {
            mpu_i2c_recover();
            return false;
        }
    }
    DL_I2C_transmitControllerData(MPU_I2C_INST, value);

    return wait_i2c_not_busy() && wait_i2c_idle() && i2c_ok();
}
static bool MPU6050_ReadRegsAddr(uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t len)
{
    if ((buffer == NULL) || (len == 0U)) {
        return false;
    }

    mpu_i2c_recover();
    if (!wait_i2c_idle()) {
        return false;
    }

    DL_I2C_transmitControllerData(MPU_I2C_INST, reg);
    DL_I2C_startControllerTransfer(MPU_I2C_INST, addr,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1U);
    delay_cycles(1000U);

    if (!wait_i2c_not_busy() || !wait_i2c_idle() || !i2c_ok()) {
        return false;
    }

    DL_I2C_startControllerTransfer(MPU_I2C_INST, addr,
        DL_I2C_CONTROLLER_DIRECTION_RX, len);
    delay_cycles(1000U);

    for (uint8_t i = 0; i < len; i++) {
        if (!wait_rx_data()) {
            return false;
        }
        buffer[i] = DL_I2C_receiveControllerData(MPU_I2C_INST);
    }

    return wait_i2c_not_busy() && wait_i2c_idle() && i2c_ok();
}
static bool MPU6050_WriteReg(uint8_t reg, uint8_t value)
{
    return MPU6050_WriteRegAddr(g_mpuAddr, reg, value);
}

static bool MPU6050_ReadRegs(uint8_t reg, uint8_t *buffer, uint8_t len)
{
    return MPU6050_ReadRegsAddr(g_mpuAddr, reg, buffer, len);
}

bool MPU6050_Init(void)
{
    uint8_t who = 0;
    const uint8_t addrs[] = {0x68U, 0x69U};
    bool found = false;

    g_initStep = 1U;
    g_whoAmI = 0U;
    delay_cycles(CPUCLK_FREQ / 10U);

    for (uint32_t i = 0; i < (sizeof(addrs) / sizeof(addrs[0])); i++) {
        if (MPU6050_ReadRegsAddr(addrs[i], MPU6050_REG_WHO_AM_I, &who, 1U)) {
            g_mpuAddr = addrs[i];
            g_whoAmI = who;
            found = true;
            break;
        }
    }

    if (!found) {
        return false;
    }

    g_initStep = 2U;
    (void) MPU6050_WriteReg(MPU6050_REG_PWR_MGMT_1, 0x80U);
    delay_cycles(CPUCLK_FREQ / 10U);

    g_initStep = 3U;
    if (!MPU6050_WriteReg(MPU6050_REG_PWR_MGMT_1, 0x01U)) {
        return false;
    }
    delay_cycles(CPUCLK_FREQ / 20U);

    g_initStep = 4U;
    if (!MPU6050_WriteReg(MPU6050_REG_SMPLRT_DIV, 0x07U)) {
        return false;
    }
    g_initStep = 5U;
    if (!MPU6050_WriteReg(MPU6050_REG_CONFIG, 0x06U)) {
        return false;
    }
    g_initStep = 6U;
    if (!MPU6050_WriteReg(MPU6050_REG_GYRO_CONFIG, 0x18U)) {
        return false;
    }
    g_initStep = 7U;
    if (!MPU6050_WriteReg(MPU6050_REG_ACCEL_CONFIG, 0x00U)) {
        return false;
    }

    g_initStep = 0U;
    MPU6050_ResetYaw();
    return true;
}


uint8_t MPU6050_GetAddress(void)
{
    return g_mpuAddr;
}

uint8_t MPU6050_GetWhoAmI(void)
{
    return g_whoAmI;
}

uint8_t MPU6050_GetInitStep(void)
{
    return g_initStep;
}

bool MPU6050_ReadRaw(MPU6050RawData *data)
{
    uint8_t raw[14];

    if (data == NULL) {
        return false;
    }
    if (!MPU6050_ReadRegs(MPU6050_REG_ACCEL_XOUT_H, raw, sizeof(raw))) {
        return false;
    }

    data->accel_x = (int16_t) (((uint16_t) raw[0] << 8) | raw[1]);
    data->accel_y = (int16_t) (((uint16_t) raw[2] << 8) | raw[3]);
    data->accel_z = (int16_t) (((uint16_t) raw[4] << 8) | raw[5]);
    data->gyro_x = (int16_t) (((uint16_t) raw[8] << 8) | raw[9]);
    data->gyro_y = (int16_t) (((uint16_t) raw[10] << 8) | raw[11]);
    data->gyro_z = (int16_t) (((uint16_t) raw[12] << 8) | raw[13]);

    return true;
}

void MPU6050_CalibrateGyroZ(void)
{
    int64_t sum = 0;
    uint32_t valid = 0;
    MPU6050RawData data;

    for (uint32_t i = 0; i < MPU6050_CALIBRATION_SAMPLES; i++) {
        if (MPU6050_ReadRaw(&data)) {
            sum += data.gyro_z;
            valid++;
        }
        delay_cycles(CPUCLK_FREQ / 1000U);
    }

    g_gyroZBias = (valid > 0U) ? (int32_t) (sum / (int64_t) valid) : 0;
    MPU6050_ResetYaw();
}

void MPU6050_ResetYaw(void)
{
    g_yawMilliDeg = 0;
}

int32_t MPU6050_UpdateYawMilliDeg(void)
{
    MPU6050RawData data;
    int32_t gyroRaw;
    int32_t gyroDpsMilli;

    if (!MPU6050_ReadRaw(&data)) {
        return g_yawMilliDeg;
    }

    gyroRaw = (int32_t) data.gyro_z - g_gyroZBias;
    gyroDpsMilli = (gyroRaw * 10000) / GYRO_2000DPS_LSB_PER_DPS_X10;
    g_yawMilliDeg += (gyroDpsMilli * (int32_t) MPU6050_SAMPLE_MS) / 1000;

    while (g_yawMilliDeg >= 360000) {
        g_yawMilliDeg -= 360000;
    }
    while (g_yawMilliDeg < 0) {
        g_yawMilliDeg += 360000;
    }

    return g_yawMilliDeg;
}
int32_t MPU6050_GetYawMilliDeg(void)
{
    return g_yawMilliDeg;
}