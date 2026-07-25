#include "bsp_mpu6050_sw.h"

#define MPU_ADDR_LOW              (0x68U)
#define MPU_ADDR_HIGH             (0x69U)
#define MPU_REG_SMPLRT_DIV        (0x19U)
#define MPU_REG_CONFIG            (0x1AU)
#define MPU_REG_GYRO_CONFIG       (0x1BU)
#define MPU_REG_PWR_MGMT_1        (0x6BU)
#define MPU_REG_WHO_AM_I          (0x75U)
#define MPU_REG_GYRO_ZOUT_H       (0x47U)
#define MPU_I2C_DELAY_CYCLES      (CPUCLK_FREQ / 200000U)
#define MPU_CALIBRATION_SAMPLES   (1000U)
#define MPU_GYRO_LSB_PER_DPS      (131)

#define MPU_SDA_PORT              GPIO_MPU_I2C_SDA_PORT
#define MPU_SDA_PIN               GPIO_MPU_I2C_SDA_PIN
#define MPU_SDA_IOMUX             GPIO_MPU_I2C_IOMUX_SDA
#define MPU_SCL_PORT              GPIO_MPU_I2C_SCL_PORT
#define MPU_SCL_PIN               GPIO_MPU_I2C_SCL_PIN
#define MPU_SCL_IOMUX             GPIO_MPU_I2C_IOMUX_SCL

static uint8_t g_address = MPU_ADDR_LOW;
static uint8_t g_who_am_i;
static int32_t g_gyro_z_bias;

static void i2c_delay(void)
{
    delay_cycles(MPU_I2C_DELAY_CYCLES);
}

static void sda_release(void)
{
    DL_GPIO_disableOutput(MPU_SDA_PORT, MPU_SDA_PIN);
}

static void sda_low(void)
{
    DL_GPIO_clearPins(MPU_SDA_PORT, MPU_SDA_PIN);
    DL_GPIO_enableOutput(MPU_SDA_PORT, MPU_SDA_PIN);
}

static void scl_release(void)
{
    DL_GPIO_disableOutput(MPU_SCL_PORT, MPU_SCL_PIN);
}

static void scl_low(void)
{
    DL_GPIO_clearPins(MPU_SCL_PORT, MPU_SCL_PIN);
    DL_GPIO_enableOutput(MPU_SCL_PORT, MPU_SCL_PIN);
}

static uint8_t read_sda(void)
{
    return ((DL_GPIO_readPins(MPU_SDA_PORT, MPU_SDA_PIN) &
        MPU_SDA_PIN) != 0U) ? 1U : 0U;
}

static void i2c_start(void)
{
    sda_release();
    scl_release();
    i2c_delay();
    sda_low();
    i2c_delay();
    scl_low();
}

static void i2c_stop(void)
{
    sda_low();
    i2c_delay();
    scl_release();
    i2c_delay();
    sda_release();
    i2c_delay();
}

static bool i2c_write_byte(uint8_t data)
{
    uint8_t mask;
    bool acknowledged;

    for (mask = 0x80U; mask != 0U; mask >>= 1U) {
        if ((data & mask) != 0U) {
            sda_release();
        } else {
            sda_low();
        }
        i2c_delay();
        scl_release();
        i2c_delay();
        scl_low();
    }

    sda_release();
    i2c_delay();
    scl_release();
    i2c_delay();
    acknowledged = (read_sda() == 0U);
    scl_low();
    return acknowledged;
}

static uint8_t i2c_read_byte(bool acknowledge)
{
    uint8_t data = 0U;
    uint8_t i;

    sda_release();
    for (i = 0U; i < 8U; i++) {
        data <<= 1U;
        scl_release();
        i2c_delay();
        if (read_sda() != 0U) {
            data |= 1U;
        }
        scl_low();
        i2c_delay();
    }

    if (acknowledge) {
        sda_low();
    } else {
        sda_release();
    }
    i2c_delay();
    scl_release();
    i2c_delay();
    scl_low();
    sda_release();
    return data;
}

static bool write_register(uint8_t address, uint8_t reg, uint8_t value)
{
    i2c_start();
    if (!i2c_write_byte((uint8_t) (address << 1U)) ||
        !i2c_write_byte(reg) || !i2c_write_byte(value)) {
        i2c_stop();
        return false;
    }
    i2c_stop();
    return true;
}

static bool read_registers(uint8_t address, uint8_t reg,
    uint8_t *data, uint8_t length)
{
    uint8_t i;

    if ((data == 0) || (length == 0U)) {
        return false;
    }

    i2c_start();
    if (!i2c_write_byte((uint8_t) (address << 1U)) ||
        !i2c_write_byte(reg)) {
        i2c_stop();
        return false;
    }

    i2c_start();
    if (!i2c_write_byte((uint8_t) ((address << 1U) | 1U))) {
        i2c_stop();
        return false;
    }

    for (i = 0U; i < length; i++) {
        data[i] = i2c_read_byte(i < (uint8_t) (length - 1U));
    }
    i2c_stop();
    return true;
}

static void recover_bus(void)
{
    uint8_t i;

    sda_release();
    for (i = 0U; i < 9U; i++) {
        scl_low();
        i2c_delay();
        scl_release();
        i2c_delay();
    }
    i2c_stop();
}

static bool read_gyro_z_raw(int16_t *gyro_z)
{
    uint8_t data[2];

    if ((gyro_z == 0) ||
        !read_registers(g_address, MPU_REG_GYRO_ZOUT_H, data, 2U)) {
        return false;
    }

    *gyro_z = (int16_t) (((uint16_t) data[0] << 8U) | data[1]);
    return true;
}

bool MPU6050_SW_Init(void)
{
    const uint8_t addresses[2] = {MPU_ADDR_LOW, MPU_ADDR_HIGH};
    uint8_t i;

    DL_GPIO_initDigitalInputFeatures(MPU_SDA_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(MPU_SCL_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_clearPins(MPU_SDA_PORT, MPU_SDA_PIN);
    DL_GPIO_clearPins(MPU_SCL_PORT, MPU_SCL_PIN);
    sda_release();
    scl_release();
    recover_bus();
    for (i = 0U; i < 2U; i++) {
        uint8_t who = 0U;
        if (read_registers(addresses[i], MPU_REG_WHO_AM_I, &who, 1U)) {
            g_address = addresses[i];
            g_who_am_i = who;
            break;
        }
    }
    if (i == 2U) {
        return false;
    }

    if (!write_register(g_address, MPU_REG_PWR_MGMT_1, 0x80U)) {
        return false;
    }
    delay_cycles(CPUCLK_FREQ / 10U);
    if (!write_register(g_address, MPU_REG_PWR_MGMT_1, 0x01U) ||
        !write_register(g_address, MPU_REG_SMPLRT_DIV, 0x09U) ||
        !write_register(g_address, MPU_REG_CONFIG, 0x03U) ||
        !write_register(g_address, MPU_REG_GYRO_CONFIG, 0x00U)) {
        return false;
    }
    delay_cycles(CPUCLK_FREQ / 20U);
    return true;
}

bool MPU6050_SW_CalibrateGyroZ(void)
{
    int64_t sum = 0;
    uint32_t valid = 0U;
    uint32_t sample;

    for (sample = 0U; sample < MPU_CALIBRATION_SAMPLES; sample++) {
        int16_t gyro_z;
        if (read_gyro_z_raw(&gyro_z)) {
            sum += gyro_z;
            valid++;
        }
        delay_cycles(CPUCLK_FREQ / 200U);
    }

    if (valid < (MPU_CALIBRATION_SAMPLES / 2U)) {
        return false;
    }
    g_gyro_z_bias = (int32_t) (sum / (int64_t) valid);
    return true;
}

bool MPU6050_SW_ReadGyroZDpsMilli(int32_t *gyro_z_mdps)
{
    int16_t gyro_z;
    uint8_t attempt;

    if (gyro_z_mdps == 0) {
        return false;
    }

    for (attempt = 0U; attempt < 3U; attempt++) {
        if (read_gyro_z_raw(&gyro_z)) {
            break;
        }
        recover_bus();
    }
    if (attempt == 3U) {
        return false;
    }

    *gyro_z_mdps = (((int32_t) gyro_z - g_gyro_z_bias) * 1000) /
        MPU_GYRO_LSB_PER_DPS;
    return true;
}

uint8_t MPU6050_SW_GetAddress(void)
{
    return g_address;
}

uint8_t MPU6050_SW_GetWhoAmI(void)
{
    return g_who_am_i;
}
