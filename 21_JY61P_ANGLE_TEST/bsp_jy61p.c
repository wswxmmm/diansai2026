#include "bsp_jy61p.h"
#include "ti_msp_dl_config.h"
#include <stddef.h>

#define I2C_DELAY_US           (5U)
#define I2C_ACK_TIMEOUT_US     (100U)
#define delay_us(x)            delay_cycles((CPUCLK_FREQ / 1000000U) * (x))

#define SCL_HIGH() DL_GPIO_setPins(IIC_Software_PORT, IIC_Software_SCL_PIN)
#define SCL_LOW()  DL_GPIO_clearPins(IIC_Software_PORT, IIC_Software_SCL_PIN)
#define SDA_HIGH() DL_GPIO_setPins(IIC_Software_PORT, IIC_Software_SDA_PIN)
#define SDA_LOW()  DL_GPIO_clearPins(IIC_Software_PORT, IIC_Software_SDA_PIN)

static JY61P_Attitude g_attitude;

static void sda_output(void)
{
    DL_GPIO_initDigitalOutput(IIC_Software_SDA_IOMUX);
    SDA_HIGH();
    DL_GPIO_enableOutput(IIC_Software_PORT, IIC_Software_SDA_PIN);
}

static void sda_input(void)
{
    DL_GPIO_disableOutput(IIC_Software_PORT, IIC_Software_SDA_PIN);
    DL_GPIO_initDigitalInput(IIC_Software_SDA_IOMUX);
}

static bool sda_is_high(void)
{
    return (DL_GPIO_readPins(IIC_Software_PORT, IIC_Software_SDA_PIN) &
        IIC_Software_SDA_PIN) != 0U;
}

static void i2c_start(void)
{
    sda_output();
    SDA_HIGH();
    SCL_HIGH();
    delay_us(I2C_DELAY_US);
    SDA_LOW();
    delay_us(I2C_DELAY_US);
    SCL_LOW();
    delay_us(I2C_DELAY_US);
}

static void i2c_stop(void)
{
    sda_output();
    SCL_LOW();
    SDA_LOW();
    delay_us(I2C_DELAY_US);
    SCL_HIGH();
    delay_us(I2C_DELAY_US);
    SDA_HIGH();
    delay_us(I2C_DELAY_US);
}

static void i2c_send_byte(uint8_t data)
{
    uint8_t bit;

    sda_output();
    for (bit = 0U; bit < 8U; bit++) {
        SCL_LOW();
        if ((data & 0x80U) != 0U) {
            SDA_HIGH();
        } else {
            SDA_LOW();
        }
        delay_us(2U);
        SCL_HIGH();
        delay_us(I2C_DELAY_US);
        data <<= 1U;
    }
    SCL_LOW();
}

static bool i2c_wait_ack(void)
{
    uint32_t timeout = I2C_ACK_TIMEOUT_US;

    sda_input();
    delay_us(2U);
    SCL_HIGH();
    delay_us(2U);
    while (sda_is_high()) {
        if (timeout-- == 0U) {
            SCL_LOW();
            sda_output();
            return false;
        }
        delay_us(1U);
    }
    delay_us(2U);
    SCL_LOW();
    sda_output();
    return true;
}

static uint8_t i2c_read_byte(void)
{
    uint8_t bit;
    uint8_t data = 0U;

    sda_input();
    for (bit = 0U; bit < 8U; bit++) {
        SCL_LOW();
        delay_us(I2C_DELAY_US);
        SCL_HIGH();
        delay_us(2U);
        data <<= 1U;
        if (sda_is_high()) {
            data |= 1U;
        }
        delay_us(3U);
    }
    SCL_LOW();
    return data;
}

static void i2c_send_ack(bool nack)
{
    sda_output();
    SCL_LOW();
    if (nack) {
        SDA_HIGH();
    } else {
        SDA_LOW();
    }
    delay_us(I2C_DELAY_US);
    SCL_HIGH();
    delay_us(I2C_DELAY_US);
    SCL_LOW();
    SDA_HIGH();
}

static void i2c_recover_bus(void)
{
    uint8_t pulse;

    sda_input();
    SCL_HIGH();
    delay_us(I2C_DELAY_US);
    if (!sda_is_high()) {
        for (pulse = 0U; pulse < 9U; pulse++) {
            SCL_LOW();
            delay_us(I2C_DELAY_US);
            SCL_HIGH();
            delay_us(I2C_DELAY_US);
        }
    }
    i2c_stop();
}

static bool read_registers(uint8_t device, uint8_t reg, uint8_t *data,
    uint32_t length, uint8_t *error)
{
    uint32_t index;

    *error = 0U;
    i2c_start();
    i2c_send_byte((uint8_t) (device << 1U));
    if (!i2c_wait_ack()) {
        *error = 1U;
        i2c_stop();
        return false;
    }

    i2c_send_byte(reg);
    if (!i2c_wait_ack()) {
        *error = 2U;
        i2c_stop();
        return false;
    }

    delay_us(I2C_DELAY_US);
    i2c_start();
    i2c_send_byte((uint8_t) ((device << 1U) | 1U));
    if (!i2c_wait_ack()) {
        *error = 3U;
        i2c_stop();
        return false;
    }

    for (index = 0U; index < length; index++) {
        data[index] = i2c_read_byte();
        i2c_send_ack(index == (length - 1U));
    }
    i2c_stop();
    return true;
}

static int16_t decode_i16_le(const uint8_t *data)
{
    return (int16_t) (((uint16_t) data[1] << 8U) | data[0]);
}

static int16_t raw_angle_to_cdeg(int16_t raw)
{
    return (int16_t) (((int32_t) raw * 18000L) / 32768L);
}

void JY61P_Init(void)
{
    g_attitude.roll_cdeg = 0;
    g_attitude.pitch_cdeg = 0;
    g_attitude.yaw_cdeg = 0;
    g_attitude.read_count = 0U;
    g_attitude.read_errors = 0U;
    g_attitude.last_error = 0U;
    i2c_recover_bus();
}

bool JY61P_ReadAttitude(JY61P_Attitude *attitude)
{
    uint8_t angle_data[6];
    uint8_t error;

    if (attitude == NULL) {
        return false;
    }

    if (!read_registers(JY61P_I2C_ADDRESS, JY61P_ANGLE_REGISTER,
            angle_data, sizeof(angle_data), &error)) {
        g_attitude.read_errors++;
        g_attitude.last_error = error;
        *attitude = g_attitude;
        return false;
    }

    g_attitude.roll_cdeg = raw_angle_to_cdeg(decode_i16_le(&angle_data[0]));
    g_attitude.pitch_cdeg = raw_angle_to_cdeg(decode_i16_le(&angle_data[2]));
    g_attitude.yaw_cdeg = raw_angle_to_cdeg(decode_i16_le(&angle_data[4]));
    g_attitude.read_count++;
    g_attitude.last_error = 0U;
    *attitude = g_attitude;
    return true;
}
