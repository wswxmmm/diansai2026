#include "bsp_sht20.h"

#define SHT20_ADDR_WRITE (0x80U)
#define SHT20_ADDR_READ  (0x81U)
#define delay_us(x)      delay_cycles((CPUCLK_FREQ / 1000000U) * (x))
#define delay_ms(x)      delay_cycles((CPUCLK_FREQ / 1000U) * (x))

static void IIC_Start(void)
{
    SDA_OUT();
    SDA(1);
    SCL(1);
    delay_us(5);
    SDA(0);
    delay_us(5);
    SCL(0);
    delay_us(5);
}

static void IIC_Stop(void)
{
    SDA_OUT();
    SCL(0);
    SDA(0);
    delay_us(5);
    SCL(1);
    delay_us(5);
    SDA(1);
    delay_us(5);
}

static void IIC_Send_Ack(uint8_t ack)
{
    SDA_OUT();
    SCL(0);
    SDA(ack ? 1 : 0);
    delay_us(5);
    SCL(1);
    delay_us(5);
    SCL(0);
    SDA(1);
}

static uint8_t IIC_Wait_Ack(void)
{
    uint8_t timeout = 200;

    SDA_IN();
    SCL(1);
    delay_us(2);

    while (SDA_GET() != 0U) {
        if (timeout-- == 0U) {
            SCL(0);
            SDA_OUT();
            return 1U;
        }
        delay_us(1);
    }

    SCL(0);
    SDA_OUT();
    return 0U;
}

static void IIC_Send_Byte(uint8_t data)
{
    uint8_t i = 0;

    SDA_OUT();
    for (i = 0; i < 8U; i++) {
        SCL(0);
        SDA((data & 0x80U) != 0U);
        data <<= 1;
        delay_us(2);
        SCL(1);
        delay_us(5);
    }
    SCL(0);
}

static uint8_t IIC_Read_Byte(uint8_t ack)
{
    uint8_t i = 0;
    uint8_t data = 0;

    SDA_IN();
    for (i = 0; i < 8U; i++) {
        SCL(0);
        delay_us(2);
        SCL(1);
        data <<= 1;
        if (SDA_GET() != 0U) {
            data |= 1U;
        }
        delay_us(5);
    }
    SCL(0);
    SDA_OUT();
    IIC_Send_Ack(ack);

    return data;
}

float SHT20_Read(unsigned char command)
{
    uint16_t raw = 0;
    uint8_t msb = 0;
    uint8_t lsb = 0;

    IIC_Start();
    IIC_Send_Byte(SHT20_ADDR_WRITE);
    if (IIC_Wait_Ack() != 0U) {
        IIC_Stop();
        return -1.0f;
    }

    IIC_Send_Byte(command);
    if (IIC_Wait_Ack() != 0U) {
        IIC_Stop();
        return -1.0f;
    }
    IIC_Stop();

    if (command == SHT20_CMD_TEMP_NO_HOLD) {
        delay_ms(85);
    } else {
        delay_ms(30);
    }

    IIC_Start();
    IIC_Send_Byte(SHT20_ADDR_READ);
    if (IIC_Wait_Ack() != 0U) {
        IIC_Stop();
        return -1.0f;
    }

    msb = IIC_Read_Byte(0U);
    lsb = IIC_Read_Byte(0U);
    (void) IIC_Read_Byte(1U);
    IIC_Stop();

    raw = (uint16_t) (((uint16_t) msb << 8) | lsb);
    raw &= 0xFFFCU;

    if (command == SHT20_CMD_TEMP_NO_HOLD) {
        return -46.85f + (175.72f * (float) raw / 65536.0f);
    }

    return -6.0f + (125.0f * (float) raw / 65536.0f);
}
