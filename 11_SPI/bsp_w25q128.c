#include "bsp_w25q128.h"

static uint8_t spi_read_write_byte(uint8_t data)
{
    DL_SPI_transmitData8(SPI_INST, data);
    while (DL_SPI_isBusy(SPI_INST)) {
    }

    return DL_SPI_receiveData8(SPI_INST);
}

uint16_t W25Q128_readID(void)
{
    uint16_t id = 0;

    SPI_CS(0);
    spi_read_write_byte(0x90U);
    spi_read_write_byte(0x00U);
    spi_read_write_byte(0x00U);
    spi_read_write_byte(0x00U);
    id |= ((uint16_t) spi_read_write_byte(0xFFU)) << 8;
    id |= spi_read_write_byte(0xFFU);
    SPI_CS(1);

    return id;
}

static void W25Q128_write_enable(void)
{
    SPI_CS(0);
    spi_read_write_byte(0x06U);
    SPI_CS(1);
}

static void W25Q128_wait_busy(void)
{
    uint8_t status = 0;

    do {
        SPI_CS(0);
        spi_read_write_byte(0x05U);
        status = spi_read_write_byte(0xFFU);
        SPI_CS(1);
    } while ((status & 0x01U) != 0U);
}

static void W25Q128_erase_sector(uint32_t addr)
{
    addr *= 4096U;

    W25Q128_write_enable();
    W25Q128_wait_busy();

    SPI_CS(0);
    spi_read_write_byte(0x20U);
    spi_read_write_byte((uint8_t) (addr >> 16));
    spi_read_write_byte((uint8_t) (addr >> 8));
    spi_read_write_byte((uint8_t) addr);
    SPI_CS(1);

    W25Q128_wait_busy();
}

void W25Q128_write(uint8_t *buffer, uint32_t addr, uint16_t numbyte)
{
    uint16_t i = 0;

    W25Q128_erase_sector(addr / 4096U);
    W25Q128_write_enable();
    W25Q128_wait_busy();

    SPI_CS(0);
    spi_read_write_byte(0x02U);
    spi_read_write_byte((uint8_t) (addr >> 16));
    spi_read_write_byte((uint8_t) (addr >> 8));
    spi_read_write_byte((uint8_t) addr);

    for (i = 0; i < numbyte; i++) {
        spi_read_write_byte(buffer[i]);
    }

    SPI_CS(1);
    W25Q128_wait_busy();
}

void W25Q128_read(uint8_t *buffer, uint32_t read_addr, uint16_t read_length)
{
    uint16_t i = 0;

    SPI_CS(0);
    spi_read_write_byte(0x03U);
    spi_read_write_byte((uint8_t) (read_addr >> 16));
    spi_read_write_byte((uint8_t) (read_addr >> 8));
    spi_read_write_byte((uint8_t) read_addr);

    for (i = 0; i < read_length; i++) {
        buffer[i] = spi_read_write_byte(0xFFU);
    }

    SPI_CS(1);
}
