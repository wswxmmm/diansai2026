#include "ti_msp_dl_config.h"
#include "No_Mcu_Ganv_Grayscale_Sensor_Config.h"
#include "oled.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define MPU_SDA_PORT   GPIO_MPU_I2C_SDA_PORT
#define MPU_SDA_PIN    GPIO_MPU_I2C_SDA_PIN
#define MPU_SDA_IOMUX  GPIO_MPU_I2C_IOMUX_SDA
#define MPU_SCL_PORT   GPIO_MPU_I2C_SCL_PORT
#define MPU_SCL_PIN    GPIO_MPU_I2C_SCL_PIN
#define MPU_SCL_IOMUX  GPIO_MPU_I2C_IOMUX_SCL

#define MPU_ADDR_68    0x68U
#define MPU_ADDR_69    0x69U
#define GYRO_LSB_PER_DPS 131.0f
#define CALIBRATION_SAMPLES 200U
#define GRAY_SAMPLE_PERIOD_MS 100U
#define OLED_REFRESH_PERIOD_MS 100U

static volatile uint32_t g_ms;
static volatile uint8_t g_ack68;
static volatile uint8_t g_ack69;
static volatile uint8_t g_line_sda;
static volatile uint8_t g_line_scl;
static volatile uint8_t g_who;
static volatile int g_last_error;
static volatile bool g_read_ok;
static volatile int16_t g_ax;
static volatile int16_t g_ay;
static volatile int16_t g_az;
static volatile int16_t g_gx;
static volatile int16_t g_gy;
static volatile int16_t g_gz;

static bool g_mpu_configured;
static float g_gz_bias;
static float g_yaw_deg;
static float g_gz_dps;
static uint32_t g_last_sample_ms;
static uint32_t g_last_gray_ms;
static uint32_t g_last_oled_ms;

static No_MCU_Sensor g_gray_sensor;
static unsigned short g_gray_white[8] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};
static unsigned short g_gray_black[8] = {100, 100, 100, 100, 100, 100, 100, 100};
static unsigned short g_gray_normal[8];
static unsigned short g_gray_analog[8];
static unsigned char g_gray_digital;

void SysTick_Handler(void)
{
    g_ms++;
}

static uint32_t millis(void)
{
    return g_ms;
}

static void systick_init_1ms(void)
{
    SysTick->LOAD = (CPUCLK_FREQ / 1000U) - 1U;
    SysTick->VAL = 0U;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
    __enable_irq();
}

static void i2c_delay(void)
{
    delay_cycles(CPUCLK_FREQ / 50000U);
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

static bool read_sda(void)
{
    return (DL_GPIO_readPins(MPU_SDA_PORT, MPU_SDA_PIN) & MPU_SDA_PIN) != 0U;
}

static bool read_scl(void)
{
    return (DL_GPIO_readPins(MPU_SCL_PORT, MPU_SCL_PIN) & MPU_SCL_PIN) != 0U;
}

static void sw_i2c_gpio_init(void)
{
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
    delay_cycles(CPUCLK_FREQ / 1000U);
}

static void sw_i2c_recover(void)
{
    sda_release();
    scl_release();
    i2c_delay();
    for (uint8_t i = 0; i < 9U; i++) {
        scl_low();
        i2c_delay();
        scl_release();
        i2c_delay();
    }
    sda_release();
    i2c_delay();
}

static void sw_i2c_start(void)
{
    sda_release();
    scl_release();
    i2c_delay();
    sda_low();
    i2c_delay();
    scl_low();
    i2c_delay();
}

static void sw_i2c_stop(void)
{
    sda_low();
    i2c_delay();
    scl_release();
    i2c_delay();
    sda_release();
    i2c_delay();
}

static bool sw_i2c_write_byte(uint8_t data)
{
    for (uint8_t mask = 0x80U; mask != 0U; mask >>= 1U) {
        if ((data & mask) != 0U) {
            sda_release();
        } else {
            sda_low();
        }
        i2c_delay();
        scl_release();
        i2c_delay();
        scl_low();
        i2c_delay();
    }

    sda_release();
    i2c_delay();
    scl_release();
    i2c_delay();
    bool ack = !read_sda();
    scl_low();
    i2c_delay();
    return ack;
}

static uint8_t sw_i2c_read_byte(bool ack)
{
    uint8_t data = 0;

    sda_release();
    for (uint8_t i = 0; i < 8U; i++) {
        data <<= 1U;
        scl_release();
        i2c_delay();
        if (read_sda()) {
            data |= 1U;
        }
        scl_low();
        i2c_delay();
    }

    if (ack) {
        sda_low();
    } else {
        sda_release();
    }
    i2c_delay();
    scl_release();
    i2c_delay();
    scl_low();
    sda_release();
    i2c_delay();
    return data;
}

static bool sw_i2c_probe(uint8_t addr)
{
    bool ok;
    sw_i2c_start();
    ok = sw_i2c_write_byte((uint8_t) (addr << 1U));
    sw_i2c_stop();
    return ok;
}

static bool sw_i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val)
{
    sw_i2c_start();
    if (!sw_i2c_write_byte((uint8_t) (addr << 1U))) {
        sw_i2c_stop();
        g_last_error = -1;
        return false;
    }
    if (!sw_i2c_write_byte(reg)) {
        sw_i2c_stop();
        g_last_error = -2;
        return false;
    }
    if (!sw_i2c_write_byte(val)) {
        sw_i2c_stop();
        g_last_error = -3;
        return false;
    }
    sw_i2c_stop();
    g_last_error = 0;
    return true;
}

static bool sw_i2c_read_regs(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
    if ((buf == 0) || (len == 0U)) {
        g_last_error = -4;
        return false;
    }

    sw_i2c_start();
    if (!sw_i2c_write_byte((uint8_t) (addr << 1U))) {
        sw_i2c_stop();
        g_last_error = -5;
        return false;
    }
    if (!sw_i2c_write_byte(reg)) {
        sw_i2c_stop();
        g_last_error = -6;
        return false;
    }

    sw_i2c_start();
    if (!sw_i2c_write_byte((uint8_t) ((addr << 1U) | 1U))) {
        sw_i2c_stop();
        g_last_error = -7;
        return false;
    }

    for (uint8_t i = 0; i < len; i++) {
        buf[i] = sw_i2c_read_byte(i < (uint8_t) (len - 1U));
    }
    sw_i2c_stop();
    g_last_error = 0;
    return true;
}

static int16_t make_i16(uint8_t hi, uint8_t lo)
{
    return (int16_t) (((uint16_t) hi << 8U) | lo);
}

static uint8_t active_addr(void)
{
    if (g_ack68 != 0U) {
        return MPU_ADDR_68;
    }
    if (g_ack69 != 0U) {
        return MPU_ADDR_69;
    }
    return MPU_ADDR_68;
}

static bool configure_mpu(uint8_t addr)
{
    if (!sw_i2c_write_reg(addr, 0x6BU, 0x00U)) return false;
    delay_cycles(CPUCLK_FREQ / 50U);
    if (!sw_i2c_write_reg(addr, 0x19U, 0x07U)) return false;
    if (!sw_i2c_write_reg(addr, 0x1AU, 0x06U)) return false;
    if (!sw_i2c_write_reg(addr, 0x1BU, 0x00U)) return false;
    if (!sw_i2c_write_reg(addr, 0x1CU, 0x00U)) return false;
    return true;
}

static bool read_motion_raw(void)
{
    uint8_t buf[14];
    uint8_t addr;

    g_line_sda = read_sda() ? 1U : 0U;
    g_line_scl = read_scl() ? 1U : 0U;
    g_ack68 = sw_i2c_probe(MPU_ADDR_68) ? 1U : 0U;
    g_ack69 = sw_i2c_probe(MPU_ADDR_69) ? 1U : 0U;
    addr = active_addr();

    if ((g_ack68 == 0U) && (g_ack69 == 0U)) {
        g_read_ok = false;
        g_who = 0;
        g_last_error = -8;
        return false;
    }

    if (!g_mpu_configured) {
        g_mpu_configured = configure_mpu(addr);
        if (!g_mpu_configured) {
            g_read_ok = false;
            return false;
        }
    }

    (void) sw_i2c_read_regs(addr, 0x75U, (uint8_t *) &g_who, 1U);
    if (sw_i2c_read_regs(addr, 0x3BU, buf, 14U)) {
        g_ax = make_i16(buf[0], buf[1]);
        g_ay = make_i16(buf[2], buf[3]);
        g_az = make_i16(buf[4], buf[5]);
        g_gx = make_i16(buf[8], buf[9]);
        g_gy = make_i16(buf[10], buf[11]);
        g_gz = make_i16(buf[12], buf[13]);
        g_read_ok = true;
        return true;
    }

    g_read_ok = false;
    return false;
}

static void show_no_data(void)
{
    char line[24];

    OLED_Clear();
    OLED_ShowString(0, 0, "NO MPU DATA", 16, 1);
    (void) snprintf(line, sizeof(line), "SCL:%u SDA:%u", g_line_scl, g_line_sda);
    OLED_ShowString(0, 22, line, 8, 1);
    (void) snprintf(line, sizeof(line), "A68:%u A69:%u E:%d", g_ack68, g_ack69, g_last_error);
    OLED_ShowString(0, 34, line, 8, 1);
    OLED_ShowString(0, 50, "CHECK PA1 PA0", 8, 1);
    OLED_Refresh();
}

static void show_calibration(uint32_t done)
{
    char line[24];

    OLED_Clear();
    OLED_ShowString(0, 0, "GYRO CAL", 16, 1);
    OLED_ShowString(0, 22, "KEEP STILL", 8, 1);
    (void) snprintf(line, sizeof(line), "%lu/%u", (unsigned long) done, CALIBRATION_SAMPLES);
    OLED_ShowString(0, 36, line, 8, 1);
    (void) snprintf(line, sizeof(line), "GZ:%d", g_gz);
    OLED_ShowString(0, 50, line, 8, 1);
    OLED_Refresh();
}

static void calibrate_gyro_z(void)
{
    int32_t sum = 0;
    uint32_t count = 0;

    while (count < CALIBRATION_SAMPLES) {
        if (read_motion_raw()) {
            sum += g_gz;
            count++;
            if ((count % 20U) == 0U) {
                show_calibration(count);
            }
        } else {
            show_no_data();
        }
        delay_cycles(CPUCLK_FREQ / 100U);
    }

    g_gz_bias = (float) sum / (float) CALIBRATION_SAMPLES;
    g_yaw_deg = 0.0f;
    g_gz_dps = 0.0f;
    g_last_sample_ms = millis();
}

static void update_yaw_from_gz(void)
{
    uint32_t now = millis();
    uint32_t dt_ms = now - g_last_sample_ms;

    g_last_sample_ms = now;
    if ((dt_ms == 0U) || (dt_ms > 500U)) {
        return;
    }

    g_gz_dps = ((float) g_gz - g_gz_bias) / GYRO_LSB_PER_DPS;
    if ((g_gz_dps > -0.3f) && (g_gz_dps < 0.3f)) {
        g_gz_dps = 0.0f;
    }

    g_yaw_deg += g_gz_dps * ((float) dt_ms / 1000.0f);

    while (g_yaw_deg >= 360.0f) {
        g_yaw_deg -= 360.0f;
    }
    while (g_yaw_deg < 0.0f) {
        g_yaw_deg += 360.0f;
    }
}

static void gray_sensor_init(void)
{
    No_MCU_Ganv_Sensor_Init(&g_gray_sensor, g_gray_white, g_gray_black);
    No_Mcu_Ganv_Sensor_Task_Without_tick(&g_gray_sensor);
    g_gray_digital = Get_Digtal_For_User(&g_gray_sensor);
    (void) Get_Normalize_For_User(&g_gray_sensor, g_gray_normal);
    (void) Get_Anolog_Value(&g_gray_sensor, g_gray_analog);
    g_last_gray_ms = millis();
}

static void update_gray_sensor(void)
{
    No_Mcu_Ganv_Sensor_Task_Without_tick(&g_gray_sensor);
    g_gray_digital = Get_Digtal_For_User(&g_gray_sensor);
    (void) Get_Normalize_For_User(&g_gray_sensor, g_gray_normal);
    (void) Get_Anolog_Value(&g_gray_sensor, g_gray_analog);
}

static void format_gray_bits(char bits[9])
{
    for (uint8_t i = 0; i < 8U; i++) {
        bits[i] = ((g_gray_digital & (1U << i)) != 0U) ? '1' : '0';
    }
    bits[8] = '\0';
}

static void show_motion(void)
{
    char line[24];
    char bits[9];
    uint32_t yaw10;

    if (!g_read_ok) {
        show_no_data();
        return;
    }

    format_gray_bits(bits);
    yaw10 = (uint32_t) (g_yaw_deg * 10.0f + 0.5f);

    OLED_Clear();
    (void) snprintf(line, sizeof(line), "G:%s 1W0B", bits);
    OLED_ShowString(0, 0, line, 8, 1);
    (void) snprintf(line, sizeof(line), "A0:%04u A1:%04u", (unsigned int) g_gray_analog[0], (unsigned int) g_gray_analog[1]);
    OLED_ShowString(0, 10, line, 8, 1);
    (void) snprintf(line, sizeof(line), "A2:%04u A3:%04u", (unsigned int) g_gray_analog[2], (unsigned int) g_gray_analog[3]);
    OLED_ShowString(0, 20, line, 8, 1);
    (void) snprintf(line, sizeof(line), "A4:%04u A5:%04u", (unsigned int) g_gray_analog[4], (unsigned int) g_gray_analog[5]);
    OLED_ShowString(0, 30, line, 8, 1);
    (void) snprintf(line, sizeof(line), "A6:%04u A7:%04u", (unsigned int) g_gray_analog[6], (unsigned int) g_gray_analog[7]);
    OLED_ShowString(0, 40, line, 8, 1);
    (void) snprintf(line, sizeof(line), "YAW:%03lu.%lu", (unsigned long) (yaw10 / 10U), (unsigned long) (yaw10 % 10U));
    OLED_ShowString(0, 54, line, 8, 1);
    OLED_Refresh();
}

int main(void)
{
    SYSCFG_DL_init();
    systick_init_1ms();

    OLED_Init();
    OLED_ColorTurn(0);
    OLED_DisplayTurn(0);
    OLED_Clear();
    OLED_ShowString(0, 8, "MPU+GRAY", 16, 1);
    OLED_ShowString(0, 36, "INIT", 8, 1);
    OLED_Refresh();

    gray_sensor_init();
    sw_i2c_gpio_init();
    sw_i2c_recover();
    delay_cycles(CPUCLK_FREQ / 2U);
    calibrate_gyro_z();

    g_last_oled_ms = millis();

    while (1) {
        uint32_t now = millis();

        if (read_motion_raw()) {
            update_yaw_from_gz();
        }

        if ((now - g_last_gray_ms) >= GRAY_SAMPLE_PERIOD_MS) {
            g_last_gray_ms = now;
            update_gray_sensor();
        }

        if ((now - g_last_oled_ms) >= OLED_REFRESH_PERIOD_MS) {
            g_last_oled_ms = now;
            show_motion();
        }

        delay_cycles(CPUCLK_FREQ / 100U);
    }
}


