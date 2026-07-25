#include "ti_msp_dl_config.h"
#include "bsp_jy61p.h"
#include "oled.h"
#include <stdint.h>
#include <stdio.h>

#define DISPLAY_PERIOD_MS       (100U)

static void format_angle(char *text, size_t size, const char *label,
    int16_t angle_cdeg)
{
    uint32_t magnitude;
    char sign = '+';

    if (angle_cdeg < 0) {
        sign = '-';
        magnitude = (uint32_t) (-(int32_t) angle_cdeg);
    } else {
        magnitude = (uint32_t) angle_cdeg;
    }
    (void) snprintf(text, size, "%s:%c%03lu.%02lu", label, sign,
        (unsigned long) (magnitude / 100U),
        (unsigned long) (magnitude % 100U));
}

static void show_i2c_error(const JY61P_Attitude *attitude)
{
    char line[24];

    OLED_Clear();
    OLED_DrawRectangle(0, 0, 127, 63, 1);
    OLED_ShowString(27, 3, "WAIT JY61P", 8, 1);
    OLED_ShowString(21, 16, "I2C ADDR:50", 8, 1);
    (void) snprintf(line, sizeof(line), "READ:%lu ERR:%lu",
        (unsigned long) (attitude->read_count % 10000U),
        (unsigned long) (attitude->read_errors % 10000U));
    OLED_ShowString(13, 29, line, 8, 1);
    (void) snprintf(line, sizeof(line), "ERROR CODE:%u", attitude->last_error);
    OLED_ShowString(19, 42, line, 8, 1);
    OLED_ShowString(19, 53, "SCL:PA1 SDA:PA0", 8, 1);
    OLED_Refresh();
}

static void show_attitude(const JY61P_Attitude *attitude)
{
    char line[24];

    OLED_Clear();
    OLED_DrawRectangle(0, 0, 127, 63, 1);
    OLED_ShowString(25, 3, "JY61P ANGLE", 8, 1);
    format_angle(line, sizeof(line), "ROLL", attitude->roll_cdeg);
    OLED_ShowString(13, 16, line, 8, 1);
    format_angle(line, sizeof(line), "PITCH", attitude->pitch_cdeg);
    OLED_ShowString(7, 29, line, 8, 1);
    format_angle(line, sizeof(line), "YAW", attitude->yaw_cdeg);
    OLED_ShowString(19, 42, line, 8, 1);
    (void) snprintf(line, sizeof(line), "I2C OK  N:%lu",
        (unsigned long) (attitude->read_count % 100000U));
    OLED_ShowString(19, 53, line, 8, 1);
    OLED_Refresh();
}

int main(void)
{
    JY61P_Attitude attitude = {0};

    SYSCFG_DL_init();
    OLED_Init();
    OLED_ColorTurn(0);
    OLED_DisplayTurn(0);
    JY61P_Init();
    show_i2c_error(&attitude);

    while (1) {
        if (JY61P_ReadAttitude(&attitude)) {
            show_attitude(&attitude);
        } else {
            show_i2c_error(&attitude);
        }
        delay_cycles((CPUCLK_FREQ / 1000U) * DISPLAY_PERIOD_MS);
    }
}
