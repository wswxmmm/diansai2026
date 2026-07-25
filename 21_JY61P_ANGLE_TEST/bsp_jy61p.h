#ifndef BSP_JY61P_H
#define BSP_JY61P_H

#include <stdbool.h>
#include <stdint.h>

#define JY61P_I2C_ADDRESS       (0x50U)
#define JY61P_ANGLE_REGISTER    (0x3DU)

typedef struct {
    int16_t roll_cdeg;
    int16_t pitch_cdeg;
    int16_t yaw_cdeg;
    uint32_t read_count;
    uint32_t read_errors;
    uint8_t last_error;
} JY61P_Attitude;

void JY61P_Init(void);
bool JY61P_ReadAttitude(JY61P_Attitude *attitude);

#endif
