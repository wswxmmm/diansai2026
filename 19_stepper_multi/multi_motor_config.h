#ifndef MULTI_MOTOR_CONFIG_H
#define MULTI_MOTOR_CONFIG_H

#define MOTOR_COUNT                 3U

#define MOTOR_1_ADDRESS             1U
#define MOTOR_2_ADDRESS             2U
#define MOTOR_3_ADDRESS             3U

/* Set an XOR value to 1 when that motor is mechanically mirrored. */
#define MOTOR_1_DIRECTION_XOR       0U
#define MOTOR_2_DIRECTION_XOR       0U
#define MOTOR_3_DIRECTION_XOR       1U

#define MOTOR_TEST_SPEED_RPM        100U
/* Emm acceleration parameter range: 0..255; 0 means immediate start. */
#define MOTOR_TEST_ACCEL            10U

/* 16 microsteps: 200 full steps/rev * 16 = 3200 pulses/rev. */
#define MOTOR_PULSES_PER_REV        3200U
#define MOTOR_TEST_POSITION_DEG     36U
#define MOTOR_TEST_POSITION_PULSES  \
    ((MOTOR_PULSES_PER_REV * MOTOR_TEST_POSITION_DEG) / 360U)
#define MOTOR_POSITION_STOP_MS      1000U

#if ((MOTOR_1_ADDRESS == 0U) || (MOTOR_2_ADDRESS == 0U) || \
     (MOTOR_3_ADDRESS == 0U))
#error "Motor addresses 1, 2 and 3 must be non-zero"
#endif

#if ((MOTOR_1_ADDRESS == MOTOR_2_ADDRESS) || \
     (MOTOR_1_ADDRESS == MOTOR_3_ADDRESS) || \
     (MOTOR_2_ADDRESS == MOTOR_3_ADDRESS))
#error "Each motor on the shared Emm bus must have a unique address"
#endif

#if ((MOTOR_1_DIRECTION_XOR > 1U) || (MOTOR_2_DIRECTION_XOR > 1U) || \
     (MOTOR_3_DIRECTION_XOR > 1U))
#error "Direction XOR values must be 0 or 1"
#endif

#if ((MOTOR_TEST_POSITION_DEG == 0U) || (MOTOR_TEST_POSITION_DEG > 360U))
#error "Test position angle must be in the range 1..360 degrees"
#endif

#if (((MOTOR_PULSES_PER_REV * MOTOR_TEST_POSITION_DEG) % 360U) != 0U)
#error "Test angle must map to a whole number of Emm pulses"
#endif

#if (MOTOR_TEST_SPEED_RPM > 5000U)
#error "Emm speed must not exceed 5000 RPM"
#endif

#if (MOTOR_TEST_ACCEL > 255U)
#error "Emm acceleration parameter must be in the range 0..255"
#endif

#endif
