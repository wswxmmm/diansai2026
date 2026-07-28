#ifndef MULTI_MOTOR_CONFIG_H
#define MULTI_MOTOR_CONFIG_H

#define MOTOR_COUNT                 3U

/*
 * The encoder loop is closed inside each Emm driver.  The present shared-TX
 * TTL wiring does not return driver position/status to the MSPM0, so a true
 * application-level "arrived" check is not available yet.
 */
#define MOTOR_DRIVER_INTERNAL_CLOSED_LOOP  1U
#define MOTOR_DRIVER_FEEDBACK_WIRED        0U
#define MOTOR_DRIVER_UART_BAUD             115200U

#define MOTOR_1_ADDRESS             1U
#define MOTOR_2_ADDRESS             2U
#define MOTOR_3_ADDRESS             3U

/* Set an XOR value to 1 when that motor is mechanically mirrored. */
#define MOTOR_1_DIRECTION_XOR       1U
#define MOTOR_2_DIRECTION_XOR       1U
#define MOTOR_3_DIRECTION_XOR       0U

#define MOTOR_TEST_SPEED_RPM       10U
/* Emm acceleration parameter range: 0..255; 0 means immediate start. */
#define MOTOR_TEST_ACCEL            5U

/* 1.8-degree motor and the 16-microstep setting found in this project. */
#define MOTOR_FULL_STEPS_PER_REV    200U
#define MOTOR_MICROSTEPS            16U
#define MOTOR_PULSES_PER_REV        \
    (MOTOR_FULL_STEPS_PER_REV * MOTOR_MICROSTEPS)
#define MOTOR_TEST_OUTPUT_DEG       3U
/* 5.625:1 reduction gives 50 motor-command pulses per output degree. */
#define MOTOR_TEST_POSITION_PULSES  \
    (MOTOR_TEST_OUTPUT_DEG * 50U)
#define MOTOR_POSITION_STOP_MS      3000U

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

#if ((MOTOR_TEST_OUTPUT_DEG == 0U) || (MOTOR_TEST_OUTPUT_DEG > 5U))
#error "Safe test output angle must be in the range 1..5 degrees"
#endif

#if (MOTOR_TEST_SPEED_RPM > 5000U)
#error "Emm speed must not exceed 5000 RPM"
#endif

#if (MOTOR_TEST_ACCEL > 255U)
#error "Emm acceleration parameter must be in the range 0..255"
#endif

#endif
