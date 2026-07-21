#ifndef MOTOR_DRIVER_CONFIG_H
#define MOTOR_DRIVER_CONFIG_H

/* Select the firmware protocol used inside the external motor driver. */
#define MOTOR_PROTOCOL_X       1
#define MOTOR_PROTOCOL_EMM     2

#ifndef MOTOR_PROTOCOL
#define MOTOR_PROTOCOL         MOTOR_PROTOCOL_X
#endif

/* Separate UART ports allow both drivers to keep their default address 1. */
#define MOTOR_1_ADDRESS            1U
#define MOTOR_2_ADDRESS            1U

/* Change to 1 when motor 2 is installed as a mirrored mechanical axis. */
#define MOTOR_2_REVERSE_DIRECTION  0U

#if ((MOTOR_2_REVERSE_DIRECTION != 0U) && \
     (MOTOR_2_REVERSE_DIRECTION != 1U))
#error "MOTOR_2_REVERSE_DIRECTION must be 0 or 1"
#endif

/* Test parameters used by the push-button mode selector. */
#define MOTOR_TEST_SPEED_RPM       100U

#if (MOTOR_PROTOCOL == MOTOR_PROTOCOL_X)
#define MOTOR_TEST_ACCEL           1000U  /* X: RPM/s */
#else
#define MOTOR_TEST_ACCEL           10U    /* Emm: 0-255 acceleration level */
#endif

/* 1.8-degree motor at 16 microsteps: 200 * 16 = 3200 pulses/revolution. */
#define MOTOR_PULSES_PER_REV       3200U
#define MOTOR_POSITION_DEGREES     90.0f

/* Safety stop after a 90-degree move; 100 RPM normally needs about 150 ms. */
#define MOTOR_POSITION_STOP_MS     1000U

#endif
