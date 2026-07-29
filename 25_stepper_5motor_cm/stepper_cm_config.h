#ifndef STEPPER_CM_CONFIG_H
#define STEPPER_CM_CONFIG_H

/* Emm driver addresses configured on the five motor driver boards. */
#define STEPPER_MOTOR_1_ADDRESS          1U
#define STEPPER_MOTOR_2_ADDRESS          2U
#define STEPPER_MOTOR_3_ADDRESS          3U
#define STEPPER_MOTOR_4_ADDRESS          4U
#define STEPPER_MOTOR_5_ADDRESS          5U

/*
 * Set a direction XOR to 1 only if that motor's mechanical positive direction
 * is opposite to the other motors. Motors 2 and 3 currently use the same
 * direction because they drive the same Y axis without mirroring.
 */
#define STEPPER_MOTOR_1_DIRECTION_XOR    0U
#define STEPPER_MOTOR_2_DIRECTION_XOR    0U
#define STEPPER_MOTOR_3_DIRECTION_XOR    0U
#define STEPPER_MOTOR_4_DIRECTION_XOR    0U
#define STEPPER_MOTOR_5_DIRECTION_XOR    0U

/* Current driver subdivision: 200 full steps/rev * 16 = 3200 pulses/rev. */
#define STEPPER_PULSES_PER_REV           3200U

/* Measured travel: 10 revolutions = 40 mm, therefore lead = 4 mm/rev. */
#define STEPPER_SLIDE_LEAD_MM            4U

#define STEPPER_DEFAULT_SPEED_RPM        300U
#define STEPPER_DEFAULT_ACCELERATION     10U

/* Prevent an accidental command longer than the physical slide. */
#define STEPPER_MAX_ABS_DISTANCE_CM      200.0F

#if ((STEPPER_DEFAULT_SPEED_RPM == 0U) || \
     (STEPPER_DEFAULT_SPEED_RPM > 5000U))
#error "STEPPER_DEFAULT_SPEED_RPM must be in the range 1..5000"
#endif

#if (STEPPER_DEFAULT_ACCELERATION > 255U)
#error "STEPPER_DEFAULT_ACCELERATION must be in the range 0..255"
#endif

#if ((STEPPER_PULSES_PER_REV == 0U) || (STEPPER_SLIDE_LEAD_MM == 0U))
#error "Stepper pulse and slide lead settings must be greater than zero"
#endif

#endif
