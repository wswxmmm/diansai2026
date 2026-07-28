#ifndef ROBOT_ARM_CONFIG_H
#define ROBOT_ARM_CONFIG_H

#include "multi_motor_config.h"

/*
 * Robot-arm geometry and temporary calibration defaults.
 *
 * Values tagged CAD come from robot_arm.step.  Values tagged DEFAULT are
 * placeholders that must be replaced by measurement/calibration before an
 * unattended pickup run.
 */

/* CAD geometry, millimetres. */
#define ARM_CAD_SHOULDER_Z_MM              62.500f
#define ARM_CAD_BASE_GROUND_Z_MM          (-67.700f)
/* Control coordinates are rebased so the base/table plane is Z=0. */
#define ARM_BASE_TO_SHOULDER_Z_MM         130.200f
#define ARM_LINK_LOWER_MM                 120.000f
#define ARM_LINK_UPPER_MM                 120.000f
#define ARM_PARALLELOGRAM_SHORT_MM         40.000f
#define ARM_WRIST_TO_TOOL_AXIS_MM          53.000f

/* DEFAULT: magnet contact face below the standard vertical tool mount. */
#define ARM_MAGNET_TOOL_LENGTH_MM          30.000f

/*
 * Confirmed GT2 transmission on all three axes: 16-tooth motor pulley drives
 * the 90-tooth printed output pulley.  The reduction value is motor turns per
 * one mechanism-output turn.
 */
#define ARM_MOTOR_PULLEY_TEETH              16U
#define ARM_OUTPUT_PULLEY_TEETH             90U
#define ARM_MOTOR_1_REDUCTION                5.625f
#define ARM_MOTOR_2_REDUCTION                5.625f
#define ARM_MOTOR_3_REDUCTION                5.625f
#define ARM_OUTPUT_PULSES_PER_REV            \
    ((MOTOR_PULSES_PER_REV * ARM_OUTPUT_PULLEY_TEETH) / \
     ARM_MOTOR_PULLEY_TEETH)
#define ARM_OUTPUT_PULSES_PER_DEG            \
    ((float)ARM_OUTPUT_PULSES_PER_REV / 360.0f)

/* Positive joint rotation convention confirmed by the user. */
#define ARM_POSITIVE_ROTATION_IS_CCW         1U

/* Physical address-to-axis mapping confirmed on the assembled robot. */
#define ARM_BASE_MOTOR_INDEX                0U
#define ARM_LOWER_LINK_MOTOR_INDEX          1U
#define ARM_UPPER_LINK_MOTOR_INDEX          2U

/*
 * User-confirmed manual startup pose: lower link vertical upward and upper
 * link horizontal outward.  These are not encoder homing results.
 */
#define ARM_START_BASE_DEG                   0.000f
#define ARM_START_LOWER_LINK_DEG            90.000f
#define ARM_START_UPPER_LINK_DEG             0.000f

/*
 * DEFAULT broad software ranges.  Limit enforcement remains disabled until
 * the physical stops and cable-safe range have been measured.
 */
#define ARM_LIMITS_ENABLED                   0U
#define ARM_BASE_MIN_DEG                  (-180.000f)
#define ARM_BASE_MAX_DEG                   180.000f
#define ARM_LOWER_LINK_MIN_DEG               0.000f
#define ARM_LOWER_LINK_MAX_DEG             180.000f
#define ARM_UPPER_LINK_MIN_DEG             (-90.000f)
#define ARM_UPPER_LINK_MAX_DEG             180.000f

/* DEFAULT pickup trajectory clearances, millimetres. */
#define ARM_PICK_APPROACH_HEIGHT_MM         40.000f
#define ARM_PICK_CONTACT_EXTRA_MM            2.000f
#define ARM_PICK_LIFT_HEIGHT_MM             50.000f

/*
 * DEFAULT camera-to-base transform.  Identity means camera coordinates are
 * temporarily treated as base coordinates; do not use it for a real pickup.
 */
#define ARM_CAMERA_X_MM                      0.000f
#define ARM_CAMERA_Y_MM                      0.000f
#define ARM_CAMERA_Z_MM                      0.000f
#define ARM_CAMERA_ROLL_DEG                  0.000f
#define ARM_CAMERA_PITCH_DEG                 0.000f
#define ARM_CAMERA_YAW_DEG                   0.000f

/* DEFAULT steel-ball contact point in the robot-base coordinate system. */
#define ARM_DEFAULT_TARGET_X_MM            200.000f
#define ARM_DEFAULT_TARGET_Y_MM              0.000f
#define ARM_DEFAULT_TARGET_Z_MM              0.000f

/* No homing switches: each button press advances at most this many degrees. */
#define ARM_IK_TEST_MAX_DELTA_DEG             5.000f

/*
 * Eye-in-hand visual-servo defaults.  The camera is assumed to point down at
 * the work plane.  Image X is treated as tangential motion and image Y as
 * radial motion relative to the base angle.  Change only the two signs below
 * if a test image moves in the opposite direction.
 */
#define ARM_VISION_IMAGE_X_SIGN                1.000f
#define ARM_VISION_IMAGE_Y_SIGN                1.000f
#define ARM_VISION_PIXEL_DEADBAND_PX           18
#define ARM_VISION_RADIUS_TARGET_PX            65
#define ARM_VISION_RADIUS_DEADBAND_PX            5
#define ARM_VISION_MM_PER_PIXEL                 0.080f
#define ARM_VISION_MAX_XY_STEP_MM               2.000f
#define ARM_VISION_DESCEND_STEP_MM              2.000f
#define ARM_VISION_MAX_JOINT_STEP_DEG           1.000f

/* Conservative software workspace used by visual tracking. */
#define ARM_VISION_MIN_RADIUS_MM               80.000f
#define ARM_VISION_MAX_RADIUS_MM              293.200f
#define ARM_VISION_MIN_TOOL_Z_MM               10.000f
#define ARM_VISION_MAX_TOOL_Z_MM              221.000f

/*
 * Pose-free visual calibration uses only relative joint jogs.  Every command
 * is deliberately tiny, and the accumulated displacement from the manually
 * chosen observation pose is bounded independently on all three axes.
 */
#define ARM_RELATIVE_JOG_MAX_MDEG              2000
#define ARM_RELATIVE_TOTAL_MAX_MDEG           25000

/* 10 motor RPM through 5.625:1 is about 10.67 output degrees/s. */
#define ARM_MOTION_MS_PER_OUTPUT_DEG           94.0f
#define ARM_MOTION_SETTLE_MS                   180U

#endif
