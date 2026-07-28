#include "ti_msp_dl_config.h"
#include "multi_motor_config.h"
#include "robot_arm_config.h"
#include "robot_arm_kinematics.h"
#include "emm_multi_motor.h"
#include "rpi_uart_link.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define CPU_CLOCK_HZ 32000000U
#define ARM_DEG_TO_RAD 0.017453292519943295f

static const uint8_t g_motorAddresses[MOTOR_COUNT] = {
    MOTOR_1_ADDRESS, MOTOR_2_ADDRESS, MOTOR_3_ADDRESS
};

static const uint8_t g_motorDirectionXor[MOTOR_COUNT] = {
    MOTOR_1_DIRECTION_XOR,
    MOTOR_2_DIRECTION_XOR,
    MOTOR_3_DIRECTION_XOR
};

volatile uint8_t g_multiMotorMode;
volatile bool g_multiLastOperationOk;
volatile uint8_t g_ikStatus;
volatile bool g_ikAtDefaultTarget;
volatile float g_ikTargetX;
volatile float g_ikTargetY;
volatile float g_ikTargetZ;
volatile float g_ikSolvedBaseDeg;
volatile float g_ikSolvedLowerDeg;
volatile float g_ikSolvedUpperDeg;
volatile uint32_t g_ikAxisPulses[MOTOR_COUNT];
volatile bool g_visionTrackingEnabled;
volatile bool g_visionPoseKnown;
volatile uint32_t g_visionLastSequence;
volatile bool g_relativeVisualMode;
volatile int32_t g_relativeJointTotalMdeg[MOTOR_COUNT];
volatile uint32_t g_relativeLastSequence;

static RobotArmJointAngles g_currentJointAngles;

static void app_delay_ms(uint32_t milliseconds)
{
    while (milliseconds-- > 0U) {
        delay_cycles(CPU_CLOCK_HZ / 1000U);
    }
}

static float clamp_float(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static int32_t round_tenths(float value)
{
    float scaled = value * 10.0f;
    return (int32_t)(scaled + ((scaled >= 0.0f) ? 0.5f : -0.5f));
}

static uint8_t motor_direction(uint8_t motorIndex, uint8_t direction)
{
    return (uint8_t)(direction ^ g_motorDirectionXor[motorIndex]);
}

static void app_blink_motor(uint8_t motorNumber)
{
    uint8_t count = motorNumber;

    DL_GPIO_clearPins(LED1_PORT, LED1_PIN_22_PIN);
    while (count-- > 0U) {
        DL_GPIO_setPins(LED1_PORT, LED1_PIN_22_PIN);
        app_delay_ms(120U);
        DL_GPIO_clearPins(LED1_PORT, LED1_PIN_22_PIN);
        app_delay_ms(120U);
    }
}

static bool app_enable_all(void)
{
    uint8_t i;
    bool ok = true;

    for (i = 0U; i < MOTOR_COUNT; i++) {
        ok = EmmMulti_Enable(g_motorAddresses[i], true) && ok;
        app_delay_ms(20U);
    }
    return ok;
}

static bool app_sync_stop_all(void)
{
    uint8_t i;
    bool ok = true;

    for (i = 0U; i < MOTOR_COUNT; i++) {
        ok = EmmMulti_QueueStop(g_motorAddresses[i]) && ok;
        app_delay_ms(10U);
    }
    return EmmMulti_TriggerSynchronous() && ok;
}

static bool app_wait_for_motion(uint32_t milliseconds)
{
    while (milliseconds-- > 0U) {
        if (RpiLink_TakeEmergencyStop()) {
            g_visionTrackingEnabled = false;
            g_relativeVisualMode = false;
            g_visionPoseKnown = false;
            app_sync_stop_all();
            RpiLink_SendString("STOPPED EMERGENCY POSE_UNKNOWN\r\n");
            return false;
        }
        app_delay_ms(1U);
    }
    return true;
}

static bool app_queue_axis_delta(uint8_t motorIndex, float deltaDeg)
{
    float magnitude = fabsf(deltaDeg);
    uint32_t pulses;
    uint8_t logicalDirection;

    if (motorIndex >= MOTOR_COUNT) {
        return false;
    }
    if (magnitude < 0.01f) {
        g_ikAxisPulses[motorIndex] = 0U;
        return true;
    }
    if (magnitude > (ARM_IK_TEST_MAX_DELTA_DEG + 0.01f)) {
        return false;
    }

    pulses = (uint32_t)(magnitude * ARM_OUTPUT_PULSES_PER_DEG + 0.5f);
    logicalDirection = (deltaDeg >= 0.0f) ? 0U : 1U;
    g_ikAxisPulses[motorIndex] = pulses;

    return EmmMulti_QueuePosition(
        g_motorAddresses[motorIndex],
        motor_direction(motorIndex, logicalDirection),
        MOTOR_TEST_SPEED_RPM, MOTOR_TEST_ACCEL, pulses);
}

static bool app_move_to_joint_angles(const RobotArmJointAngles *target)
{
    float deltas[MOTOR_COUNT];
    float maximum;
    uint32_t waitMs;
    uint8_t i;
    bool ok;
    bool hasMotion = false;

    deltas[ARM_BASE_MOTOR_INDEX] = RobotArm_WrappedDeltaDeg(
        target->baseDeg, g_currentJointAngles.baseDeg);
    deltas[ARM_LOWER_LINK_MOTOR_INDEX] = RobotArm_WrappedDeltaDeg(
        target->lowerDeg, g_currentJointAngles.lowerDeg);
    deltas[ARM_UPPER_LINK_MOTOR_INDEX] = RobotArm_WrappedDeltaDeg(
        target->upperDeg, g_currentJointAngles.upperDeg);

    maximum = 0.0f;
    for (i = 0U; i < MOTOR_COUNT; i++) {
        if (fabsf(deltas[i]) > maximum) {
            maximum = fabsf(deltas[i]);
        }
        if (fabsf(deltas[i]) > (ARM_IK_TEST_MAX_DELTA_DEG + 0.01f)) {
            g_ikStatus = 2U;
            return false;
        }
    }

    ok = app_sync_stop_all();
    app_delay_ms(80U);

    for (i = 0U; i < MOTOR_COUNT; i++) {
        if (fabsf(deltas[i]) >= 0.01f) {
            ok = app_queue_axis_delta(i, deltas[i]) && ok;
            hasMotion = true;
            app_delay_ms(10U);
        } else {
            g_ikAxisPulses[i] = 0U;
        }
    }

    if (hasMotion) {
        ok = EmmMulti_TriggerSynchronous() && ok;
        waitMs = (uint32_t)(maximum * ARM_MOTION_MS_PER_OUTPUT_DEG) +
                 ARM_MOTION_SETTLE_MS;
        if (!app_wait_for_motion(waitMs)) {
            return false;
        }
        ok = app_sync_stop_all() && ok;
    }

    if (ok) {
        g_currentJointAngles = *target;
    } else if (hasMotion) {
        g_visionPoseKnown = false;
        app_sync_stop_all();
    }
    return ok;
}

static bool app_step_toward_joint_angles(
    const RobotArmJointAngles *goal, float maximumStepDeg, bool *reached)
{
    RobotArmJointAngles stepTarget;
    float baseDelta = RobotArm_WrappedDeltaDeg(
        goal->baseDeg, g_currentJointAngles.baseDeg);
    float lowerDelta = RobotArm_WrappedDeltaDeg(
        goal->lowerDeg, g_currentJointAngles.lowerDeg);
    float upperDelta = RobotArm_WrappedDeltaDeg(
        goal->upperDeg, g_currentJointAngles.upperDeg);
    float maximum = fabsf(baseDelta);
    float scale = 1.0f;

    if (fabsf(lowerDelta) > maximum) {
        maximum = fabsf(lowerDelta);
    }
    if (fabsf(upperDelta) > maximum) {
        maximum = fabsf(upperDelta);
    }

    if (maximum < 0.01f) {
        *reached = true;
        g_ikAxisPulses[0] = 0U;
        g_ikAxisPulses[1] = 0U;
        g_ikAxisPulses[2] = 0U;
        return true;
    }

    if (maximum > maximumStepDeg) {
        scale = maximumStepDeg / maximum;
    }

    stepTarget.baseDeg = g_currentJointAngles.baseDeg + baseDelta * scale;
    stepTarget.lowerDeg = g_currentJointAngles.lowerDeg + lowerDelta * scale;
    stepTarget.upperDeg = g_currentJointAngles.upperDeg + upperDelta * scale;
    *reached = (scale >= 1.0f);
    return app_move_to_joint_angles(&stepTarget);
}

static void app_send_pose(const char *prefix)
{
    float x;
    float y;
    float z;

    if (!RobotArm_ForwardTarget(&g_currentJointAngles, &x, &y, &z)) {
        RpiLink_SendString("ERR FK\r\n");
        return;
    }
    RpiLink_SendString(prefix);
    RpiLink_SendString(" ");
    RpiLink_SendSigned(round_tenths(x));
    RpiLink_SendString(" ");
    RpiLink_SendSigned(round_tenths(y));
    RpiLink_SendString(" ");
    RpiLink_SendSigned(round_tenths(z));
    RpiLink_SendString("\r\n");
}

static bool parse_unsigned(const char **cursor, uint32_t *value)
{
    const char *text = *cursor;
    uint32_t result = 0U;
    bool found = false;

    while (*text == ' ') {
        text++;
    }
    while ((*text >= '0') && (*text <= '9')) {
        uint32_t digit = (uint32_t)(*text - '0');
        found = true;
        if ((result > 429496729U) ||
            ((result == 429496729U) && (digit > 5U))) {
            return false;
        }
        result = result * 10U + digit;
        text++;
    }
    if (!found) {
        return false;
    }
    *cursor = text;
    *value = result;
    return true;
}

static bool parse_ball(const char *text, uint32_t values[6])
{
    uint8_t i;

    for (i = 0U; i < 6U; i++) {
        if (!parse_unsigned(&text, &values[i])) {
            return false;
        }
    }
    while (*text == ' ') {
        text++;
    }
    return (*text == '\0');
}

static bool parse_signed(const char **cursor, int32_t *value)
{
    const char *text = *cursor;
    uint32_t magnitude;
    bool negative = false;

    while (*text == ' ') {
        text++;
    }
    if ((*text == '-') || (*text == '+')) {
        negative = (*text == '-');
        text++;
    }
    if (!parse_unsigned(&text, &magnitude) || (magnitude > 2147483647U)) {
        return false;
    }
    *cursor = text;
    *value = negative ? -(int32_t)magnitude : (int32_t)magnitude;
    return true;
}

static bool parse_jog(const char *text, uint32_t *sequence,
                      int32_t deltasMdeg[MOTOR_COUNT])
{
    uint8_t i;

    if (!parse_unsigned(&text, sequence)) {
        return false;
    }
    for (i = 0U; i < MOTOR_COUNT; i++) {
        if (!parse_signed(&text, &deltasMdeg[i])) {
            return false;
        }
    }
    while (*text == ' ') {
        text++;
    }
    return (*text == '\0');
}

static bool app_move_relative_joints(const int32_t deltasMdeg[MOTOR_COUNT])
{
    float maximumDeg = 0.0f;
    uint32_t waitMs;
    uint8_t i;
    bool ok;
    bool hasMotion = false;

    for (i = 0U; i < MOTOR_COUNT; i++) {
        int32_t magnitude = (deltasMdeg[i] < 0) ?
                            -deltasMdeg[i] : deltasMdeg[i];
        int32_t future = g_relativeJointTotalMdeg[i] + deltasMdeg[i];
        int32_t futureMagnitude = (future < 0) ? -future : future;
        float deltaDeg = (float)deltasMdeg[i] / 1000.0f;

        if ((magnitude > ARM_RELATIVE_JOG_MAX_MDEG) ||
            (futureMagnitude > ARM_RELATIVE_TOTAL_MAX_MDEG)) {
            return false;
        }
        if (fabsf(deltaDeg) > maximumDeg) {
            maximumDeg = fabsf(deltaDeg);
        }
    }

    ok = app_sync_stop_all();
    app_delay_ms(80U);
    for (i = 0U; i < MOTOR_COUNT; i++) {
        float deltaDeg = (float)deltasMdeg[i] / 1000.0f;
        if (fabsf(deltaDeg) >= 0.01f) {
            ok = app_queue_axis_delta(i, deltaDeg) && ok;
            hasMotion = true;
            app_delay_ms(10U);
        } else {
            g_ikAxisPulses[i] = 0U;
        }
    }

    if (hasMotion) {
        ok = EmmMulti_TriggerSynchronous() && ok;
        waitMs = (uint32_t)(maximumDeg * ARM_MOTION_MS_PER_OUTPUT_DEG) +
                 ARM_MOTION_SETTLE_MS;
        if (!app_wait_for_motion(waitMs)) {
            return false;
        }
        ok = app_sync_stop_all() && ok;
    }

    if (ok) {
        for (i = 0U; i < MOTOR_COUNT; i++) {
            g_relativeJointTotalMdeg[i] += deltasMdeg[i];
        }
    } else if (hasMotion) {
        g_relativeVisualMode = false;
        app_sync_stop_all();
    }
    return ok;
}

static void app_send_relative_totals(uint32_t sequence)
{
    uint8_t i;

    RpiLink_SendString("JOG DONE ");
    RpiLink_SendUnsigned(sequence);
    for (i = 0U; i < MOTOR_COUNT; i++) {
        RpiLink_SendString(" ");
        RpiLink_SendSigned(g_relativeJointTotalMdeg[i]);
    }
    RpiLink_SendString("\r\n");
}

static void app_visual_servo_ball(const uint32_t values[6])
{
    uint32_t sequence = values[0];
    int32_t centerX;
    int32_t centerY;
    int32_t errorX;
    int32_t errorY;
    float x;
    float y;
    float z;
    float currentRadius;
    float newRadius;
    float baseRad;
    float newBaseRad;
    float tangentialMm = 0.0f;
    float radialMm = 0.0f;
    RobotArmJointAngles solution;
    RobotArmIkStatus ikResult;
    bool centeredX;
    bool centeredY;
    bool reached;
    bool ok;

    if (!g_visionTrackingEnabled) {
        RpiLink_SendString("BALL IDLE\r\n");
        return;
    }
    if (!g_visionPoseKnown) {
        RpiLink_SendString("ERR POSE_UNKNOWN\r\n");
        return;
    }
    if ((values[4] < 64U) || (values[5] < 64U) ||
        (values[4] > 4096U) || (values[5] > 4096U) ||
        (values[1] >= values[4]) || (values[2] >= values[5]) ||
        (values[3] == 0U) || (values[3] > values[4])) {
        RpiLink_SendString("ERR BALL_RANGE\r\n");
        return;
    }
    if (sequence == g_visionLastSequence) {
        RpiLink_SendString("BALL DUPLICATE\r\n");
        return;
    }
    g_visionLastSequence = sequence;

    centerX = (int32_t)(values[4] / 2U);
    centerY = (int32_t)(values[5] / 2U);
    errorX = (int32_t)values[1] - centerX;
    errorY = (int32_t)values[2] - centerY;
    centeredX = (errorX >= -ARM_VISION_PIXEL_DEADBAND_PX) &&
                (errorX <= ARM_VISION_PIXEL_DEADBAND_PX);
    centeredY = (errorY >= -ARM_VISION_PIXEL_DEADBAND_PX) &&
                (errorY <= ARM_VISION_PIXEL_DEADBAND_PX);

    if (centeredX && centeredY &&
        (values[3] + ARM_VISION_RADIUS_DEADBAND_PX >=
         ARM_VISION_RADIUS_TARGET_PX)) {
        g_visionTrackingEnabled = false;
        app_sync_stop_all();
        DL_GPIO_setPins(LED1_PORT, LED1_PIN_22_PIN);
        RpiLink_SendString("BALL READY ");
        RpiLink_SendUnsigned(sequence);
        RpiLink_SendString(" TRACK_OFF\r\n");
        return;
    }

    if (!RobotArm_ForwardTarget(&g_currentJointAngles, &x, &y, &z)) {
        RpiLink_SendString("ERR FK\r\n");
        return;
    }

    if (!centeredX || !centeredY) {
        if (!centeredX) {
            tangentialMm = clamp_float(
                (float)errorX * ARM_VISION_MM_PER_PIXEL *
                ARM_VISION_IMAGE_X_SIGN,
                -ARM_VISION_MAX_XY_STEP_MM, ARM_VISION_MAX_XY_STEP_MM);
        }
        if (!centeredY) {
            radialMm = clamp_float(
                (float)errorY * ARM_VISION_MM_PER_PIXEL *
                ARM_VISION_IMAGE_Y_SIGN,
                -ARM_VISION_MAX_XY_STEP_MM, ARM_VISION_MAX_XY_STEP_MM);
        }
    } else {
        z -= ARM_VISION_DESCEND_STEP_MM;
    }

    currentRadius = sqrtf(x * x + y * y);
    newRadius = currentRadius + radialMm;
    if (currentRadius < 1.0f) {
        RpiLink_SendString("ERR BASE_SINGULAR\r\n");
        return;
    }
    baseRad = g_currentJointAngles.baseDeg * ARM_DEG_TO_RAD;
    newBaseRad = baseRad + tangentialMm / currentRadius;
    x = newRadius * cosf(newBaseRad);
    y = newRadius * sinf(newBaseRad);

    if ((newRadius < ARM_VISION_MIN_RADIUS_MM) ||
        (newRadius > ARM_VISION_MAX_RADIUS_MM) ||
        (z < ARM_VISION_MIN_TOOL_Z_MM) ||
        (z > ARM_VISION_MAX_TOOL_Z_MM)) {
        g_visionTrackingEnabled = false;
        app_sync_stop_all();
        RpiLink_SendString("ERR WORKSPACE TRACK_OFF\r\n");
        return;
    }

    g_ikTargetX = x;
    g_ikTargetY = y;
    g_ikTargetZ = z;
    ikResult = RobotArm_SolveTarget(x, y, z, &g_currentJointAngles, &solution);
    if (ikResult != ROBOT_ARM_IK_OK) {
        g_visionTrackingEnabled = false;
        app_sync_stop_all();
        RpiLink_SendString("ERR IK TRACK_OFF\r\n");
        return;
    }

    g_ikSolvedBaseDeg = solution.baseDeg;
    g_ikSolvedLowerDeg = solution.lowerDeg;
    g_ikSolvedUpperDeg = solution.upperDeg;
    ok = app_step_toward_joint_angles(
        &solution, ARM_VISION_MAX_JOINT_STEP_DEG, &reached);
    g_multiLastOperationOk = ok;
    if (!ok) {
        g_visionTrackingEnabled = false;
        if (g_visionPoseKnown) {
            RpiLink_SendString("ERR MOTOR TRACK_OFF\r\n");
        }
        return;
    }

    DL_GPIO_togglePins(LED1_PORT, LED1_PIN_22_PIN);
    RpiLink_SendString("BALL MOVED ");
    RpiLink_SendUnsigned(sequence);
    RpiLink_SendString(reached ? " TARGET_STEP" : " JOINT_LIMITED");
    RpiLink_SendString("\r\n");
    app_send_pose("POSE_XYZ_X10");
}

static void app_process_rpi_command(char *line)
{
    uint32_t ball[6];
    uint32_t jogSequence;
    int32_t jogDeltas[MOTOR_COUNT];
    uint8_t i;

    if (strcmp(line, "PING") == 0) {
        RpiLink_SendString("PONG\r\n");
    } else if (strncmp(line, "PING ", 5U) == 0) {
        RpiLink_SendString("PONG ");
        RpiLink_SendString(line + 5);
        RpiLink_SendString("\r\n");
    } else if (strcmp(line, "STATUS") == 0) {
        RpiLink_SendString("STATUS OK ARM_VISION_V2 TRACK=");
        RpiLink_SendString(g_visionTrackingEnabled ? "1" : "0");
        RpiLink_SendString(" RELATIVE=");
        RpiLink_SendString(g_relativeVisualMode ? "1" : "0");
        RpiLink_SendString(" POSE=");
        RpiLink_SendString(g_visionPoseKnown ? "KNOWN\r\n" : "UNKNOWN\r\n");
        app_send_pose("POSE_XYZ_X10");
    } else if (strcmp(line, "HELP") == 0) {
        RpiLink_SendString(
            "COMMANDS PING | STATUS | TRACK START | TRACK STOP | "
            "POSE START | RELATIVE START | RELATIVE STOP | "
            "JOG seq d1_mdeg d2_mdeg d3_mdeg | "
            "BALL seq cx cy radius width height | LOST seq | !\r\n");
    } else if (strcmp(line, "RELATIVE START") == 0) {
        g_visionTrackingEnabled = false;
        g_relativeVisualMode = true;
        g_relativeLastSequence = 0xFFFFFFFFU;
        for (i = 0U; i < MOTOR_COUNT; i++) {
            g_relativeJointTotalMdeg[i] = 0;
        }
        app_sync_stop_all();
        RpiLink_SendString("RELATIVE READY LIMIT_MDEG ");
        RpiLink_SendUnsigned(ARM_RELATIVE_JOG_MAX_MDEG);
        RpiLink_SendString(" TOTAL_MDEG ");
        RpiLink_SendUnsigned(ARM_RELATIVE_TOTAL_MAX_MDEG);
        RpiLink_SendString("\r\n");
    } else if (strcmp(line, "RELATIVE STOP") == 0) {
        g_relativeVisualMode = false;
        app_sync_stop_all();
        RpiLink_SendString("RELATIVE OFF\r\n");
    } else if (strncmp(line, "JOG ", 4U) == 0) {
        if (!parse_jog(line + 4, &jogSequence, jogDeltas)) {
            RpiLink_SendString("ERR JOG_FORMAT\r\n");
        } else if (!g_relativeVisualMode) {
            RpiLink_SendString("ERR RELATIVE_OFF\r\n");
        } else if (jogSequence == g_relativeLastSequence) {
            RpiLink_SendString("ERR JOG_DUPLICATE\r\n");
        } else {
            g_relativeLastSequence = jogSequence;
            if (app_move_relative_joints(jogDeltas)) {
                app_send_relative_totals(jogSequence);
            } else {
                g_relativeVisualMode = false;
                app_sync_stop_all();
                RpiLink_SendString("ERR JOG_LIMIT_OR_MOTOR RELATIVE_OFF\r\n");
            }
        }
    } else if (strcmp(line, "TRACK START") == 0) {
        if (!g_visionPoseKnown) {
            RpiLink_SendString("ERR POSE_UNKNOWN\r\n");
        } else {
            g_visionTrackingEnabled = true;
            g_visionLastSequence = 0xFFFFFFFFU;
            DL_GPIO_clearPins(LED1_PORT, LED1_PIN_22_PIN);
            RpiLink_SendString("TRACK ON\r\n");
        }
    } else if ((strcmp(line, "TRACK STOP") == 0) ||
               (strcmp(line, "STOP") == 0)) {
        g_visionTrackingEnabled = false;
        g_relativeVisualMode = false;
        app_sync_stop_all();
        RpiLink_SendString("TRACK OFF\r\n");
    } else if (strcmp(line, "POSE START") == 0) {
        if (g_visionTrackingEnabled) {
            RpiLink_SendString("ERR TRACK_ACTIVE\r\n");
        } else {
            g_currentJointAngles.baseDeg = ARM_START_BASE_DEG;
            g_currentJointAngles.lowerDeg = ARM_START_LOWER_LINK_DEG;
            g_currentJointAngles.upperDeg = ARM_START_UPPER_LINK_DEG;
            g_visionPoseKnown = true;
            RpiLink_SendString("POSE ASSUMED_START\r\n");
            app_send_pose("POSE_XYZ_X10");
        }
    } else if (strncmp(line, "BALL ", 5U) == 0) {
        if (parse_ball(line + 5, ball)) {
            app_visual_servo_ball(ball);
        } else {
            RpiLink_SendString("ERR BALL_FORMAT\r\n");
        }
    } else if (strncmp(line, "LOST ", 5U) == 0) {
        RpiLink_SendString("LOST ACK\r\n");
    } else if (line[0] != '\0') {
        RpiLink_SendString("ERR UNKNOWN_COMMAND\r\n");
    }
}

static void app_run_ik_demo(void)
{
    RobotArmJointAngles target;
    RobotArmIkStatus ikResult;
    bool ok;
    bool reached;
    bool movingToDefault = !g_ikAtDefaultTarget;

    g_ikStatus = 0U;
    if (!g_ikAtDefaultTarget) {
        g_ikTargetX = ARM_DEFAULT_TARGET_X_MM;
        g_ikTargetY = ARM_DEFAULT_TARGET_Y_MM;
        g_ikTargetZ = ARM_DEFAULT_TARGET_Z_MM;
        ikResult = RobotArm_SolveTarget(
            g_ikTargetX, g_ikTargetY, g_ikTargetZ,
            &g_currentJointAngles, &target);
        if (ikResult != ROBOT_ARM_IK_OK) {
            g_ikStatus = 1U;
            g_multiLastOperationOk = false;
            app_blink_motor(5U);
            return;
        }
    } else {
        target.baseDeg = ARM_START_BASE_DEG;
        target.lowerDeg = ARM_START_LOWER_LINK_DEG;
        target.upperDeg = ARM_START_UPPER_LINK_DEG;
    }

    g_ikSolvedBaseDeg = target.baseDeg;
    g_ikSolvedLowerDeg = target.lowerDeg;
    g_ikSolvedUpperDeg = target.upperDeg;
    ok = app_step_toward_joint_angles(
        &target, ARM_IK_TEST_MAX_DELTA_DEG, &reached);
    g_multiLastOperationOk = ok;
    if (!ok) {
        if (g_ikStatus == 0U) {
            g_ikStatus = 3U;
        }
        app_blink_motor(5U);
        return;
    }
    if (reached) {
        g_ikAtDefaultTarget = !g_ikAtDefaultTarget;
    }
    g_ikStatus = 0U;
    g_multiMotorMode = movingToDefault ? 1U : 2U;
    app_blink_motor(g_multiMotorMode);
}

int main(void)
{
    char line[RPI_LINK_LINE_SIZE];

    SYSCFG_DL_init();
    EmmMulti_Init();
    RpiLink_Init();

    g_multiMotorMode = 0U;
    g_multiLastOperationOk = false;
    g_ikStatus = 0U;
    g_ikAtDefaultTarget = false;
    g_ikTargetX = ARM_DEFAULT_TARGET_X_MM;
    g_ikTargetY = ARM_DEFAULT_TARGET_Y_MM;
    g_ikTargetZ = ARM_DEFAULT_TARGET_Z_MM;
    g_ikSolvedBaseDeg = ARM_START_BASE_DEG;
    g_ikSolvedLowerDeg = ARM_START_LOWER_LINK_DEG;
    g_ikSolvedUpperDeg = ARM_START_UPPER_LINK_DEG;
    g_ikAxisPulses[0] = 0U;
    g_ikAxisPulses[1] = 0U;
    g_ikAxisPulses[2] = 0U;
    g_currentJointAngles.baseDeg = ARM_START_BASE_DEG;
    g_currentJointAngles.lowerDeg = ARM_START_LOWER_LINK_DEG;
    g_currentJointAngles.upperDeg = ARM_START_UPPER_LINK_DEG;
    g_visionTrackingEnabled = false;
    g_visionPoseKnown = false;
    g_visionLastSequence = 0xFFFFFFFFU;
    g_relativeVisualMode = false;
    g_relativeLastSequence = 0xFFFFFFFFU;
    g_relativeJointTotalMdeg[0] = 0;
    g_relativeJointTotalMdeg[1] = 0;
    g_relativeJointTotalMdeg[2] = 0;
    DL_GPIO_clearPins(LED1_PORT, LED1_PIN_22_PIN);

    app_delay_ms(2000U);
    g_multiLastOperationOk = app_enable_all();
    app_delay_ms(100U);
    g_multiLastOperationOk = app_sync_stop_all() && g_multiLastOperationOk;
    RpiLink_SendString("READY ARM_VISION_V2 TRACK_OFF RELATIVE_OFF\r\n");

    while (1) {
        if (RpiLink_TakeEmergencyStop()) {
            g_visionTrackingEnabled = false;
            g_relativeVisualMode = false;
            g_visionPoseKnown = false;
            app_sync_stop_all();
            RpiLink_SendString("STOPPED EMERGENCY POSE_UNKNOWN\r\n");
        }
        if (RpiLink_TakeOverflow()) {
            RpiLink_SendString("ERR RX_OVERFLOW\r\n");
        }
        while (RpiLink_ReadLine(line, (uint16_t)sizeof(line))) {
            app_process_rpi_command(line);
        }

        if ((!g_visionTrackingEnabled) && g_visionPoseKnown &&
            (DL_GPIO_readPins(KEY_PORT, KEY_PIN_21_PIN) == 0U)) {
            app_delay_ms(20U);
            if (DL_GPIO_readPins(KEY_PORT, KEY_PIN_21_PIN) != 0U) {
                continue;
            }
            app_run_ik_demo();
            while (DL_GPIO_readPins(KEY_PORT, KEY_PIN_21_PIN) == 0U) {
                app_delay_ms(1U);
            }
            app_delay_ms(20U);
        }
    }
}
