#include "robot_arm_kinematics.h"
#include "robot_arm_config.h"

#include <math.h>
#include <stddef.h>

#define ARM_RAD_TO_DEG 57.29577951308232f
#define ARM_DEG_TO_RAD 0.017453292519943295f

static float square(float value)
{
    return value * value;
}

float RobotArm_WrappedDeltaDeg(float targetDeg, float currentDeg)
{
    float delta = targetDeg - currentDeg;

    while (delta > 180.0f) {
        delta -= 360.0f;
    }
    while (delta < -180.0f) {
        delta += 360.0f;
    }
    return delta;
}

bool RobotArm_ForwardTarget(const RobotArmJointAngles *joints,
                            float *xMm, float *yMm, float *zMm)
{
    float baseRad;
    float lowerRad;
    float upperRad;
    float toolRadius;

    if ((joints == NULL) || (xMm == NULL) ||
        (yMm == NULL) || (zMm == NULL)) {
        return false;
    }

    baseRad = joints->baseDeg * ARM_DEG_TO_RAD;
    lowerRad = joints->lowerDeg * ARM_DEG_TO_RAD;
    upperRad = joints->upperDeg * ARM_DEG_TO_RAD;
    toolRadius = ARM_LINK_LOWER_MM * cosf(lowerRad) +
                 ARM_LINK_UPPER_MM * cosf(upperRad) +
                 ARM_WRIST_TO_TOOL_AXIS_MM;

    *xMm = toolRadius * cosf(baseRad);
    *yMm = toolRadius * sinf(baseRad);
    *zMm = ARM_BASE_TO_SHOULDER_Z_MM +
           ARM_LINK_LOWER_MM * sinf(lowerRad) +
           ARM_LINK_UPPER_MM * sinf(upperRad) -
           ARM_MAGNET_TOOL_LENGTH_MM;
    return true;
}

static void solve_planar_candidate(float wristRadius, float wristHeight,
                                   float relativeAngle,
                                   RobotArmJointAngles *candidate)
{
    float lower = atan2f(wristHeight, wristRadius) -
        atan2f(ARM_LINK_UPPER_MM * sinf(relativeAngle),
               ARM_LINK_LOWER_MM +
               ARM_LINK_UPPER_MM * cosf(relativeAngle));

    candidate->lowerDeg = lower * ARM_RAD_TO_DEG;
    candidate->upperDeg = (lower + relativeAngle) * ARM_RAD_TO_DEG;
}

RobotArmIkStatus RobotArm_SolveTarget(float xMm, float yMm, float zMm,
                                      const RobotArmJointAngles *reference,
                                      RobotArmJointAngles *solution)
{
    RobotArmJointAngles candidatePositive;
    RobotArmJointAngles candidateNegative;
    float toolRadius;
    float wristRadius;
    float wristHeight;
    float cosineRelative;
    float relativeMagnitude;
    float positiveCost;
    float negativeCost;

    if ((reference == NULL) || (solution == NULL)) {
        return ROBOT_ARM_IK_INVALID_ARGUMENT;
    }

    toolRadius = sqrtf(square(xMm) + square(yMm));
    wristRadius = toolRadius - ARM_WRIST_TO_TOOL_AXIS_MM;
    wristHeight = zMm + ARM_MAGNET_TOOL_LENGTH_MM -
                  ARM_BASE_TO_SHOULDER_Z_MM;

    cosineRelative =
        (square(wristRadius) + square(wristHeight) -
         square(ARM_LINK_LOWER_MM) - square(ARM_LINK_UPPER_MM)) /
        (2.0f * ARM_LINK_LOWER_MM * ARM_LINK_UPPER_MM);

    if ((cosineRelative < -1.0001f) || (cosineRelative > 1.0001f)) {
        return ROBOT_ARM_IK_UNREACHABLE;
    }
    if (cosineRelative < -1.0f) {
        cosineRelative = -1.0f;
    } else if (cosineRelative > 1.0f) {
        cosineRelative = 1.0f;
    }

    relativeMagnitude = acosf(cosineRelative);
    solve_planar_candidate(wristRadius, wristHeight, relativeMagnitude,
                           &candidatePositive);
    solve_planar_candidate(wristRadius, wristHeight, -relativeMagnitude,
                           &candidateNegative);

    candidatePositive.baseDeg = atan2f(yMm, xMm) * ARM_RAD_TO_DEG;
    candidateNegative.baseDeg = candidatePositive.baseDeg;

    positiveCost =
        square(RobotArm_WrappedDeltaDeg(candidatePositive.lowerDeg,
                                        reference->lowerDeg)) +
        square(RobotArm_WrappedDeltaDeg(candidatePositive.upperDeg,
                                        reference->upperDeg));
    negativeCost =
        square(RobotArm_WrappedDeltaDeg(candidateNegative.lowerDeg,
                                        reference->lowerDeg)) +
        square(RobotArm_WrappedDeltaDeg(candidateNegative.upperDeg,
                                        reference->upperDeg));

    *solution = (negativeCost <= positiveCost) ?
                candidateNegative : candidatePositive;
    return ROBOT_ARM_IK_OK;
}
