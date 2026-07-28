#ifndef ROBOT_ARM_KINEMATICS_H
#define ROBOT_ARM_KINEMATICS_H

#include <stdbool.h>

typedef struct {
    float baseDeg;
    float lowerDeg;
    float upperDeg;
} RobotArmJointAngles;

typedef enum {
    ROBOT_ARM_IK_OK = 0,
    ROBOT_ARM_IK_INVALID_ARGUMENT,
    ROBOT_ARM_IK_UNREACHABLE
} RobotArmIkStatus;

RobotArmIkStatus RobotArm_SolveTarget(float xMm, float yMm, float zMm,
                                      const RobotArmJointAngles *reference,
                                      RobotArmJointAngles *solution);

bool RobotArm_ForwardTarget(const RobotArmJointAngles *joints,
                            float *xMm, float *yMm, float *zMm);

float RobotArm_WrappedDeltaDeg(float targetDeg, float currentDeg);

#endif
