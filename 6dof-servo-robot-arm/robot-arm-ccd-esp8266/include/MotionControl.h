#pragma once
#include <Servo.h>

extern Servo  servos[6];
extern double currentServoAngle[6];
extern int8_t lastMoveDir[6];
extern double baseTargetDeg;

void   initServos();
bool   anglesCloseTo(const double a[6], const double b[6], double eps=0.5);

// เขียนแบบนุ่ม: จาก current → targets โดยจำกัด ≤ MAX_STEP_DEG/แกน + no-reverse
void   moveTowardAnglesCappedSmooth(const double targetAngles[6],
                                    double maxStepDeg,
                                    int substeps,
                                    int subDelayMs);
