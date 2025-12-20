#include "MotionControl.h"
#include "Config.h"
#include "Utilities.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Create PCA9685 driver object with default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// PWM configuration
#define SERVO_FREQ 50
#define USMIN  450    // Minimum microsecond length
#define USMAX  2300   // Maximum microsecond length

// Standard servos run at ~50 Hz updates
double currentServoAngle[6] = {INIT_ANGLES[0],INIT_ANGLES[1],INIT_ANGLES[2],INIT_ANGLES[3],INIT_ANGLES[4],INIT_ANGLES[5]};
int8_t lastMoveDir[6]       = {0,0,0,0,0,0};
double baseTargetDeg        = INIT_ANGLES[0];

// Convert angle to microseconds
static inline uint16_t angleToUs(double angle) {
    // Map angle (0-180) to microseconds range
    return map(angle, 0.0, 180.0, USMIN, USMAX);
}

// Base angle to microseconds conversion with special mapping
static inline int baseAngleToUs(double angleDeg) {
    // Clamp angle within allowed yaw range
    double a = clampBaseDeg(angleDeg);

    // Linear mapping from [BASE_DEG_MIN..BASE_DEG_MAX] → [BASE_US_AT_DEG_MIN..BASE_US_AT_DEG_MAX]
    long a10 = (long)(a * 10.0);
    long us  = map(a10,
                   (long)(BASE_DEG_MIN * 10.0),
                   (long)(BASE_DEG_MAX * 10.0),
                   BASE_US_AT_DEG_MIN,
                   BASE_US_AT_DEG_MAX);

    // Constrain to safe pulse range
    if (us < BASE_SAFE_MIN_US) us = BASE_SAFE_MIN_US;
    if (us > BASE_SAFE_MAX_US) us = BASE_SAFE_MAX_US;
    return (int)us;
}

void initServos() {
    // Initialize I2C and PCA9685
    Wire.begin();
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(SERVO_FREQ);
    
    // Small delay for the PWM chip to be ready
    delay(10);

    // Initialize base servo with special mapping
    pwm.writeMicroseconds(ATTACH_PIN[0], baseAngleToUs(currentServoAngle[0]));

    // Initialize other servos with standard microsecond mapping
    for(int i=1; i<6; ++i) {
        pwm.writeMicroseconds(ATTACH_PIN[i], 
            angleToUs(constrain(currentServoAngle[i], 0.0, 180.0)));
    }

    baseTargetDeg = clampBaseDeg(currentServoAngle[0]);
}

bool anglesCloseTo(const double a[6], const double b[6], double eps) {
    for(int i=0; i<6; ++i) {
        if (fabs(a[i]-b[i]) > eps) return false;
    }
    return true;
}

void moveTowardAnglesCappedSmooth(const double targetAngles[6],
                                 double maxStepDeg,
                                 int substeps,
                                 int subDelayMs)
{
    // 1) Limit targets to maxStepDeg per axis + no-reverse
    double clippedTarget[6];
    for(int i=0; i<6; ++i) {
        const double cur   = currentServoAngle[i];
        const double tgt   = targetAngles[i];
        const double delta = tgt - cur;
        const int    dir   = sgn(delta);

        if (lastMoveDir[i] != 0 && dir != 0 && dir != lastMoveDir[i] && 
            fabs(delta) <= NO_REVERSE_DEADBAND_DEG) {
            clippedTarget[i] = cur; // Don't move this axis
            continue;
        }

        double limited = cur + capDelta(delta, maxStepDeg);
        clippedTarget[i] = (i==0) ? clampBaseDeg(limited) : clampDeg(limited);

        if (fabs(clippedTarget[i] - cur) > 1e-6) {
            lastMoveDir[i] = sgn(clippedTarget[i] - cur);
        }
    }

    // 2) Break into sub-steps for smooth movement
    double start[6];
    for(int i=0; i<6; ++i) start[i] = currentServoAngle[i];

    for(int s=1; s<=substeps; ++s) {
        const double a = (double)s / (double)substeps; // 0..1
        for(int i=0; i<6; ++i) {
            double inter = start[i] + (clippedTarget[i] - start[i]) * a;

            if(i==0) {
                // Base: write microseconds with special mapping
                inter = clampBaseDeg(inter);
                pwm.writeMicroseconds(ATTACH_PIN[i], baseAngleToUs(inter));
            } else {
                // Other servos: write microseconds with standard mapping
                inter = constrain(inter, 0.0, 180.0);
                pwm.writeMicroseconds(ATTACH_PIN[i], angleToUs(inter));
            }

            currentServoAngle[i] = inter;
        }
        delay(subDelayMs);
    }
}
