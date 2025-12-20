#pragma once

#include <Arduino.h>
#include "Config.h"

// ============================================
// Servo State Structure
// ============================================
struct ServoState {
    double currentAngle;       // Current angle in degrees
    double targetAngle;        // Target angle in degrees
    double velocity;           // Current angular velocity (deg/s)
    double lastUpdateTime;     // Last update timestamp (ms)
    bool   isMoving;           // Whether servo is currently moving
};

// ============================================
// Motion Control API
// ============================================

/**
 * Initialize all servos
 * - Initializes I2C bus
 * - Initializes PCA9685 PWM driver
 * - Sets all servos to initial angles
 */
void initMotionControl();

/**
 * Update servo motion
 * Must be called regularly (ideally every 20ms)
 * Applies velocity and acceleration limits to reach target angles
 */
void updateMotion();

/**
 * Set target angle for a specific servo
 * @param servoId: Servo index (0 to NUM_SERVOS-1)
 * @param targetAngle: Target angle in degrees (0-180)
 * @return true if successful, false if invalid servoId
 */
bool setServoTarget(uint8_t servoId, double targetAngle);

/**
 * Get current angle of a servo
 * @param servoId: Servo index (0 to NUM_SERVOS-1)
 * @return Current angle in degrees, or -1.0 if invalid servoId
 */
double getServoAngle(uint8_t servoId);

/**
 * Get target angle of a servo
 * @param servoId: Servo index (0 to NUM_SERVOS-1)
 * @return Target angle in degrees, or -1.0 if invalid servoId
 */
double getServoTarget(uint8_t servoId);

/**
 * Check if a servo is currently moving
 * @param servoId: Servo index (0 to NUM_SERVOS-1)
 * @return true if servo is moving, false otherwise
 */
bool isServoMoving(uint8_t servoId);

/**
 * Check if all servos have reached their targets
 * @return true if all servos are idle at targets
 */
bool allServosAtTarget();

/**
 * Update servo profile at runtime
 * @param servoId: Servo index (0 to NUM_SERVOS-1)
 * @param newProfile: New servo profile with updated velocity/acceleration
 * @return true if successful
 */
bool setServoProfile(uint8_t servoId, const ServoProfile &newProfile);

/**
 * Get current servo profile
 * @param servoId: Servo index (0 to NUM_SERVOS-1)
 * @return Servo profile structure
 */
ServoProfile getServoProfile(uint8_t servoId);

/**
 * Move all servos to their initial angles (defined in Config)
 */
void resetToInitial();

/**
 * Stop all servos immediately
 */
void stopAllServos();

/**
 * Get the total number of servos
 * @return NUM_SERVOS
 */
uint8_t getNumServos();
