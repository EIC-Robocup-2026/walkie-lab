#include "MotionControl.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ============================================
// PCA9685 PWM Driver Instance
// ============================================
static Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDRESS);

// ============================================
// Servo State Management
// ============================================
static ServoState servoStates[NUM_SERVOS];
static ServoProfile activeProfiles[NUM_SERVOS];
static unsigned long lastUpdateMs = 0;

// ============================================
// Helper Functions
// ============================================

/**
 * Convert angle to PWM microseconds based on servo profile range
 */
static uint16_t angleToMicroseconds(double angle, const ServoProfile &profile) {
    // Clamp angle to profile's servo range (0 to servoRangeDegrees)
    angle = constrain(angle, 0.0, profile.servoRangeDegrees);
    
    // Linear interpolation from angle range to microsecond range
    // angle: 0 to servoRangeDegrees maps to minPulseUs to maxPulseUs
    double normalized = angle / profile.servoRangeDegrees;
    double microseconds = profile.minPulseUs + (normalized * (double)(profile.maxPulseUs - profile.minPulseUs));
    
    return (uint16_t)constrain(microseconds, profile.minPulseUs, profile.maxPulseUs);
}

/**
 * Write angle to servo via PCA9685
 */
static void writeServoAngle(uint8_t servoId, double angle) {
    if (servoId >= NUM_SERVOS) return;
    
    const ServoProfile &profile = activeProfiles[servoId];
    uint16_t microseconds = angleToMicroseconds(angle, profile);
    pwm.writeMicroseconds(profile.pwmChannel, microseconds);
}

/**
 * Calculate max angle change based on velocity and acceleration limits
 */
static double calculateMaxDelta(double currentVelocity, double maxVelocity, 
                               double maxAcceleration, double deltaTime) {
    // deltaTime in seconds
    double dt = deltaTime / 1000.0;
    
    // Maximum velocity change due to acceleration in this time step
    double maxVelChange = maxAcceleration * dt;
    
    // New velocity (cannot exceed maxVelocity)
    double newVelocity = currentVelocity + maxVelChange;
    if (newVelocity > maxVelocity) newVelocity = maxVelocity;
    
    // Maximum angle change = average velocity * time
    double maxDelta = ((currentVelocity + newVelocity) / 2.0) * dt;

    return maxDelta;
}

// ============================================
// Public API Implementation
// ============================================

void initMotionControl() {
    // Initialize I2C bus with custom pins
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    
    // Initialize PCA9685 PWM driver
    if (!pwm.begin()) {
        if (DEBUG_SERIAL) {
            Serial.println("ERROR: PCA9685 not found!");
        }
        while (1) delay(10); // Halt on failure
    }
    
    // Configure PCA9685
    pwm.setOscillatorFrequency(27000000);  // 27 MHz internal oscillator
    pwm.setPWMFreq(PCA9685_FREQUENCY);    // 50 Hz for standard servos
    
    // Initialize servo states
    for (int i = 0; i < NUM_SERVOS; i++) {
        // Copy initial profile
        activeProfiles[i] = SERVO_PROFILES[i];
        
        // Initialize state
        servoStates[i].currentAngle = activeProfiles[i].initAngle;
        servoStates[i].targetAngle = activeProfiles[i].initAngle;
        servoStates[i].velocity = 0.0;
        servoStates[i].lastUpdateTime = millis();
        servoStates[i].isMoving = false;
        
        // Write initial position
        writeServoAngle(i, activeProfiles[i].initAngle);
    }
    
    lastUpdateMs = millis();
    
    if (DEBUG_SERIAL) {
        Serial.println("Motion control initialized with 7 servos");
    }
}

void updateMotion() {
    unsigned long now = millis();
    double deltaTimeMs = (double)(now - lastUpdateMs);
    lastUpdateMs = now;
    
    if (deltaTimeMs < 1.0) return;  // Prevent division by very small numbers
    
    // Update each servo
    for (int i = 0; i < NUM_SERVOS; i++) {
        ServoState &state = servoStates[i];
        const ServoProfile &profile = activeProfiles[i];
        
        // Calculate error (difference between target and current)
        double error = state.targetAngle - state.currentAngle;
        double absError = fabs(error);
        
        // Check if servo has reached target
        if (absError < ANGLE_TOLERANCE) {
            state.currentAngle = state.targetAngle;
            state.velocity = 0.0;
            state.isMoving = false;
            writeServoAngle(i, state.currentAngle);
            continue;
        }
        
        // Calculate maximum allowed position change
        double maxDelta = calculateMaxDelta(state.velocity, profile.maxVelocity, 
                                           profile.maxAcceleration, deltaTimeMs);
        
        // Determine direction
        int direction = (error > 0) ? 1 : -1;
        
        // Apply movement constraint
        double newAngle;
        if (absError <= maxDelta) {
            // Can reach target this update
            newAngle = state.targetAngle;
            state.velocity = 0.0;
            state.isMoving = false;
        } else {
            // Move towards target by maxDelta
            newAngle = state.currentAngle + maxDelta;
            // Update velocity (accelerating towards max)
            state.velocity = state.velocity + (direction * (profile.maxAcceleration * deltaTimeMs / 1000.0));
            if (fabs(state.velocity) > profile.maxVelocity) {
                state.velocity = direction * profile.maxVelocity;
            }
            state.isMoving = true;
        }
        
        // Ensure angle is within profile's safe limits
        newAngle = constrain(newAngle, profile.minAngleLimit, profile.maxAngleLimit);
        state.currentAngle = newAngle;
        
        if(DEBUG_SERIAL && i == 1) {
            // Debug the direction and error
            Serial.printf("Servo %d: Curr=%.1f°, Target=%.1f°, Error=%.2f°, New=%.2f°, Vel=%.2f°/s\n",
                          i, state.currentAngle, state.targetAngle, error, newAngle, state.velocity);
        }

        // Write to servo
        writeServoAngle(i, state.currentAngle);

    }
    // if (DEBUG_SERIAL) {
    //     // One line summary of servo states
    //     // Current angles
    //     Serial.print("Currents: ");
    //     for (int i = 0; i < NUM_SERVOS; i++) {
    //         Serial.printf("S%d:%.1f° ", i, servoStates[i].currentAngle);
    //     }
    //     Serial.println();
    //     // Target angles
    //     Serial.print("Targets: ");
    //     for (int i = 0; i < NUM_SERVOS; i++) {
    //         Serial.printf("S%d:%.1f° ", i, servoStates[i].targetAngle);
    //     }
    //     Serial.println();

    // }
}

bool setServoTarget(uint8_t servoId, double targetAngle) {
    if (servoId >= NUM_SERVOS) {
        return false;
    }
    
    const ServoProfile &profile = activeProfiles[servoId];
    
    // Clamp target angle to profile limits (minAngleLimit to maxAngleLimit)
    targetAngle = constrain(targetAngle, profile.minAngleLimit, profile.maxAngleLimit);
    servoStates[servoId].targetAngle = targetAngle;
    
    if (DEBUG_SERIAL) {
        Serial.printf("Servo %d target set to %.1f°\n", servoId, targetAngle);
    }
    
    return true;
}

double getServoAngle(uint8_t servoId) {
    if (servoId >= NUM_SERVOS) {
        return -1.0;
    }
    return servoStates[servoId].currentAngle;
}

double getServoTarget(uint8_t servoId) {
    if (servoId >= NUM_SERVOS) {
        return -1.0;
    }
    return servoStates[servoId].targetAngle;
}

bool isServoMoving(uint8_t servoId) {
    if (servoId >= NUM_SERVOS) {
        return false;
    }
    return servoStates[servoId].isMoving;
}

bool allServosAtTarget() {
    for (int i = 0; i < NUM_SERVOS; i++) {
        if (servoStates[i].isMoving) {
            return false;
        }
    }
    return true;
}

bool setServoProfile(uint8_t servoId, const ServoProfile &newProfile) {
    if (servoId >= NUM_SERVOS) {
        return false;
    }
    
    activeProfiles[servoId] = newProfile;
    
    if (DEBUG_SERIAL) {
        Serial.printf("Servo %d profile updated: vel=%.1f°/s, accel=%.1f°/s²\n", 
                      servoId, newProfile.maxVelocity, newProfile.maxAcceleration);
    }
    
    return true;
}

ServoProfile getServoProfile(uint8_t servoId) {
    if (servoId >= NUM_SERVOS) {
        return SERVO_PROFILES[0];  // Return default as fallback
    }
    return activeProfiles[servoId];
}

void resetToInitial() {
    for (int i = 0; i < NUM_SERVOS; i++) {
        setServoTarget(i, SERVO_PROFILES[i].initAngle);
    }
    
    if (DEBUG_SERIAL) {
        Serial.println("All servos reset to initial positions");
    }
}

void stopAllServos() {
    for (int i = 0; i < NUM_SERVOS; i++) {
        servoStates[i].targetAngle = servoStates[i].currentAngle;
        servoStates[i].velocity = 0.0;
        servoStates[i].isMoving = false;
    }
    
    if (DEBUG_SERIAL) {
        Serial.println("All servos stopped");
    }
}

uint8_t getNumServos() {
    return NUM_SERVOS;
}
