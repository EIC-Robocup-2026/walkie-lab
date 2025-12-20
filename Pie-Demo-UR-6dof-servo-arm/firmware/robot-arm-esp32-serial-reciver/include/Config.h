#pragma once
#include <Arduino.h>

// ============================================
// COMMAND RECEPTION MODE (select one)
// ============================================
#define COMMAND_MODE_UART    0
#define COMMAND_MODE_ESPNOW  1

// Select which mode to use (change this to switch receiver)
// #define ACTIVE_COMMAND_MODE  COMMAND_MODE_UART
#define ACTIVE_COMMAND_MODE  COMMAND_MODE_ESPNOW

// ============================================
// UART Configuration
// ============================================
#define UART_BAUD_RATE      115200

// ============================================
// I2C Configuration
// ============================================
#define I2C_SDA_PIN         5
#define I2C_SCL_PIN         6
#define PCA9685_ADDRESS     0x40
#define PCA9685_FREQUENCY   50  // Hz (standard for servos)

// ============================================
// Servo Configuration
// ============================================
#define NUM_SERVOS          7

// Servo profile structure for each servo
struct ServoProfile {
    uint8_t  pwmChannel;           // PCA9685 channel (0-15)
    double   initAngle;            // Initial position (degrees)
    double   servoRangeDegrees;    // Full servo range: 180 (standard) or 270 (continuous)
    double   minAngleLimit;        // Minimum safe operating angle (degrees)
    double   maxAngleLimit;        // Maximum safe operating angle (degrees)
    double   maxVelocity;          // Max angular velocity (degrees/second)
    double   maxAcceleration;      // Max angular acceleration (degrees/second²)
    uint16_t minPulseUs;           // Minimum pulse width (microseconds)
    uint16_t maxPulseUs;           // Maximum pulse width (microseconds)
};

// Servo profiles for all 7 servos
// Adjust these values based on your specific servos and requirements
static const ServoProfile SERVO_PROFILES[NUM_SERVOS] = {
    // {ch, init_angle, range_deg, min_limit, max_limit, ang_vel, ang_accel, min_us, max_us}

    // Servo 1 - Base rotation 
    {0,   135.0,  270.0,  0.0,    270.0,  30.0,  10.0,  450,  2420},
    // Servo 2 - Shoulder 
    {1,   180.0,   270.0,  0.0,    270.0,  30.0,  10.0,  450,  2420},
    // Servo 3 - Elbow 
    {2,   225.0,   270.0,  0.0,    270.0,  30.0,  10.0,  450,  2420},
    // Servo 4 - Wrist pitch 
    {3,   135.0,   270.0,  0.0,    270.0,  30.0,  10.0,  450,  2420},
    // Servo 5 - Wrist yaw 
    {4,   135.0,  180.0,  0.0,    180.0,  30.0,  10.0,  450,  2420},
    // Servo 6 - Wrist roll
    {5,   135.0,   270.0,  45.0,    225.0,  30.0,  10.0,  450,  2420},
    // Servo 7 - Gripper
    {6,   45.0,   270.0,  0.0,   270.0,  30.0,  10.0,  450,  2420},
};

// ============================================
// Motion Control Parameters
// ============================================
#define MOTION_UPDATE_INTERVAL_MS  20  // Update servos every 20ms (~50Hz)
#define ANGLE_TOLERANCE            0.5  // Consider servo "at target" within this tolerance

// ============================================
// ESP-NOW Configuration (if using ESP-NOW mode)
// ============================================
#define ESPNOW_CHANNEL       1
#define ESPNOW_WIFI_MODE     WIFI_MODE_STA

// MAC address of the sender device (configure this)
// Example: {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF}
// 40:4C:CA:5E:98:A8
static const uint8_t ESPNOW_SENDER_MAC[6] = {0x40, 0x4C, 0xCA, 0x5E, 0x98, 0xA8};

// ============================================
// Debug & Serial Output
// ============================================
#define DEBUG_SERIAL         1  // 1 to enable debug messages, 0 to disable
#define DEBUG_BAUD_RATE      115200