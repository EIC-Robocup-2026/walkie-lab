#include <Arduino.h>
#include "Config.h"
#include "MotionControl.h"
#include "CommandReceiver.h"
#include "UartReceiver.h"
#include "EspNowReceiver.h"

// ============================================
// Command Receiver Instance
// ============================================
CommandReceiver* commandReceiver = nullptr;

// ============================================
// Setup
// ============================================
void setup() {
    // Initialize debug serial
    Serial.begin(DEBUG_BAUD_RATE);
    delay(1000);
    
    if (DEBUG_SERIAL) {
        Serial.println("\n\n========================================");
        Serial.println("     Robot Arm Servo Controller");
        Serial.println("     7-Servo System with PCA9685");
        Serial.println("========================================");
    }
    
    // Initialize motion control (I2C, PCA9685, servos)
    initMotionControl();
    
    // Initialize command receiver based on configuration
    if (ACTIVE_COMMAND_MODE == COMMAND_MODE_UART) {
        commandReceiver = new UartReceiver();
        if (DEBUG_SERIAL) {
            Serial.println("Selected: UART Command Receiver");
        }
    } else if (ACTIVE_COMMAND_MODE == COMMAND_MODE_ESPNOW) {
        commandReceiver = new EspNowReceiver();
        if (DEBUG_SERIAL) {
            Serial.println("Selected: ESP-NOW Command Receiver");
        }
    }
    
    if (commandReceiver != nullptr) {
        commandReceiver->init();
    } else {
        if (DEBUG_SERIAL) {
            Serial.println("ERROR: Invalid command mode selected!");
        }
    }
    
    if (DEBUG_SERIAL) {
        Serial.println("Setup complete. Ready to receive commands.\n");
    }
}

// ============================================
// Main Loop
// ============================================
void loop() {
    // Update command receiver (check for new commands)
    if (commandReceiver != nullptr) {
        commandReceiver->update();
        
        // Process incoming commands
        uint8_t servoId;
        double targetAngle;
        
        if (commandReceiver->getCommand(servoId, targetAngle)) {
            // Set new target for the servo
            setServoTarget(servoId, targetAngle);
            if (DEBUG_SERIAL) {
                Serial.printf("Command received: Servo %d -> Target Angle %.1f°\n", servoId, targetAngle);
            }
        }
    }
    
    // Update motion control (apply velocity/acceleration limits)
    updateMotion();
    
    // Small delay to control loop rate
    delay(MOTION_UPDATE_INTERVAL_MS);
}
