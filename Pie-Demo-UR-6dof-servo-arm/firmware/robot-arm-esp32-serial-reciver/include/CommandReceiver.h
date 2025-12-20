#pragma once

#include <Arduino.h>

// ============================================
// Command Receiver Abstract Interface
// ============================================

class CommandReceiver {
public:
    virtual ~CommandReceiver() {}
    
    /**
     * Initialize the command receiver
     * Called once during setup
     */
    virtual void init() = 0;
    
    /**
     * Update/process incoming commands
     * Should be called in the main loop
     */
    virtual void update() = 0;
    
    /**
     * Check if a new command is available
     * @return true if new command received, false otherwise
     */
    virtual bool hasCommand() = 0;
    
    /**
     * Get the latest command
     * @param servoId: Output - servo index (0 to NUM_SERVOS-1)
     * @param targetAngle: Output - target angle in degrees (0-180)
     * @return true if command retrieved successfully, false otherwise
     */
    virtual bool getCommand(uint8_t &servoId, double &targetAngle) = 0;
};
