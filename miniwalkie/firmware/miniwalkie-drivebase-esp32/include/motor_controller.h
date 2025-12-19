#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>
#include "motor.h"
#include "wheel_feedback.h"

class MotorSpeedController {
private:
    MotorI2C& motor;
    WheelFeedback& wheelFeedback;
    
    // PID gains
    float kp;
    float ki;
    float kd;
    
    // Feed-forward gains
    float ff_m;  // slope
    float ff_b;  // offset
    bool useFF;  // enable/disable feed-forward
    
    // PID variables
    float targetSpeed;      // Desired angular velocity in rad/s
    float lastError;        // Previous error for derivative term
    float integralError;    // Accumulated error for integral term
    unsigned long lastTime; // Last update time
    
    // Controller limits
    float maxOutput;        // Maximum motor command
    float minOutput;        // Minimum motor command
    float maxIntegral;      // Anti-windup limit
    
    // Compute feed-forward term
    float computeFeedForward(float targetVel) {
        if (!useFF) return 0;
        if (targetVel == 0) return 0;
        float ff = targetVel * ff_m + (targetVel > 0 ? ff_b : -ff_b);
        return constrain(ff, minOutput, maxOutput);
    }
    
    // Compute PID output
    float computePID(float currentSpeed) {
        unsigned long currentTime = millis();
        float deltaTime = (currentTime - lastTime) / 1000.0f; // Convert to seconds
        
        if (deltaTime <= 0) return 0;
        
        // Calculate error
        float error = targetSpeed - currentSpeed;
        
        // Proportional term
        float P = kp * error;
        
        // Integral term with anti-windup
        integralError += error * deltaTime;
        integralError = constrain(integralError, -maxIntegral, maxIntegral);
        float I = ki * integralError;
        
        // Derivative term (on measurement to avoid derivative kick)
        float derivative = (currentSpeed - lastError) / deltaTime;
        float D = -kd * derivative; // Negative because we want derivative of measurement
        
        // Update state variables
        lastError = currentSpeed;
        lastTime = currentTime;
        
        // Calculate total output
        float output = P + I + D;
        
        // Constrain output to motor limits
        return constrain(output, minOutput, maxOutput);
    }

public:
    MotorSpeedController(MotorI2C& _motor, WheelFeedback& _wheelFeedback, 
                        float _kp = 1.0, float _ki = 0.1, float _kd = 0.01)
        : motor(_motor)
        , wheelFeedback(_wheelFeedback)
        , kp(_kp)
        , ki(_ki)
        , kd(_kd)
        , ff_m(0.0)
        , ff_b(0.0)
        , useFF(false)
        , targetSpeed(0.0)
        , lastError(0.0)
        , integralError(0.0)
        , lastTime(0)
        , maxOutput(100.0)
        , minOutput(-100.0)
        , maxIntegral(1000.0)
    {
    }
    
    // Set feed-forward parameters
    void setFeedForward(float slope, float offset) {
        ff_m = slope;
        ff_b = offset;
        useFF = true;
    }
    
    // Enable/disable feed-forward control
    void enableFeedForward(bool enable) {
        useFF = enable;
    }
    
    // Get feed-forward parameters
    float getFFSlope() const { return ff_m; }
    float getFFOffset() const { return ff_b; }
    bool isFeedForwardEnabled() const { return useFF; }
    
    // Set the desired angular velocity in rad/s
    void setTargetSpeed(float speed) {
        targetSpeed = speed;
    }
    
    // Update control loop - call this regularly
    float update() {
        float currentSpeed = wheelFeedback.updateAngularVelocity();
        
        // Compute feed-forward term
        float ff_output = computeFeedForward(targetSpeed);
        
        // Compute PID correction
        float pid_output = computePID(currentSpeed);
        
        // Combine feed-forward and PID
        float total_output = ff_output + pid_output;
        
        // Apply final output with limits
        total_output = constrain(total_output, minOutput, maxOutput);
        motor.run(total_output);
        
        return total_output;
    }
    
    // Configure PID gains
    void setPIDGains(float _kp, float _ki, float _kd) {
        kp = _kp;
        ki = _ki;
        kd = _kd;
    }
    
    // Set output limits
    void setOutputLimits(float min, float max) {
        minOutput = min;
        maxOutput = max;
    }
    
    // Set integral anti-windup limit
    void setIntegralLimit(float limit) {
        maxIntegral = limit;
    }
    
    // Reset the controller
    void reset() {
        integralError = 0.0;
        lastError = 0.0;
        lastTime = millis();
        targetSpeed = 0.0;
        motor.run(0);
    }
    
    // Get current error
    float getError() {
        return targetSpeed - wheelFeedback.getAngularVelocity();
    }
    
    // Get integral error
    float getIntError() {
        return integralError;
    }
    
    // Get current target speed
    float getTargetSpeed() {
        return targetSpeed;
    }
    
    // Getters for PID parameters
    float getKp() const { return kp; }
    float getKi() const { return ki; }
    float getKd() const { return kd; }
};

#endif // MOTOR_CONTROLLER_H