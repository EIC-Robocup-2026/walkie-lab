#ifndef WHEEL_FEEDBACK_H
#define WHEEL_FEEDBACK_H

#include <ESP32Encoder.h>
#include <Arduino.h>

class WheelFeedback {
private:
    ESP32Encoder& encoder;
    float encoderCPR;      // Counts per revolution
    unsigned long lastTime;
    int32_t lastCount;
    float angularVelocity;   // in rad/s
    
public:
    WheelFeedback(ESP32Encoder& _encoder, float _encoderCPR = 1228.8)
        : encoder(_encoder)
        , encoderCPR(_encoderCPR)
        , lastTime(0)
        , lastCount(0)
        , angularVelocity(0.0)
    {
    }

    // Get the current angular position in radians
    float getAngularPosition() {
        return (2.0f * M_PI * (float)encoder.getCount()) / encoderCPR;
    }

    // Update and get the current angular velocity in rad/s
    float updateAngularVelocity() {
        unsigned long currentTime = millis();
        int32_t currentCount = encoder.getCount();
        
        // Calculate time difference in seconds
        float deltaTime = (currentTime - lastTime) / 1000.0f;
        
        if (deltaTime > 0) {
            // Calculate encoder ticks difference
            float deltaTicks = currentCount - lastCount;
            
            // Calculate angular velocity (rad/s)
            angularVelocity = (2.0f * M_PI * deltaTicks) / (encoderCPR * deltaTime);
            
            // Update last values
            lastTime = currentTime;
            lastCount = currentCount;
        }
        
        return angularVelocity;
    }

    // Get the last calculated angular velocity without updating
    float getAngularVelocity() {
        return angularVelocity;
    }

    // Reset the encoder count
    void reset() {
        encoder.setCount(0);
        lastCount = 0;
        angularVelocity = 0.0;
        lastTime = millis();
    }

    // Set wheel parameters
    void setWheelParameters(int _encoderCPR) {
        encoderCPR = _encoderCPR;
    }
};

#endif // WHEEL_FEEDBACK_H