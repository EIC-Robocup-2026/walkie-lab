#pragma once

#include "CommandReceiver.h"
#include "Config.h"
#include <esp_now.h>
#include <WiFi.h>
#include "MotionControl.h"

// ============================================
// ESP-NOW Command Receiver Implementation
// ============================================

class EspNowReceiver : public CommandReceiver {
private:
    uint8_t  lastServoId;
    double   lastTargetAngle;
    bool     commandAvailable;
    static EspNowReceiver* instance;
    
    // Static callback (required by ESP-NOW)
    static void onDataReceived(const esp_now_recv_info_t *recvInfo, const uint8_t *data, int dataLen) {
        if (instance == nullptr) return;

        // Only accept the batch packet: exactly 7 floats
        if (dataLen == (int)(sizeof(float) * NUM_SERVOS)) {
            // Copy floats safely
            float angles[NUM_SERVOS];
            memcpy(angles, data, sizeof(float) * NUM_SERVOS);

            if (DEBUG_SERIAL) {
                const uint8_t *macAddr = recvInfo->src_addr;
                Serial.printf("ESP-NOW RX: ALL 7 angles from MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                             macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]);
            }

            // Apply each angle to motion controller
            for (uint8_t i = 0; i < NUM_SERVOS; i++) {
            float a = angles[i];
            const ServoProfile &profile = SERVO_PROFILES[i];
            if (a < (float)profile.minAngleLimit || a > (float)profile.maxAngleLimit) {
                    if (DEBUG_SERIAL) {
                        Serial.printf("ERROR: Angle for servo %d out of range: %.1f\n", i, a);
                    }
                    continue; // skip invalid values but continue with others
                }
                // Use motion control API to set target
                setServoTarget(i, (double)a);
                if (DEBUG_SERIAL) {
                    Serial.printf("  -> Set Servo %d target to %.1f°\n", i, a);
                }
            }
            return;
        }

        // Unknown or unsupported packet size — ignore
        if (DEBUG_SERIAL) {
            Serial.printf("ESP-NOW RX: Ignored packet with unexpected size: %d bytes\n", dataLen);
        }
    }
    
public:
    EspNowReceiver() : lastServoId(0), lastTargetAngle(90.0), commandAvailable(false) {
        instance = this;
    }
    
    ~EspNowReceiver() {
        instance = nullptr;
        esp_now_deinit();
    }
    
    void init() override {
        // Set Wi-Fi mode to Station
        WiFi.mode(ESPNOW_WIFI_MODE);
        WiFi.disconnect();  // Disconnect from any connected network
        
        // Get ESP32 MAC address
        uint8_t localMac[6];
        WiFi.macAddress(localMac);
        
        if (DEBUG_SERIAL) {
            Serial.println("\n=== ESP-NOW Receiver Initialized ===");
            Serial.printf("Local MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                         localMac[0], localMac[1], localMac[2], localMac[3], localMac[4], localMac[5]);
        }
        
        // Initialize ESP-NOW
        if (esp_now_init() != ESP_OK) {
            if (DEBUG_SERIAL) {
                Serial.println("ERROR: Failed to initialize ESP-NOW");
            }
            return;
        }
        
        // Register the receive callback
        esp_now_register_recv_cb(onDataReceived);
        
        if (DEBUG_SERIAL) {
            Serial.println("Waiting for commands via ESP-NOW...");
        }
    }
    
    void update() override {
        // ESP-NOW uses interrupts, so no update needed
        // Command availability is handled in the callback
    }
    
    bool hasCommand() override {
        return commandAvailable;
    }
    
    bool getCommand(uint8_t &servoId, double &targetAngle) override {
        if (!commandAvailable) {
            return false;
        }
        
        servoId = lastServoId;
        targetAngle = lastTargetAngle;
        commandAvailable = false;
        
        return true;
    }
};

// Static instance pointer initialization
EspNowReceiver* EspNowReceiver::instance = nullptr;
