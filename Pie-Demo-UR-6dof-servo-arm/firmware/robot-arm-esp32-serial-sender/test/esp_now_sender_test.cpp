/*
  ESP-NOW Sender - Mode 3 Only
  
  Sends all 7 servo angles at once in a single packet.
  
  Input Format: <angle1>,<angle2>,...,<angle7>
  Example: 90.0,45.5,135.0,90.0,45.0,120.5,80.0
  
  Compile & Upload:
    platformio run -e esp32_c6_dev --target upload --target monitor
*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// ============================================
// PACKET STRUCTURE - ALL 7 SERVOS
// ============================================

typedef struct {
    float angles[7];
} all_servos_packet_t;

// ============================================
// CONFIGURATION
// ============================================

uint8_t receiverMAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};  // Update with receiver MAC

// Global variables
uint32_t sentCount = 0;
esp_now_peer_info_t peerInfo = {};

// ============================================
// CALLBACK
// ============================================

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    sentCount++;
    Serial.printf("✓ Packet #%lu - %s\n", sentCount, 
                  status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAIL");
}

// ============================================
// SEND FUNCTION
// ============================================

void sendAllServos(float angles[7]) {
    all_servos_packet_t packet;
    
    // Validate and copy angles
    bool valid = true;
    for (int i = 0; i < 7; i++) {
        if (angles[i] < 0 || angles[i] > 180) {
            Serial.printf("❌ Servo %d: Angle %.1f° out of range (0-180°)\n", i, angles[i]);
            valid = false;
            break;
        }
        packet.angles[i] = angles[i];
    }
    
    if (!valid) return;
    
    // Send packet
    Serial.println("\n📤 Sending all 7 servo angles:");
    for (int i = 0; i < 7; i++) {
        Serial.printf("   Servo %d: %.1f°\n", i, packet.angles[i]);
    }
    Serial.printf("   Packet size: %d bytes\n", sizeof(packet));
    
    esp_now_send(receiverMAC, (uint8_t *)&packet, sizeof(packet));
}

// ============================================
// INITIALIZATION
// ============================================

void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("\n\n" + String(60, '='));
    Serial.println("ESP-NOW SENDER - MODE 3 (ALL 7 SERVOS)");
    Serial.println(String(60, '='));
    
    // Show this device's MAC
    WiFi.mode(WIFI_STA);
    Serial.print("Device MAC: ");
    Serial.println(WiFi.macAddress());
    
    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("ERROR: esp_now_init failed");
        while (1) delay(1000);
    }
    
    Serial.println("Initializing peer...");
    
    // Register send callback
    esp_now_register_send_cb(onDataSent);
    
    // Add receiver as peer
    memcpy(peerInfo.peer_addr, receiverMAC, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("ERROR: Failed to add peer!");
        while (1) delay(1000);
    }
    
    Serial.println("✓ Peer added\n");
    Serial.println(String(60, '='));
    Serial.println("INPUT FORMAT");
    Serial.println(String(60, '='));
    Serial.println("Enter 7 comma-separated angles (0-180°)");
    Serial.println("\nExample:");
    Serial.println("  90.0,45.5,135.0,90.0,45.0,120.5,80.0");
    Serial.println("\nTest Command:");
    Serial.println("  T  (sends test pose)");
    Serial.println(String(60, '=') + "\n");
}

// ============================================
// MAIN LOOP
// ============================================

void loop() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        
        if (input.length() == 0) return;
        
        // Test command
        if (input == "T" || input == "t") {
            float testAngles[7] = {90.0, 45.5, 135.0, 90.0, 45.0, 120.5, 80.0};
            sendAllServos(testAngles);
        }
        // Parse 7 comma-separated angles
        else if (input.indexOf(',') > 0) {
            float angles[7];
            int startIdx = 0;
            bool valid = true;
            
            for (int i = 0; i < 7; i++) {
                int commaIdx = input.indexOf(',', startIdx);
                
                String angleStr;
                if (commaIdx == -1) {
                    if (i < 6) {
                        Serial.printf("❌ Expected 7 angles, got %d\n", i);
                        valid = false;
                        break;
                    }
                    angleStr = input.substring(startIdx);
                } else {
                    angleStr = input.substring(startIdx, commaIdx);
                    startIdx = commaIdx + 1;
                }
                
                angles[i] = angleStr.toFloat();
            }
            
            if (valid) {
                sendAllServos(angles);
            }
        }
        else {
            Serial.println("❌ Invalid format. Use: angle1,angle2,...,angle7");
        }
    }
    
    delay(100);
}
