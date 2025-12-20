/*
  ESP-NOW Receiver - Mode 3 Only
  
  Receives all 7 servo angles in a single packet.
  Displays received angles and statistics.
  
  Compile & Upload:
    platformio run -e esp32_c6_dev --target upload --target monitor
*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// ============================================
// PACKET STRUCTURE (must match sender)
// ============================================

typedef struct {
    float angles[7];
} all_servos_packet_t;

// ============================================
// GLOBAL VARIABLES
// ============================================

uint32_t receivedCount = 0;
uint32_t errorCount = 0;

// ============================================
// DISPLAY FUNCTIONS
// ============================================

void displayRawPacket(const uint8_t *data, int dataLen) {
    Serial.print("  Raw Hex: ");
    for (int i = 0; i < dataLen; i++) {
        Serial.printf("%02X ", data[i]);
        if ((i + 1) % 16 == 0 && i + 1 < dataLen) {
            Serial.print("\n            ");
        }
    }
    Serial.print(" (");
    Serial.print(dataLen);
    Serial.println(" bytes)");
}

void displaySenderInfo(const esp_now_recv_info_t *recvInfo) {
    Serial.print("  Sender MAC: ");
    for (int i = 0; i < 6; i++) {
        Serial.printf("%02X", recvInfo->src_addr[i]);
        if (i < 5) Serial.print(":");
    }
    Serial.printf(" | RSSI: %d dBm\n", recvInfo->rx_ctrl->rssi);
}

// ============================================
// PACKET HANDLER
// ============================================

void handleAllServos(const uint8_t *data, int dataLen) {
    if (dataLen != sizeof(all_servos_packet_t)) {
        Serial.printf("  ❌ Invalid packet size! Expected %d bytes, got %d\n", 
                     sizeof(all_servos_packet_t), dataLen);
        errorCount++;
        return;
    }
    
    all_servos_packet_t *pkt = (all_servos_packet_t *)data;
    
    Serial.println("  📋 ALL 7 SERVOS");
    Serial.println("  Angles:");
    
    for (int i = 0; i < 7; i++) {
        Serial.printf("    Servo %d: %.1f°\n", i, pkt->angles[i]);
    }
    
    // TODO: Here you would write to actual servos
    // for (int i = 0; i < 7; i++) {
    //     moveServo(i, pkt->angles[i]);
    // }
}

// ============================================
// RECEIVE CALLBACK
// ============================================

void onDataReceive(const esp_now_recv_info_t *recvInfo, const uint8_t *data, int dataLen) {
    receivedCount++;
    
    Serial.println("\n" + String(70, '='));
    Serial.printf("Message #%lu Received | %d bytes\n", receivedCount, dataLen);
    Serial.println(String(70, '='));
    
    displaySenderInfo(recvInfo);
    displayRawPacket(data, dataLen);
    Serial.println("");
    
    // Check if packet size matches expected structure
    if (dataLen == sizeof(all_servos_packet_t)) {
        handleAllServos(data, dataLen);
    } else {
        Serial.printf("  ❌ Unknown packet type (size: %d bytes, expected: %d)\n", 
                     dataLen, sizeof(all_servos_packet_t));
        errorCount++;
    }
    
    Serial.println(String(70, '='));
}

void displayStats() {
    Serial.println("\n" + String(70, '-'));
    Serial.println("RECEPTION STATISTICS");
    Serial.println(String(70, '-'));
    Serial.printf("Total Messages:   %lu\n", receivedCount);
    Serial.printf("Errors:           %lu\n", errorCount);
    Serial.printf("Success Rate:     %.1f%%\n", 
                 receivedCount > 0 ? (100.0 * (receivedCount - errorCount) / receivedCount) : 0);
    Serial.println(String(70, '-'));
}

// ============================================
// INITIALIZATION
// ============================================

void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("\n\n" + String(70, '='));
    Serial.println("ESP-NOW RECEIVER - MODE 3 (ALL 7 SERVOS)");
    Serial.println(String(70, '='));
    
    // Show this device's MAC
    WiFi.mode(WIFI_STA);
    Serial.print("Device MAC: ");
    Serial.println(WiFi.macAddress());
    Serial.println("\nListening for ESP-NOW messages...\n");
    
    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("ERROR: esp_now_init failed");
        while (1) delay(1000);
    }
    
    // Register receive callback
    esp_now_register_recv_cb(onDataReceive);
    
    Serial.println(String(70, '='));
}

// ============================================
// MAIN LOOP
// ============================================

void loop() {
    // Display stats every 30 seconds if messages received
    static uint32_t lastStatsTime = 0;
    uint32_t now = millis();
    
    if (now - lastStatsTime > 30000) {
        lastStatsTime = now;
        if (receivedCount > 0) {
            displayStats();
        }
    }
    
    delay(100);
}
