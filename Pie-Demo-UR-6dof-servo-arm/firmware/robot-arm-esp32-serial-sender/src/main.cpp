/*
    ESP-NOW UART -> ESP-NOW forwarder

    Receives a single newline-terminated line on UART that contains
    seven comma-separated angles (floats), validates them (0-180), and
    forwards them in a compact binary packet via ESP-NOW to the
    configured `receiverMAC`.

    Input Format (UART): angle1,angle2,...,angle7\n
    Example: 90.0,45.5,135.0,90.0,45.0,120.5,80.0

    This file is trimmed of the previous interactive/testing commands
    so it is intended for production use where a host sends lines
    over a serial/UART connection.
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
// 40:4C:CA:5E:CC:5C
uint8_t receiverMAC[6] = {0x40, 0x4C, 0xCA, 0x5E, 0xCC, 0x5C};  // <-- set to receiver MAC

// Servo angle range (adjustable)
#ifndef SERVO_MIN_ANGLE
#define SERVO_MIN_ANGLE 0.0f
#endif
#ifndef SERVO_MAX_ANGLE
#define SERVO_MAX_ANGLE 270.0f
#endif

// Global variables
uint32_t sentCount = 0;
esp_now_peer_info_t peerInfo = {};

// ============================================
// CALLBACK
// ============================================

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    sentCount++;
    Serial.printf("Packet #%lu send %s\n", sentCount, 
                  status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

// ============================================
// SEND FUNCTION
// ============================================

void sendAllServos(float angles[7]) {
    all_servos_packet_t packet;

    for (int i = 0; i < 7; i++) {
        if (angles[i] < SERVO_MIN_ANGLE || angles[i] > SERVO_MAX_ANGLE) return; // silently drop invalid packet
        packet.angles[i] = angles[i];
    }

    esp_err_t res = esp_now_send(receiverMAC, (uint8_t *)&packet, sizeof(packet));
    (void)res; // callback reports status
}

// ============================================
// INITIALIZATION
// ============================================

void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("\n--- ESP-NOW UART -> ESP-NOW forwarder ---\n");

    // Set WiFi to station mode (required for ESP-NOW)
    WiFi.mode(WIFI_STA);
    Serial.print("Device MAC: ");
    Serial.println(WiFi.macAddress());

    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("ERROR: esp_now_init failed");
        while (1) delay(1000);
    }

    esp_now_register_send_cb(onDataSent);

    // Add receiver as peer
    memcpy(peerInfo.peer_addr, receiverMAC, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("ERROR: Failed to add peer!");
        while (1) delay(1000);
    }

    Serial.println("Peer added");

    // Warn if receiverMAC looks like the default broadcast placeholder
    bool isBroadcast = true;
    for (int i = 0; i < 6; ++i) if (receiverMAC[i] != 0xFF) isBroadcast = false;
    if (isBroadcast) {
        Serial.println("WARNING: receiverMAC is set to broadcast (0xFF:..). Update receiverMAC before deployment.");
    }

    Serial.println();
}

// ============================================
// MAIN LOOP
// ============================================

void loop() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();

        if (input.length() == 0) return;

        // Copy into a mutable buffer for strtok
        char buf[192];
        input.toCharArray(buf, sizeof(buf));

        float angles[7];
        int idx = 0;
        char *saveptr = NULL;
        char *tok = strtok_r(buf, ",", &saveptr);

        while (tok != NULL && idx < 7) {
            // strtod handles leading/trailing spaces
            float v = (float)strtod(tok, NULL);
            angles[idx++] = v;
            tok = strtok_r(NULL, ",", &saveptr);
        }

        if (idx != 7) {
            Serial.println("ERR: expected 7 comma-separated angles");
        } else {
            bool ok = true;
            for (int i = 0; i < 7; ++i) {
                if (angles[i] < SERVO_MIN_ANGLE || angles[i] > SERVO_MAX_ANGLE) { ok = false; break; }
            }
            if (!ok) {
                Serial.println("ERR: one or more angles out of range (0-180)");
            } else {
                sendAllServos(angles);
            }
        }
    }

    delay(10);
}
