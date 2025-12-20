#include <Arduino.h>
#include "ApiServer.h"
#include "MotionControl.h"
#include "Config.h"

void setup() {
  Serial.begin(9600);
  initServos();                  // เซ็ตช่วงพัลส์ฐาน 1000..2000 และตัวอื่น 500..2500
  delay(5000);
  Serial.println("Robotic arm initialized.");
  setupServer();                 // รวม Wi-Fi + route ของ HTTP server
}

void loop() {
  handleServerLoop();            // server.handleClient();
}
