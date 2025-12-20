#include "ApiServer.h"
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include "Config.h"
#include "Utilities.h"
#include "MotionControl.h"
#include "ArmKinematics.h"

static ESP8266WebServer server(80);

static void sendCORS(){
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type,X-API-Key");
  server.sendHeader("Access-Control-Allow-Methods", "GET,POST,OPTIONS");
}
static bool checkApiKey(){
  if (!API_KEY || strlen(API_KEY)==0) return true;
  return server.header("X-API-Key") == API_KEY;
}

static void handleRoot(){
  sendCORS();
  String html =
    "<html><body><h3>Robot Arm API (ESP8266)</h3>"
    "<p>POST <code>/control</code> (text/plain): <b>dx,dy,dz,roll,pitch,yaw,gripper_intensity,mission_status</b></p>"
    "<p>GET <code>/status</code> — angles & EE</p>"
    "<p>GET/POST <code>/reset</code> — smooth reset to HOME (≤5°/axis per step)</p>"
    "</body></html>";
  server.send(200, "text/html", html);
}
static void handleOptions(){ sendCORS(); server.send(204); }

static void handleStatus(){
  sendCORS();
  FK2D fk = computeFK_fromAngles(currentServoAngle);
  String json = "{";
  json += "\"angles\":[" + String(currentServoAngle[0],1)+","+String(currentServoAngle[1],1)+","+String(currentServoAngle[2],1)+","+String(currentServoAngle[3],1)+","+String(currentServoAngle[4],1)+","+String(currentServoAngle[5],1)+"],";
  json += "\"ee\":{\"x\":"+String(fk.EE.x,1)+",\"z\":"+String(fk.EE.y,1)+"},";
  json += "\"uptime_ms\":"+String(millis())+"}";
  server.send(200, "application/json", json);
}

static void handleControl(){
  sendCORS();
  if(!checkApiKey()){ server.send(401,"application/json","{\"error\":\"unauthorized\"}"); return; }

  String body = server.arg("plain"); body.trim();
  float v[8];
  if(!parse8CSV(body,v)){
    server.send(400,"application/json","{\"error\":\"invalid_format\",\"hint\":\"dx,dy,dz,roll,pitch,yaw,gripper,mission\"}");
    return;
  }

  float dx=v[0], dy=v[1], dz=v[2], roll=v[3], pitch=v[4], yaw=v[5], grip=v[6], status=v[7];

  // --- Base target update with deadband & limits ---
  if (fabs(dy) >= DY_DEADBAND) {
    baseTargetDeg = clampBaseDeg(baseTargetDeg + dy * VLA_BASE_ROTATION_GAIN);
  }

  // 1) เป้าปลายแขน (2D x,z)
  Vector2D dV = Vector2D(dx, dz).scalarMultiply(VLA_ARM_PROPORTIONAL_GAIN);
  FK2D fkNow = computeFK_fromAngles(currentServoAngle);
  Vector2D target = fkNow.EE.add(dV);

  // 2) CCD บนสำเนา → desiredAngles
  double desiredAngles[6]; for(int i=0;i<6;++i) desiredAngles[i]=currentServoAngle[i];
  runCCD_onAngles(target, desiredAngles); // ปรับ [1..3]

  desiredAngles[3] = clampDeg(desiredAngles[3] - pitch /* * gain ถ้าต้องการ */);

  // 3) เติมแกนอื่น
  desiredAngles[0] = baseTargetDeg;                                       // base absolute
  desiredAngles[4] = currentServoAngle[4] + roll * VLA_GRIPPER_ROTATION_GAIN;
  desiredAngles[5] = (1 - grip) * 90.0;                                   // absolute 0..90

  // 4) ขยับจริงแบบนุ่ม + ≤5°/แกน + no-reverse
  moveTowardAnglesCappedSmooth(desiredAngles, MAX_STEP_DEG, SMOOTH_SUBSTEPS, SMOOTH_SUBSTEP_DELAY_MS);

  // 5) ตอบกลับ
  FK2D fkNew = computeFK_fromAngles(currentServoAngle);
  String json="{\"ok\":true,\"angles\":["
    +String(currentServoAngle[0],1)+","+String(currentServoAngle[1],1)+","+String(currentServoAngle[2],1)+","
    +String(currentServoAngle[3],1)+","+String(currentServoAngle[4],1)+","+String(currentServoAngle[5],1)
    +"],\"ee\":{\"x\":"+String(fkNew.EE.x,1)+",\"z\":"+String(fkNew.EE.y,1)+"}}";
  server.send(200,"application/json",json);
}

static void handleReset(){
  sendCORS();
  if(!checkApiKey()){ server.send(401,"application/json","{\"error\":\"unauthorized\"}"); return; }

  // รีเซ็ต state สำหรับ no-reverse และ base target
  for(int i=0;i<6;++i) lastMoveDir[i] = 0;
  baseTargetDeg = clampBaseDeg(HOME_ANGLES[0]);

  // ค่อย ๆ เดินกลับบ้านด้วยก้าวละ ≤5°/แกน (ลูปสั้น)
  int guard = 0;
  while(!anglesCloseTo(currentServoAngle, HOME_ANGLES) && guard < 200){
    moveTowardAnglesCappedSmooth(HOME_ANGLES, MAX_STEP_DEG, SMOOTH_SUBSTEPS, SMOOTH_SUBSTEP_DELAY_MS);
    guard++;
  }

  FK2D fk = computeFK_fromAngles(currentServoAngle);
  String json="{\"ok\":true,\"reset\":true,\"angles\":["
    +String(currentServoAngle[0],1)+","+String(currentServoAngle[1],1)+","+String(currentServoAngle[2],1)+","
    +String(currentServoAngle[3],1)+","+String(currentServoAngle[4],1)+","+String(currentServoAngle[5],1)
    +"],\"ee\":{\"x\":"+String(fk.EE.x,1)+",\"z\":"+String(fk.EE.y,1)+"}}";
  server.send(200,"application/json",json);
}

void setupServer(){
  // Wi-Fi
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("Connecting to WiFi SSID %s ...\n", WIFI_SSID);
  int tries=0; while(WiFi.status()!=WL_CONNECTED && tries<60){ delay(500); Serial.print("."); ++tries; }
  Serial.println();
  if(WiFi.status()==WL_CONNECTED) Serial.printf("WiFi connected. IP: %s\n", WiFi.localIP().toString().c_str());
  else { Serial.println("WiFi connect failed. Rebooting in 5s..."); delay(5000); ESP.restart(); }

  // Routes
  server.on("/",        HTTP_GET,     handleRoot);
  server.on("/status",  HTTP_GET,     handleStatus);
  server.on("/control", HTTP_OPTIONS, handleOptions);
  server.on("/control", HTTP_POST,    handleControl);
  server.on("/reset",   HTTP_OPTIONS, handleOptions);
  server.on("/reset",   HTTP_GET,     handleReset);
  server.on("/reset",   HTTP_POST,    handleReset);

  server.begin();
  Serial.println("HTTP server started.");
}

void handleServerLoop(){
  server.handleClient();
}
