#include "MotionControl.h"
#include "Config.h"
#include "Utilities.h"

Servo  servos[6];
double currentServoAngle[6] = {INIT_ANGLES[0],INIT_ANGLES[1],INIT_ANGLES[2],INIT_ANGLES[3],INIT_ANGLES[4],INIT_ANGLES[5]};
int8_t lastMoveDir[6]       = {0,0,0,0,0,0};
double baseTargetDeg        = INIT_ANGLES[0];

// ---------- ช่วย: map มุมฐาน → ไมโครวินาที แบบหนีบพัลส์ให้ปลอดภัย ----------
static inline int baseAngleToUs(double angleDeg){
  // หนีบมุมในกรอบ yaw ที่อนุญาต
  double a = clampBaseDeg(angleDeg);

  // map เชิงเส้นจาก [BASE_DEG_MIN..BASE_DEG_MAX] → [BASE_US_AT_DEG_MIN..BASE_US_AT_DEG_MAX]
  // ใช้ long*10 เพื่อลด error จาก float
  long a10 = (long)(a * 10.0);
  long us  = map(a10,
                 (long)(BASE_DEG_MIN * 10.0),
                 (long)(BASE_DEG_MAX * 10.0),
                 BASE_US_AT_DEG_MIN,
                 BASE_US_AT_DEG_MAX);

  // บังคับอยู่ในโซนพัลส์ที่เรากำหนดว่าปลอดภัย
  if (us < BASE_SAFE_MIN_US) us = BASE_SAFE_MIN_US;
  if (us > BASE_SAFE_MAX_US) us = BASE_SAFE_MAX_US;
  return (int)us;
}

void initServos(){
  // Base: positional servo → attach ด้วยช่วงกว้างพอให้ไปต่ำกว่า 70°
  servos[0].attach(ATTACH_PIN[0], BASE_ATTACH_MIN_US, BASE_ATTACH_MAX_US);

  // Other servos: ช่วงกว้างเดิม
  for(int i=1;i<6;++i){
    servos[i].attach(ATTACH_PIN[i], MIN_MICROSEC, MAX_MICROSEC);
  }

  // เขียนค่าเริ่มต้น: ฐานใช้ µs ตาม mapping, ที่เหลือใช้องศา
  servos[0].writeMicroseconds(baseAngleToUs(currentServoAngle[0]));
  for(int i=1;i<6;++i){
    servos[i].write(constrain(currentServoAngle[i], 0.0, 180.0));
  }

  baseTargetDeg = clampBaseDeg(currentServoAngle[0]);
}

bool anglesCloseTo(const double a[6], const double b[6], double eps){
  for(int i=0;i<6;++i){ if (fabs(a[i]-b[i])>eps) return false; }
  return true;
}

void moveTowardAnglesCappedSmooth(const double targetAngles[6],
                                  double maxStepDeg,
                                  int substeps,
                                  int subDelayMs)
{
  // 1) จำกัดเป้าหมายห่างจาก current ≤ maxStepDeg/แกน + no-reverse
  double clippedTarget[6];
  for(int i=0;i<6;++i){
    const double cur   = currentServoAngle[i];
    const double tgt   = targetAngles[i];
    const double delta = tgt - cur;
    const int    dir   = sgn(delta);

    if (lastMoveDir[i] != 0 && dir != 0 && dir != lastMoveDir[i] && fabs(delta) <= NO_REVERSE_DEADBAND_DEG) {
      clippedTarget[i] = cur; // ไม่ขยับแกนนี้
      continue;
    }

    double limited = cur + capDelta(delta, maxStepDeg);
    clippedTarget[i] = (i==0) ? clampBaseDeg(limited) : clampDeg(limited);

    if (fabs(clippedTarget[i] - cur) > 1e-6) lastMoveDir[i] = sgn(clippedTarget[i] - cur);
  }

  // 2) แตก sub-steps เพื่อความนุ่ม
  double start[6]; for(int i=0;i<6;++i) start[i]=currentServoAngle[i];

  for(int s=1; s<=substeps; ++s){
    const double a = (double)s / (double)substeps; // 0..1
    for(int i=0;i<6;++i){
      double inter = start[i] + (clippedTarget[i] - start[i]) * a;

      if(i==0){
        // ฐาน: เขียนเป็น µs ปลอดภัยตาม mapping ใหม่ → อนุญาตมุมต่ำกว่า 70°
        inter = clampBaseDeg(inter);
        servos[0].writeMicroseconds(baseAngleToUs(inter));
      } else {
        servos[i].write(constrain(inter, 0.0, 180.0));
      }

      currentServoAngle[i] = inter;
    }
    delay(subDelayMs); // yield() ในตัว (ESP8266)
  }
}
