#pragma once

// ---- PWM range (global default for non-base servos) ----
#define MIN_MICROSEC 500
#define MAX_MICROSEC 2500

// ---- Gains / CCD ----
#define VLA_ARM_PROPORTIONAL_GAIN 500
#define VLA_BASE_ROTATION_GAIN    1
#define VLA_GRIPPER_ROTATION_GAIN 1
#define CCD_ITERATION             200

// ---- Wrist pitch offset (add after CCD) ----
#define PITCH_CONST_OFFSET_DEG  10.0   // ปรับตามต้องการ (+ คือเชิดปลายขึ้น)

// ---- Link lengths (mm) ----
#define l1 105
#define l2 125
#define l3 120

// ---- Smoothing & per-command step limit ----
#define MAX_STEP_DEG            5.0   // จำกัดก้าวต่อ "คำสั่ง" ต่อแกน (deg)
#define SMOOTH_SUBSTEPS         5     // แตก sub-steps
#define SMOOTH_SUBSTEP_DELAY_MS 15    // ms ต่อ sub-step

// ---- No-reverse (กันกลับทิศจุกจิก) ----
#define NO_REVERSE_DEADBAND_DEG 1.0   // 0.5–2.0 แนะนำ

// ---- Base yaw control constraints ----
// อนุญาตให้ลงต่ำกว่า 70° ได้แล้ว → ลด min ลง (เช่น 40°)
// ถ้าต่ำกว่านี้ยังไม่นิ่ง ค่อยยกค่านี้ขึ้นทีละน้อย
#define YAW_MIN_DEG 40.0
#define YAW_MAX_DEG 160.0
#define DY_DEADBAND 1.0               // deadband อินพุต dy (deg)

// ---- Base servo (positional) dedicated attach pulse range ----
// ขยายช่วง attach เพื่อรองรับมุมต่ำกว่า 70° (จะจูนพัลส์จริงตอนเขียนอีกชั้น)
#define BASE_ATTACH_MIN_US 900
#define BASE_ATTACH_MAX_US 2100

// ---- Mapping มุมฐาน → ไมโครวินาที (คาลิเบรตได้) ----
// กำหนด "ปลายช่วงเชิงมุม" ที่อยากได้ กับพัลส์ที่ส่งจริง (เชิงเส้น)
// ค่าเริ่มต้นทั่วไป: 0°≈1000us, 180°≈2000us
#define BASE_DEG_MIN         0.0
#define BASE_DEG_MAX         180.0
#define BASE_US_AT_DEG_MIN   1000
#define BASE_US_AT_DEG_MAX   2000

// ---- Safe guard โซนพัลส์ที่ไว/รวนของ servo บางรุ่น ----
// ถ้าลงต่ำกว่า 70° แล้วยังรวน ให้เพิ่ม BASE_SAFE_MIN_US (เช่น 1040→1060→1080)
#define BASE_SAFE_MIN_US     1020
#define BASE_SAFE_MAX_US     1980

// ---- Wi-Fi ----
static const char* WIFI_SSID = "EIC_2G";
static const char* WIFI_PASS = "EicChula";
static const char* API_KEY   = "";    // ตั้งว่าง = ไม่ตรวจ

// ---- Pins / initial angles ----
static const int ATTACH_PIN[6] = {5,4,0,14,12,13}; // D1,D2,D3,D5,D6,D7
static const double INIT_ANGLES[6] = {90,90,90,30,90,0};
static const double HOME_ANGLES[6] = {90,90,90,30,90,0};
