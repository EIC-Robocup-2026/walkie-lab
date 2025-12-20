#pragma once
#include <Arduino.h>
#include "Config.h"

#ifndef M_PI
  #define M_PI 3.14159265358979323846
#endif
#ifndef M_PI_2
  #define M_PI_2 1.57079632679489661923
#endif

static inline double clampDeg(double x){ return constrain(x, -50.0, 230.0); }       // joints/roll/grip
static inline double clampBaseDeg(double d){ return constrain(d, YAW_MIN_DEG, YAW_MAX_DEG); } // base
static inline double deg2rad(double d){ return d * M_PI / 180.0; }
static inline double rad2deg(double r){ return r * 180.0 / M_PI; }
static inline double capDelta(double d, double cap){ return (d > cap) ? cap : (d < -cap ? -cap : d); }
static inline int sgn(double x){ return (x>0) - (x<0); }

// CSV parser: 8 ค่า คั่นด้วย comma
static inline bool parse8CSV(const String& s, float out[8]){
  int start=0, idx;
  for(int i=0;i<7;++i){
    idx = s.indexOf(',', start);
    if(idx<0) return false;
    out[i] = s.substring(start, idx).toFloat();
    start  = idx + 1;
  }
  out[7] = s.substring(start).toFloat();
  return true;
}
