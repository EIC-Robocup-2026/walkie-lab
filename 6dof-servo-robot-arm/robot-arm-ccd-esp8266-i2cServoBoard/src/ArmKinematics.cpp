#include "ArmKinematics.h"
#include "Utilities.h"
#include "Config.h"

FK2D computeFK_raw(double a2_deg, double a3_deg, double a4_deg){
  double t2 = deg2rad(a2_deg), t3 = deg2rad(a3_deg), t4 = deg2rad(a4_deg);
  FK2D k;
  k.link1 = Vector2D(-l1 * cos(t2),                   l1 * sin(t2));
  k.link2 = Vector2D( l2 * cos(3 * M_PI_2 - t3 - t2), l2 * sin(3 * M_PI_2 - t3 - t2));
  k.link3 = Vector2D( l3 * cos(M_PI + t4 - t3 - t2),  l3 * sin(M_PI + t4 - t3 - t2));
  k.EE    = k.link1.add(k.link2).add(k.link3);
  return k;
}

FK2D computeFK_fromAngles(const double ang[6]){
  return computeFK_raw(ang[1], ang[2], ang[3]);
}

void runCCD_onAngles(const Vector2D& target, double ang[6]){
  uint8_t state = 3;
  for(int it=0; it<CCD_ITERATION; ++it){
    FK2D fk = computeFK_raw(ang[1], ang[2], ang[3]);
    Vector2D e1=fk.EE, e2=fk.link2.add(fk.link3), e3=fk.link3;
    Vector2D t1=target, t2=target.minus(fk.link1), t3=target.minus(fk.link1.add(fk.link2));
    switch(state){
      case 1:{ double d=e1.getMagnitude()<1e-6?0:e1.angleBetweenVector(t1);
               ang[1]=clampDeg(ang[1]-rad2deg(d)); state=3; break; }
      case 2:{ double d=e2.getMagnitude()<1e-6?0:e2.angleBetweenVector(t2);
               ang[2]=clampDeg(ang[2]-rad2deg(d)); state=1; break; }
      case 3:{ double d=e3.getMagnitude()<1e-6?0:e3.angleBetweenVector(t3);
               ang[3]=clampDeg(ang[3]+rad2deg(d)); state=2; break; }
    }
  }
}
