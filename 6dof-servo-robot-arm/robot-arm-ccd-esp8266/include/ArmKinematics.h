#pragma once
#include "Types.h"

FK2D computeFK_raw(double a2_deg, double a3_deg, double a4_deg);
FK2D computeFK_fromAngles(const double ang[6]);
void  runCCD_onAngles(const Vector2D& target, double ang[6]);
