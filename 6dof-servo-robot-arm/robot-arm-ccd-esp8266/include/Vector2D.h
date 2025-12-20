// Vector2D.h
#pragma once
#include <Arduino.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

struct Vector2D {
  double x, y;

  Vector2D() : x(0), y(0) {}
  Vector2D(double x_, double y_) : x(x_), y(y_) {}

  Vector2D add(const Vector2D& v)   const { return Vector2D(x + v.x, y + v.y); }
  Vector2D minus(const Vector2D& v) const { return Vector2D(x - v.x, y - v.y); }
  Vector2D scalarMultiply(double s) const { return Vector2D(x * s, y * s); }
  double   getMagnitude()           const { return sqrt(x * x + y * y); }
  double   dot(const Vector2D& v)   const { return x * v.x + y * v.y; }
  double   cross(const Vector2D& v) const { return x * v.y - y * v.x; }

  // Signed angle from this -> v (radians)
  double angleBetweenVector(const Vector2D& v) const {
    return atan2(cross(v), dot(v));
  }
};
