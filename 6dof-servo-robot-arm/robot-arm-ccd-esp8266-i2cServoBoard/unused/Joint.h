#ifndef Joint_H
#define Joint_H

#include <iostream>
#include "Point.h"
#include <Servo.h>

class Joint {
    private:
        double absoluteAngleDeg; // Absolute angle in degrees
        double relativeAngleDeg; // Relative angle in degrees
        double length; // Length of the joint segment
        double vectorX; // X component of the joint vector
        double vectorY; // Y component of the joint vector
        double vectorStartX; // X component of the joint vector start point
        double vectorStartY; // Y component of the joint vector start point
        double vectorEndX; // X component of the joint vector end point
        double vectorEndY; // Y component of the joint vector end point
        Servo servo; // Servo controlling the joint
        Joint *parentJoint; // Pointer to the parent joint, if any
    public:
        Joint(double initialAbsoluteAngle, double length, Servo& servo);

        Joint(Joint* parentJoint, double initialAbsoluteAngle, double length, Servo& servo);

        double getAbsoluteAngleDeg();

        double getRelativeAngleDeg();

        double getAbsoluteAngleRad();

        double getRelativeAngleRad();

        double getVectorX();
        double getVectorY();

        double getVectorStartX();
        double getVectorStartY();

        double getVectorEndX();
        double getVectorEndY();

        double getLength();

        void updateAbsoluteAngleDeg(double newAbsoluteAngleDeg);
};

#endif
