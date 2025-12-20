#include "Joint.h"
#include "Point.h"
#include <cmath>

Joint::Joint(double initialAbsoluteAngleDeg, double length  , Servo& servo) {
    absoluteAngleDeg = initialAbsoluteAngleDeg;
    relativeAngleDeg = initialAbsoluteAngleDeg - 90;
    length = length;
    vectorStartX = 0.0; // Assuming the start point is at origin if no parent joint
    vectorStartY = 0.0; // Assuming the start point is at origin if no parent joint
    vectorEndX = length * cos(getAbsoluteAngleRad());
    vectorEndY = length * sin(getAbsoluteAngleRad());
    vectorX = vectorEndX - vectorStartX;
    vectorY = vectorEndY - vectorStartY;
    servo = servo; // Initialize the servo controlling this joint
}

Joint::Joint(Joint* parentJoint, double initialRelativeAngleDeg, double length, Servo& servo) {
    absoluteAngleDeg = parentJoint->getAbsoluteAngleDeg() + initialRelativeAngleDeg;
    relativeAngleDeg = initialRelativeAngleDeg;
    length = length;
    vectorStartX = parentJoint->getVectorEndX();
    vectorStartY = parentJoint->getVectorEndY();
    vectorEndX = vectorStartX + length * cos(getAbsoluteAngleRad());
    vectorEndY = vectorStartY + length * sin(getAbsoluteAngleRad());
    vectorX = vectorEndX - vectorStartX;
    vectorY = vectorEndY - vectorStartY;
    servo = servo; // Initialize the servo controlling this joint
    this->parentJoint = parentJoint; // Set the parent joint
}

double Joint::getAbsoluteAngleDeg() { 
    return absoluteAngleDeg;
}

double Joint::getRelativeAngleDeg() { 
    return relativeAngleDeg; 
}

double Joint::getAbsoluteAngleRad() {
    return absoluteAngleDeg * (M_PI / 180.0);
}

double Joint::getRelativeAngleRad() {
    return relativeAngleDeg * (M_PI / 180.0);
}

double Joint::getVectorX() {
    return length * cos(getAbsoluteAngleRad());
}

double Joint::getVectorY() {
    return length * sin(getAbsoluteAngleRad());
}

double Joint::getVectorStartX() {
    return vectorStartX;
}

double Joint::getVectorStartY() {
    return vectorStartY;
}

double Joint::getVectorEndX() {
    return vectorEndX;
} 

double Joint::getVectorEndY() {
    return vectorEndY;
}

double Joint::getLength() {
    return length;
}

void Joint::updateAbsoluteAngleDeg(double newAbsoluteAngleDeg) {
    absoluteAngleDeg = newAbsoluteAngleDeg;
    vectorEndX = vectorStartX + length * cos(getAbsoluteAngleRad());
    vectorEndY = vectorStartY + length * sin(getAbsoluteAngleRad());
    vectorX = vectorEndX - vectorStartX;
    vectorY = vectorEndY - vectorStartY;
    relativeAngleDeg = servo.read() - 90 - absoluteAngleDeg; // Update relative angle based on servo position
}

