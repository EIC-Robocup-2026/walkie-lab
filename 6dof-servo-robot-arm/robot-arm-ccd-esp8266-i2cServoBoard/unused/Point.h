#ifndef Point_H
#define Point_H

#include <iostream>

class Point {
    private:
        double x; 
        double y;
        double z;
    public:
        Point(double x, double y, double z); //3D point constructor

        double getX();
        double getY();
        double getZ();
};

#endif