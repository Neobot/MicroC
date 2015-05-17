#ifndef POINT_H
#define POINT_H

#include "Tools.h"

class Point 
{
public:
    Point();
    Point(float xx, float yy, float zz, float ttheta = 0);
    Point& operator+=(const Point& p);
    const Point operator+(const Point &p) const;
    Point& operator-=(const Point& p);
    const Point operator-(const Point &p) const;

    float x;
    float y;
    float z;
    float theta;
};

#endif // POINT_H

