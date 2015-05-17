#include "Arduino.h"
#include "Point.h"

Point::Point() : x(0), y(0), z(0), theta(0) {}

Point::Point(float xx, float yy, float zz, float ttheta) : x(xx), y(yy), z(zz), theta(ttheta) {}

Point& Point::operator+=(const Point& p)
{
	this->x += p.x;
	this->y += p.y;
	this->z += p.z;
	this->theta += p.theta;
	this->theta = Tools::trimAngle(this->theta);
	
	return *this;
}

const Point Point::operator+(const Point &p) const
{
	Point result = *this;
    result += p;
    
    return result;
}

Point& Point::operator-=(const Point& p)
{
	this->x -= p.x;
	this->y -= p.y;
	this->z -= p.z;
	this->theta -= p.theta;
	this->theta = Tools::trimAngle(this->theta);
	
	return *this;
}

const Point Point::operator-(const Point &p) const
{
	Point result = *this;
    result -= p;
    
    return result;
}