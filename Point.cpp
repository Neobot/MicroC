#include "Arduino.h"
#include "Point.h"

Point::Point() : x(0), y(0), theta(0), pointArret(false), typeAsserv(Auto), typeDeplacement(TourneEtAvance), vitessMax(100.0) {}

MPoint* Point::getMPoint()
{
    MPoint *p = new MPoint();
    
    p->_x = x;
    p->_y = y;
    
    return p;
}