#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include "QueueList.h"
#include "Point.h"
#include "Leg.h"
#include "DynamixelSerial.h"

class Comm;

class Robot
{
  public:
	Robot();
	
	enum HexStatus {
		WALKING,
		TURNING_CW,
		TURNING_CCW,
		NOT_MOVING
	};
	
	void setComm(Comm* comm);

    void teleport(Point point);
    void ajoutPoint(Point point);
    void ajoutPoint(float x, float y);
    void flush();
    void stop();
    void calculateMovement(short cycleTimeUs);
    void processGait(short cycleTimeUs);
	bool initLegs();
	void commitToServos(bool moveBody);
	bool moveBody();

    QueueList<Point> 	queue;
    Point 				pointSuivant;
    Point 				position;
    HexStatus 			status;
    Point				deltaBodyPos;
	float				posError;
	float				angleError;
	
private:
	Comm* 				_comm;
	Leg					_leg[6];

};

#endif // ROBOT_H


