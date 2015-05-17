#ifndef LEG_H
#define LEG_H

#include "Arduino.h"
#include "Point.h"

class Leg 
{
private:
	float 	_angle;	// angle of leg in radian (relative to body)
	float	_coxaX, _coxaY;	// position of coxa relative to body centre in mm
	float	_coxaMin, _coxaMax, _femurMin, _femurMax, _tibiaMin, _tibiaMax;	// min/max angle for each join in rad
    Point 	_goal; // in mm
    Point 	_offset; // in mm
    float	_stepDirection; // in rad
    Point	_initPos;	// leg rest position
    Point 	_relPos;
    int		_coxaServoId, _femurServoId, _tibiaServoId;
    float 	_coxaAngle, _femurAngle, _tibiaAngle;
    float 	_newCoxaAngle, _newFemurAngle, _newTibiaAngle;
    bool	_readyToCommit;
	
public:
    Leg(float angle, float x, float y, int coxaServoId, int femurServoId, int tibiaServoId);
    bool 	inverseKinematics();
    void	interpolateStep(float deltaT);
    void	moveBody(Point delta);
    void	lower(float deltaT);
    void	step(float direction, float stepSize);
    void	stepRotate(bool direction, float stepSize);
    void	setHeight(int z);
    bool	lagging();
    bool	centered();
    void	commitToServos();
    boolean	lifted;
    Leg		*previousLeg;
    Leg		*nextLeg;
};

#endif // LEG_H

