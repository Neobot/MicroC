#include "Robot.h"
#include "Comm.h"
#include "Parameters.h"

Robot::Robot() : pointSuivant(), position(), status(NOT_MOVING), deltaBodyPos(), posError(0), angleError(0), _comm(0),
	_leg{Leg(-PI_4, 	 94.3, 	-50.1, 1,  7, 13), 
		Leg(-PI_2, 	  0.0, 	-82.8, 2,  8, 14), 
		Leg(-3 * PI_4,-94.3,	-50.1, 3,  9, 15),
		Leg(3 * PI_4,	-94.3,	 50.1, 4, 10, 16), 
		Leg(PI_2, 	  0.0, 	 82.8, 5, 11, 17),
		Leg(PI_4,		 94.3,	 50.1, 6, 12, 18)}
{
	short legIndex;
	
	_leg[0].previousLeg = &_leg[5]; _leg[0].nextLeg = &_leg[1];
	_leg[1].previousLeg = &_leg[0]; _leg[1].nextLeg = &_leg[2];
	_leg[2].previousLeg = &_leg[1]; _leg[2].nextLeg = &_leg[3];
	_leg[3].previousLeg = &_leg[2]; _leg[3].nextLeg = &_leg[4];
	_leg[4].previousLeg = &_leg[3]; _leg[4].nextLeg = &_leg[5];
	_leg[5].previousLeg = &_leg[4]; _leg[5].nextLeg = &_leg[0];
	
	for (legIndex = 0; legIndex < 6; legIndex++)
	{
		_leg[legIndex].setHeight(50);
	}
}

void Robot::setComm(Comm *comm)
{
	_comm = comm;
}

void Robot::teleport(Point point)
{
    position = point;
	stop();
}

void Robot::ajoutPoint(Point point)
{ 
    queue.push(point);
}

void Robot::ajoutPoint(float x, float y)
{
	Point point;
	point.x = x;
	point.y = y;
	
	queue.push(point);
}

void Robot::flush()
{
    queue.clear();
}

void Robot::stop()
{
    flush();

	pointSuivant.x = position.x;
	pointSuivant.y = position.y;
	pointSuivant.theta = position.theta;
}

void Robot::calculateMovement(short cycleTimeUs)
{
	// calculate new body position and robot status
	float dx, dy;

	dx = pointSuivant.x - position.x;
	dy = pointSuivant.y - position.y;
	posError = sqrt(dx * dx + dy * dy);
	angleError = atan2(dy, dx) - position.theta;
	
	if (angleError > 0.01) {
		status = TURNING_CW;
		deltaBodyPos.x = 0;
		deltaBodyPos.y = 0;
		deltaBodyPos.z = 0;
		deltaBodyPos.theta = speed * cycleTimeUs / 1000000 / 250;
	}
	else if (angleError < -0.01) {
		status = TURNING_CCW;
		deltaBodyPos.x = 0;
		deltaBodyPos.y = 0;
		deltaBodyPos.z = 0;
		deltaBodyPos.theta = -speed * cycleTimeUs / 1000000 / 250;
	}
	else if (posError > 2) {
		status = WALKING;
		deltaBodyPos.x = speed * cycleTimeUs / 1000000 * cos(position.theta);
		deltaBodyPos.y = speed * cycleTimeUs / 1000000 * sin(position.theta);
		deltaBodyPos.z = 0;
		deltaBodyPos.theta = 0;		
	}
	else
		status = NOT_MOVING;
}

void Robot::processGait(short cycleTimeUs)
{
	short legIndex;
	
	if (status == NOT_MOVING && !queue.isEmpty())
		pointSuivant = queue.pop();
	
	calculateMovement(cycleTimeUs);

	if (status == TURNING_CW)
	{
		for(legIndex = 0; legIndex < 6; legIndex++)
		{
			if (_leg[legIndex].lagging() && !_leg[legIndex].previousLeg->lifted && !_leg[legIndex].nextLeg->lifted)
			{
				float stepSize = (angleError > maxStepRotation) ? maxStepRotation : max(angleError - maxStepRotation/2, 0.1);
				_leg[legIndex].stepRotate(true, stepSize);
			}
		}
	}
	else if (status == TURNING_CCW)
	{
		for(legIndex = 0; legIndex < 6; legIndex++)
		{
			if (_leg[legIndex].lagging() && !_leg[legIndex].previousLeg->lifted && !_leg[legIndex].nextLeg->lifted)
			{
				float stepSize = (angleError > maxStepRotation) ? maxStepRotation : max(angleError - maxStepRotation/2, 0.1);
				_leg[legIndex].stepRotate(false, stepSize);
			}
		}
	}
	else if (status == WALKING)	// walk
	{
		for(legIndex = 0; legIndex < 6; legIndex++)
		{
			if (_leg[legIndex].lagging() && !_leg[legIndex].previousLeg->lifted && !_leg[legIndex].nextLeg->lifted)
			{
				float stepSize = (posError > maxStepSize) ? maxStepSize : max(posError - maxStepSize/2, 15);
				_leg[legIndex].step(0, stepSize);
			}
		}
	}
	else	// arrived, center legs
	{
		for(legIndex = 0; legIndex < 6; legIndex++)
		{
			if (!_leg[legIndex].centered() && !_leg[legIndex].previousLeg->lifted && !_leg[legIndex].nextLeg->lifted)
			{
				_leg[legIndex].step(0, 0);
			}
		}
	}
	
	for (legIndex = 0; legIndex < 6; legIndex++) {
		if (_leg[legIndex].lifted)
			_leg[legIndex].interpolateStep(cycleTimeUs);
	}
	
	if (status == NOT_MOVING)
	{
		commitToServos(false);
	}
	else if (!moveBody())
	{
		#ifdef ENABLE_SERVOS
		commitToServos(false);	// move only lifted _leg
		#endif
		
		#ifdef ENABLE_DEBUG
		//SerialDebug.println("Coordinate not reachable");
		#endif
	}
	else
	{
		#ifdef ENABLE_SERVOS
		commitToServos(true);		// move legs + body
		#endif
		
		position += deltaBodyPos;	// update robot position
	}
}

bool Robot::initLegs()
{
	// move to initial position
	short legIndex;
	
	for (legIndex = 0; legIndex < 6; legIndex++)
	{
		_leg[legIndex].lower(5000);
		_leg[legIndex].inverseKinematics();
	}
	
	commitToServos(true);
	
	return !_leg[0].lifted;	// return true when done
}

void Robot::commitToServos(bool moveBody)
{
	short legIndex;

	for (legIndex = 0; legIndex < 6; legIndex++)
	{
		if (_leg[legIndex].lifted || moveBody)
		{
			_leg[legIndex].commitToServos();
		}
	}
	Dynamixel.action();
}

bool Robot::moveBody()
{
	short legIndex;
	bool moveOk = true;
	
	for (legIndex = 0; legIndex < 6; legIndex++)
	{
		if (!_leg[legIndex].lifted)
		{
			_leg[legIndex].moveBody(deltaBodyPos);
			
			if (!_leg[legIndex].inverseKinematics())
				moveOk = false;
		}
	}
	
	return moveOk;	// returns false if position not reachable
}

