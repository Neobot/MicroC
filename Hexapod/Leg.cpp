#include "Leg.h"
#include "Parameters.h"
#include "Tools.h"
#include "DynamixelSerial.h"

Leg::Leg(float angle, float x, float y, int coxaServoId, int femurServoId, int tibiaServoId) : _angle(angle), _coxaX(x), _coxaY(y), 
	_coxaMin(-0.972), _coxaMax(0.972), _femurMin(-0.164), _femurMax(1.227), _tibiaMin(-2.618), _tibiaMax(-0.061),
	_goal(), _offset(), _stepDirection(0), _coxaServoId(coxaServoId), _femurServoId(femurServoId), _tibiaServoId(tibiaServoId),
	_coxaAngle(0), _femurAngle(0.34), _tibiaAngle(-0.42), _newCoxaAngle(0), _newFemurAngle(0.34), _newTibiaAngle(-0.42), 
	_readyToCommit(false), lifted(false)
{
	_initPos = Point(initCoxaFootDist * cosf(_angle),
					initCoxaFootDist * sinf(_angle), 
					coxaZOffset - initBodyHeight,
					0.0f);
	
	_relPos = Point(_initPos.x, _initPos.y, _initPos.z, 0.0f);
}

/* 
calculates servo angles based on previously defined foot position
returns false if point not reachable
*/
bool Leg::inverseKinematics()
{
	float coxaFootDist = sqrt(square(_relPos.x) + square(_relPos.y));	// distance in (x, y) plan

	float sw, a1, a2, ta;
	
	sw = sqrt(square(coxaFootDist - coxaLength) + square(_relPos.z));	// dist from femur axis and foot in (x, y) plane
	a1 = atan2(_relPos.z, coxaFootDist - coxaLength);					// angle between horiz plane and sw
	a2 = acos((square(tibiaLength)-square(femurLength)-square(sw))/(-2 * sw * femurLength));	// angle between sw and femur
	ta = acos((square(sw)-square(tibiaLength)-square(femurLength))/(-2 * tibiaLength * femurLength));	// tibia angle

	_newCoxaAngle = Tools::trimAngle(_angle - atan2(_relPos.y, _relPos.x));
	_newFemurAngle = Tools::trimAngle(a1 + a2 - PI_4);
	_newTibiaAngle = Tools::trimAngle(-ta + 0.422);
	_readyToCommit = (_newCoxaAngle > _coxaMin && _newCoxaAngle < _coxaMax && _newFemurAngle > _femurMin && _newFemurAngle < _femurMax && _newTibiaAngle > _tibiaMin && _newTibiaAngle < _tibiaMax);

	return _readyToCommit;
}

void Leg::interpolateStep(float deltaT)
{
	float delta = speed * deltaT / 1000000 * 1.5;

	// move on (x,y) plane
	_relPos.x += delta * cos(_stepDirection);
	_relPos.y += delta * sin(_stepDirection);
	
	// project current position against vector representing whole movement
	float progress = ((_relPos.x - _goal.x + _offset.x) * _offset.x + (_relPos.y - _goal.y + _offset.y) * _offset.y) / (_offset.x * _offset.x + _offset.y * _offset.y);
	
	if (progress < 0.5)			// raise
		_relPos.z = _initPos.z + stepHeight * progress * 2;
	else if (progress > 0.5)	// lower
		_relPos.z = _initPos.z + stepHeight * (1 - progress) * 2;
		
	// end of step
	if (progress >= 1)
	{
		_relPos.x = _goal.x;
		_relPos.y = _goal.y;
		_relPos.z = _initPos.z;
		lifted = false;
	}

	inverseKinematics();
}

void Leg::moveBody(Point delta)
{
	if (delta.theta != 0)	// rotate
	{
		float cosine = cos(-delta.theta);
		float sine = sin(-delta.theta);
	
		float footToCenterPosX = _relPos.x + _coxaX;
		float footToCenterPosY = _relPos.y + _coxaY;
	
		float newFootToCenterPosX = footToCenterPosX * cosine - footToCenterPosY * sine;
		float newFootToCenterPosY = footToCenterPosX * sine + footToCenterPosY * cosine;

		_relPos.x = newFootToCenterPosX - _coxaX;
		_relPos.y = newFootToCenterPosY - _coxaY;
		_relPos.z = _initPos.z;
	
		if (delta.theta > 0)
			_stepDirection = _angle + PI_2;
		else
			_stepDirection = _angle - PI_2;
	}
	else
	{
		_relPos.x -= sqrt(square(delta.x)+square(delta.y));
		_relPos.z = _initPos.z;
		_stepDirection = 0;
	}
}

void Leg::lower(float deltaT)
{
	_relPos.z -= speed * deltaT / 1000000;
	Serial2.println(_relPos.z);
	if (_relPos.z < _initPos.z) {
		_relPos.z = _initPos.z;
		lifted = false;
	}
}

void Leg::step(float direction, float stepSize)
{
	// find destination coordinate for foot within reachable area
	// (simplified as circle of radius 50mm centered on default leg position)

	_goal.x = _initPos.x + stepSize * cos(direction);
	_goal.y = _initPos.y + stepSize * sin(direction);
	_offset.x = _goal.x - _relPos.x;
	_offset.y = _goal.y - _relPos.y;
	_stepDirection = direction;
	lifted = true;
}

void Leg::stepRotate(bool direction, float stepSize)
{
	float angle = (direction ? stepSize : -stepSize);	// rotation angle for step
	float cosine = cos(angle);
	float sine = sin(angle);
	
	// new position = default position rotated by 0.5 rad
	float newFootToCenterPosX = (_initPos.x + _coxaX) * cosine - (_initPos.y + _coxaY) * sine;
	float newFootToCenterPosY = (_initPos.x + _coxaX) * sine + (_initPos.y + _coxaY) * cosine;
	
	_goal.x = newFootToCenterPosX - _coxaX;
	_goal.y = newFootToCenterPosY - _coxaY;
	
	_offset.x = _goal.x - _relPos.x;
	_offset.y = _goal.y - _relPos.y;
	_stepDirection = atan2(_offset.y, _offset.x);
	lifted = true;
}

void Leg::setHeight(int z)
{
	if (z > 0) {
		_relPos.z = z + _initPos.z;
	
		lifted = true;
	}
}

bool Leg::lagging()
{
	float posX = _relPos.x - _initPos.x;
	float posY = _relPos.y - _initPos.y;
	float pos = posX * cos(_stepDirection) + posY * sin(_stepDirection);

	return pos < -5;
}

bool Leg::centered()
{
	float posX = _relPos.x - _initPos.x;
	float posY = _relPos.y - _initPos.y;
	float pos = posX * cos(_stepDirection) + posY * sin(_stepDirection);

	return pos > -5 || pos < 5;
}

void Leg::commitToServos()
{
	if (_readyToCommit)
	{
		Dynamixel.moveRW(_coxaServoId, (int)((_newCoxaAngle + SERVO_ANGLE_OFFSET) * SERVO_ANGLE_FACTOR));
		Dynamixel.moveRW(_femurServoId, (int)((_newFemurAngle + SERVO_ANGLE_OFFSET) * SERVO_ANGLE_FACTOR));
		Dynamixel.moveRW(_tibiaServoId, (int)((_newTibiaAngle + SERVO_ANGLE_OFFSET) * SERVO_ANGLE_FACTOR));
		
		_coxaAngle = _newCoxaAngle;
		_femurAngle = _newFemurAngle;
		_tibiaAngle = _newTibiaAngle;
		_readyToCommit = false;
	}
}