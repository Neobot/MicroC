#include "Simulation.h"
#include "Arduino.h"

Simulation::Simulation(int intervalMs, float mmPerStep, int maxPwm, float maxSpeed, float maxAcceleration)
{
	_interval = intervalMs;
	_mmPerStep = mmPerStep;
	_maxPwm = (float)maxPwm;
	_neutralPwm = (float)maxPwm / 2;
	_maxSpeed = maxSpeed;
	_maxAccelPerInterval = maxAcceleration * (float)intervalMs;

	_currentSpeed = 0;
	_currentSteps = 0;
}

void Simulation::setCommande(int pwm)
{
	_pwm = constrain((float)pwm, 0, _maxPwm);
}

int Simulation::getSteps()
{
	float targetSpeed;

	targetSpeed = (_pwm - _neutralPwm) / _neutralPwm * _maxSpeed;

	// limit the acceleration
	if (targetSpeed > _currentSpeed + _maxAccelPerInterval)
		_currentSpeed += _maxAccelPerInterval;
	else if (targetSpeed < _currentSpeed - _maxAccelPerInterval)
		_currentSpeed -= _maxAccelPerInterval;
	else
		_currentSpeed = targetSpeed;

	float deltaSteps = _currentSpeed * _interval / _mmPerStep;	// convert speed (m/s) to steps per interval
	_currentSteps += (int)deltaSteps;

	return _currentSteps;
}
