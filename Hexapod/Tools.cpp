#include "Tools.h"
#include "Arduino.h"

float Tools::trimAngle(float angle)
{
	while (angle > PI)
		angle -= TWO_PI;
		
	while (angle < -PI)
		angle += TWO_PI;
		
	return angle;
}