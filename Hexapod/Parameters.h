#ifndef PARAMETERS_H
#define PARAMETERS_H

#define ENABLE_SERVOS
#define ENABLE_DEBUG

#define SERVO_ANGLE_FACTOR	195.56959407
#define SERVO_ANGLE_OFFSET	2.617993878

static float	coxaLength = 34.5;		// distance between coxa and femur axis in (x, y) plan (in mm)
static float	coxaZOffset = 29.5;		// distance between body and femur axis on z axis (in mm)
static float	femurLength = 100;		// distance between femur and tibia axis (in mm)
static float	tibiaLength = 152;		// distance between tibia axis and foot (in mm)
static float	initCoxaFootDist = 135;	// distance from coxa to foot in default position (in mm)
static float	initBodyHeight = 90;	// height of body from ground (in mm)
static float	speed = 100;			// movement speed in mm/s
static float	stepHeight = 50;		// how much to lift legs at each step (in mm)
static float	maxStepSize = 50;		// stride size (in mm)
static float	maxStepRotation = 0.3;	// stride size (in rad)
static int		debugInterval = 500;	// interval for debug messages (in ms)

#endif // PARAMETERS_H
