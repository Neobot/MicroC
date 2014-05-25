#ifndef INSTRUCTIONS_H
#define INSTRUCTIONS_H

// PC -> MicroC
#define INSTR_DEST_ADD				1
#define INSTR_DEST_REPLACE			2
#define INSTR_FLUSH					3
#define INSTR_SET_POS				10
#define INSTR_ENABLE_SENSOR			20
#define INSTR_DISABLE_SENSOR		21
#define INSTR_SET_PARAMETERS		50
#define INSTR_ASK_PARAMETERS		51
#define INSTR_ACTION				60

// MicroC -> PC
#define INSTR_COORD					100
#define INSTR_CONSIGNE				104
#define INSTR_SEND_SONARS			110
#define INSTR_INIT_DONE				120
#define INSTR_GO					121
#define INSTR_RESTART				122
#define INSTR_QUIT					123
#define INSTR_LOG					124
#define INSTR_PARAMETERS			125
#define INSTR_PARAMETERS_NAMES		126
#define INSTR_EVENT					130
#define INSTR_SENSOR_EVENT			131

// bidirectional
#define INSTR_PING					254
#define INSTR_AR					255

enum RobotAction
{
	NO_ACTION = -1,
	ACTION_START_PUMP = 1,
	ACTION_STOP_PUMP = 2
};

enum RobotEvent
{
	NO_EVENT = -1,
	EVENT_IS_ARRIVED = 1,
	EVENT_IS_BLOCKED = 2,
	EVENT_QUIT = 3,
	EVENT_RESTART = 4
};

enum SensorType
{
	SharpSensor = 0,
	MicroswitchSensor = 1,
	ColorSensor = 2
};

enum SharpState
{
	SharpNothingDetected,
	SharpObjectDetected,
	SharpObjectVeryClose,
};

enum MicroswicthState
{
	MicroswicthOff,
	MicroswicthOn
};

enum ColorSensorState
{
	ColorUnknown,
	ColorRed,
	ColorGreen,
	ColorBlue,
	ColorYellow,
	ColorWhite,
	ColorBlack
};

#endif

