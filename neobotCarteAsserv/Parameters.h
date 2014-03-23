//#define COUNTDOWN
//#define DEBUG_RECEIVED_COMM_INSTRUCTION
#define NO_JACK
#define SIMULATION						// simulates motors & robot movements
#define NO_TPS_MATCH

#define ENABLE_DEBUG		true		// if false, disable all logging
#define ENABLE_PC_COMM		true		// enable comm to PC and redirect debug messages to PC
//#define DEBUG_ENCODER
//#define DEBUG_POSITION
//#define DEBUG_CONSIGNE_LIN
//#define DEBUG_CONSIGNE_ROT
//#define DEBUG_ULTRASON

#define MAX_PWM 4095.0
#define MAX_PWM_MOTORS 65535.0

#define PERIODE_ASSERV_MS 5.0
#define PERIODE_COM_LECTURE 50.0
#define PERIODE_COM_ECRITURE 50.0

#define TPS_MATCH 90000


// robot
#define PI 3.1415926535897
#define NB_PAS_TOUR 4096.0
#define ENTRAXE_MM 340.6 // mm
#define DIAMETRE_ROUE_MM 57.6 // en mm

#define ACCELARATION_MAX_EN_REEL_ROT 0.004
#define ACCELARATION_MAX_EN_REEL_LIN 0.004 // en mm/ms² ou pas : 0.003 * PERIODE_ASSERV_MS * PERIODE_ASSERV_MS * COEFF_CONVERTION_PAS_METRE // 1m/s² => 0.001 mm/ms² => 0.001*Te² mm mais comme on travaille en pas on multiplis pas le coeef de correction
#define VITESSE_MAX .5 * 1.4 // mm/ms
#define VITESSE_MAX_ROT .75 * 1.4

#define MAX_PWM_MOTORS 65535.0
#define OFFSET 0.0
#define RATIO_PWM 1.0 // pwm = OFFSET + consigne * RATIO_PWM = OFFSET + consigne * (MAX_PWM_MOTORS - OFFSET) / MAX_PWM_MOTORS

// PID
#define ACTIVE_PID_DISTANCE true
#define KP_DISTANCE 0.01
#define KD_DISTANCE 0.0

#define ACTIVE_PID_ANGLE true
#define KP_ANGLE 0.50
#define KD_ANGLE 0.0

#define COEFF_CORRECTION_TAILLE_ROUE_FOLLE 1

#define COEFF_CONVERTION_PAS_RADIAN NB_PAS_TOUR / (2.0 * PI) // 651.898646 pas / rad, valeur en 2011 : 1912
#define COEFF_CONVERTION_PAS_MM DIAMETRE_ROUE_MM / (2.0 * COEFF_CONVERTION_PAS_RADIAN) // 0.044178 mm / pas , valeur en 2011 : 5.62

#define COEF_CORRECTION_ROUE_FOLLES 0.0 // plus la valeur est grande plus il part à gauche, valeur en 2011 : 0.0019

#define COEF_CORRECTION_ROUE_FOLLE_DROITE 1.0 + COEF_CORRECTION_ROUE_FOLLES / 2.0  // coef de corrections des valeurs envoyées par les roues folles
#define COEF_CORRECTION_ROUE_FOLLE_GAUCHE 1.0 - COEF_CORRECTION_ROUE_FOLLES / 2.0  // coef de corrections des valeurs envoyées par les roues folles

// Odometrie
#define CORFUGE 0.0
