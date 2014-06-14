//#define COUNTDOWN
//#define NO_JACK
//#define SIMULATION						// simulates motors & robot movements
//#define NO_TPS_MATCH

#define ENABLE_DEBUG		true		// if false, disable all logging
#define ENABLE_PC_COMM		false		// enable comm to PC and redirect debug messages to PC

//#define DEBUG_RECEIVED_COMM_INSTRUCTION
//#define DEBUG_ENCODER
//#define DEBUG_SPEED
//#define DEBUG_POSITION
//#define DEBUG_PID
//#define DEBUG_CONSIGNE_MOTEUR
//#define DEBUG_CONSIGNE_LIN
//#define DEBUG_CONSIGNE_ROT
#define DEBUG_ULTRASON
//#define DEBUG_COLOR_SENSORS

#define GRAPH_VCC
#define GRAPH_ULTRASON

#define VccGraph 0
#define UltrasonGraph 1

#define MAX_PWM 4095.0

#define PERIODE_ASSERV_MS 5.0
#define PERIODE_COM_LECTURE 50.0
#define PERIODE_COM_ECRITURE 50.0
#define PERIODE_READ_COLOR_SENSOR 50.0

#define TPS_MATCH 90000

#define PI 3.1415926535897

// Robot
#define NB_PAS_TOUR 4096.0
#define ENTRAXE_MM 340.6 // mm
#define DIAMETRE_ROUE_MM 57.6 // en mm

#define ACCELARATION_MAX_EN_REEL_ROT 0.01
#define ACCELARATION_MAX_EN_REEL_LIN 0.006 // en mm/ms¬≤ ou pas : 0.003 * PERIODE_ASSERV_MS * PERIODE_ASSERV_MS * COEFF_CONVERTION_PAS_METRE // 1m/s¬≤ => 0.001 mm/ms¬≤ => 0.001*Te¬≤ mm mais comme on travaille en pas on multiplis pas le coeef de correction

#define VITESSE_MAX_REEL 1.4
#define VITESSE_MAX .9 * VITESSE_MAX_REEL // mm/ms
#define VITESSE_MAX_ROT .9 * VITESSE_MAX_REEL

#define MAX_PWM_MOTORS 65535.0
#define MINI_MOTOR 10
#define OFFSET 0.0
#define RATIO_PWM 1.0 // pwm = OFFSET + consigne * RATIO_PWM = OFFSET + consigne * (MAX_PWM_MOTORS - OFFSET) / MAX_PWM_MOTORS

// Consigne

#define COEFF_FREINAGE_DIST 3
#define COEFF_FREINAGE_ANG 1
#define COEFF_AUGMENTATION_FREINAGE 4

#define DIST_ARRIVE_DIST 10.0 // mm
#define DIST_ARRIVE_ANG 1 * PI * ENTRAXE_MM / 360.0 // en degr√©e 0.1¬∞ ~ 0.35 mm

// PID
#define ACTIVE_PID_DISTANCE true
#define KP_DISTANCE 2.0
#define KD_DISTANCE 0.0
#define KI_DISTANCE 0.0 // ca fout la merde ;)

#define ACTIVE_PID_ANGLE true
#define KP_ANGLE 1.0
#define KD_ANGLE 0.0
#define KI_ANGLE 0.0 // ca fout la merde ;)

// Odometrie
#define COEFF_CORRECTION_TAILLE_ROUE_FOLLE 1

#define COEFF_CONVERTION_PAS_RADIAN NB_PAS_TOUR / (2.0 * PI) // 651.898646 pas / rad, valeur en 2011 : 1912
#define COEFF_CONVERTION_PAS_MM DIAMETRE_ROUE_MM / (2.0 * COEFF_CONVERTION_PAS_RADIAN) // 0.044178 mm / pas , valeur en 2011 : 5.62

#define COEF_CORRECTION_ROUE_FOLLES 0.0 // plus la valeur est grande plus il part √† gauche, valeur en 2011 : 0.0019

#define COEF_CORRECTION_ROUE_FOLLE_DROITE 1.0 + COEF_CORRECTION_ROUE_FOLLES / 2.0  // coef de corrections des valeurs envoy√©es par les roues folles
#define COEF_CORRECTION_ROUE_FOLLE_GAUCHE 1.0 - COEF_CORRECTION_ROUE_FOLLES / 2.0  // coef de corrections des valeurs envoy√©es par les roues folles

#define CORFUGE 0.0


