#ifndef ROBOT_H
#define ROBOT_H
#include <Arduino.h>
#include <Servo.h>
#include "Pid.h"
#include "Consigne.h"
#include "Point.h"
#include "QueueList.h"
#include <Wire.h>
#include "Adafruit_TCS34725.h"	// color sensors
#include "Parameters.h"
#include "Instructions.h"

/*
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * Neobot wrote this file. As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy us a beer in return.
 * ----------------------------------------------------------------------------
 */
 
/*
 * Project : Neobot
 * Version : 0.42
 * Date : 30/12/2012
 * Author : Neobot
 */

class Logger;
class Comm;

class Robot
{
  public:
	Robot(Adafruit_TCS34725 *colorSensor1, Adafruit_TCS34725 *colorSensor2, float periodAsserv = 5.0, float x = 0.0, float y = 0.0, float theta = 0.0);
  
    enum TypeDeplacement
    {
        TournePuisAvance = 0,
        TourneEtAvance = 1,
        TourneSeulement = 2,
        AvanceSeulement = 3,
    };
    
    enum TypeAsserv
    {
        Aucun = 0,
        EnAvant = 1,
        EnArriere = 2,
        Auto = 13,
    };

	enum ColorSensorId
	{
		ColorSensor1,
		ColorSensor2,
		ColorSensorCount
	};
  
	void setLogger(Logger* logger);
	void setComm(Comm* comm);

    void teleport(Point point);
    void forceObjectif(Point point);
    void ajoutPoint(Point point);
	void ajoutPoint(float x, float y, bool pointArret = false, int typeDeplacement =0, float vitessMax = 100);
    void flush();
    void stop();
    void majPosition(float pasRoueGauche, float pasRoueDroite);
	bool checkBlocked(int speedL, int speedMotL, int speedR, int speedMotR);
    void calculConsigne();
    void calculCommande();
    float filtreCommandeRoue(float value);
    void avanceDe(float avance, bool avecFreinage = true, float vitessMax = VITESSE_MAX); // en mm
    void tourneDe(float rotation, bool avecFreinage = true, float vitessMax = VITESSE_MAX_ROT); // en rad
    void vaEnXY(float x, float y, bool estPointArret = true, float vitessMax = VITESSE_MAX);
    bool passageAuPointSuivant();
    void vaVersPointSuivant();
    bool estArrive();
	bool etaitArrive();
	bool quelSens();
	void attend(unsigned long attente); // tps en milliseconde
    bool estEnAttente();
    void stopAttente();
	void enableColorSensor(int sensorId);
	void disableColorSensor(int sensorId);
	bool isColorSensorEnabled(int sensorId);
	bool colorSensorValueHasChanged(int sensorId, ColorSensorState *color);
	void startPump(int pumpId);
	void stopPump(int pumpId);

	void MAJSonar(int avg, int avd, int arg, int ard);
	void detectObstacleFrein();

    float pasPrecendentGauche;
    float pasPrecendentDroit;
    
    PID _pidDist;
    PID _pidOrientation;
    
    Consigne _consigneDist;
    Consigne _consigneOrientation;

    QueueList<Point> queue;
    Point pointSuivant;
    Point position;
	Point sauvPointSuivant;
    
    float _periodAsserv;
    
    TypeDeplacement _typeDeplacement;
    TypeAsserv _typeAsserv;
  
	float _deltaDistMm;
	float _deltaOrientRad;
      
    float _thetaTotal; // angle non born√©

	int _commandeRoueGauche;
	int _commandeRoueDroite;
    
    unsigned long tempsAttenteDeplacement;
	unsigned long debutAttenteDeplacement;

	bool _pingReceived;

	bool _sens;

	int _sonar_AVD;
	int _sonar_AVG;
	int _sonar_ARD;
	int _sonar_ARG;

	bool _obstAvD;
	bool _obstAvG;
	bool _obstArD;
	bool _obstArG;

	bool _stopObst;

private:
    bool _tourneFini;

	bool _colorSensorEnabled[2];
	ColorSensorState _colorSensorStatus[2];
	Adafruit_TCS34725* _colorSensor[2];
	Logger* _logger;
	Comm* _comm;

};

#endif // ROBOT_H


