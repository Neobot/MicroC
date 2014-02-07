#ifndef ROBOT_H
#define ROBOT_H
#include <Arduino.h>
#include <Servo.h>
#include "Pid.h"
#include "Consigne.h"
#include "Point.h"
#include "QueueList.h"

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
 
#define PI 3.1415926535897
#define NB_PAS_TOUR 4096.0
#define ENTRAXE 340.6 // mm
#define DIAMETRE_ROUE 57.6 // en mm

#define ACCELARATION_MAX_EN_REEL_ROT 0.004
#define ACCELARATION_MAX_EN_REEL_LIN 0.004 // en mm/ms² ou pas : 0.003 * PERIODE_ASSERV_MS * PERIODE_ASSERV_MS * COEFF_CONVERTION_PAS_METRE // 1m/s² => 0.001 mm/ms² => 0.001*Te² mm mais comme on travaille en pas on multiplis pas le coeef de correction
#define VITESSE_MAX .5 * 1.4 // mm/ms
#define VITESSE_MAX_ROT .75 * 1.4

#define VALEUR_MAX_PWM 4095.0
#define OFFSET 0.0
#define RATIO_PWM 1.0 // pwm = OFFSET + consigne * RATIO_PWM = OFFSET + consigne * (VALEUR_MAX_PWM - OFFSET) / VALEUR_MAX_PWM

// PID
#define ACTIVE_PID_DISTANCE true
#define KP_DISTANCE 0.01
#define KD_DISTANCE 0.0

#define ACTIVE_PID_ANGLE true
#define KP_ANGLE 0.50
#define KD_ANGLE 0.0

#define COEFF_CORRECTION_TAILLE_ROUE_FOLLE 1

#define COEFF_CONVERTION_PAS_RADIAN NB_PAS_TOUR / (2.0 * PI) // 651.898646 pas / rad, valeur en 2011 : 1912
#define COEFF_CONVERTION_PAS_METRE DIAMETRE_ROUE / (2.0 * COEFF_CONVERTION_PAS_RADIAN) // 22.6353 mm / pas , valeur en 2011 : 5.62 \\//  1 pas = PI*D/4096 ~ 0.000044178 m

#define COEF_CORRECTION_ROUE_FOLLES 0.0 //0.0019 //plus la valeur est grande plus il part à gauche, valeur en 2011 : 0.0019

#define COEF_CORRECTION_ROUE_FOLLE_DROITE 1.0 + COEF_CORRECTION_ROUE_FOLLES / 2.0  //1.0003183F  //coef de corrections des valeurs envoyées par les roues folles
#define COEF_CORRECTION_ROUE_FOLLE_GAUCHE 1.0 - COEF_CORRECTION_ROUE_FOLLES / 2.0  //0.9996817F  //coef de corrections des valeurs envoyées par les roues folles

// Odometrie
#define CORFUGE 0.0


class Robot
{
  public:
    Robot(Servo* servoArG, Servo* servoArD, float periodAsserv = 5.0, float x = 0.0, float y = 0.0, float theta = 0.0);
  
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
  
    void teleport(Point point);
    void forceObjectif(Point point);
    void ajoutPoint(Point point);
    void ajoutPoint(float x, float y, bool pointArret = false, int typeDeplacement =1, float vitessMax = 100);
    void flush();
    void stop();
    void majPosition(float pasRoueGauche, float pasRoueDroite);
    void calculConsigne();
    void calculCommande();
    float filtreCommandeRoue(float value);
    void avanceDe(float avance, bool avecFreinage = true, float vitessMax = VITESSE_MAX); // en mm
    void tourneDe(float rotation, bool avecFreinage = true, float vitessMax = VITESSE_MAX_ROT); // en rad
    void vaEnXY(float x, float y, bool estPointArret = true, float vitessMax = VITESSE_MAX);
    bool passageAuPointSuivant();
    void vaVersPointSuivant();
    bool estArrive();
    bool quelSens();
    void attend(unsigned long attente); // tps ne milliseconde
    bool estEnAttente();
    void stopAttente();
    void MAJContaineur(bool isGauche, int pos);
    
    
    float pasPrecendentGauche;
    float pasPrecendentDroit;
    
    PID _pidDist;
    PID _pidOrientation;
    
    Consigne _consigneDist;
    Consigne _consigneOrientation;

    QueueList<Point> queue;
    Point pointSuivant;
    Point position;
    
    float _periodAsserv;
    
    TypeDeplacement _typeDeplacement;
    TypeAsserv _typeAsserv;
  
    float _deltaDist;
    float _deltaOrient;
      
    float _thetaTotal; // angle non borné

    int _commmandeRoueGauche;
    bool _sensAvantRoueGauche;
    int _commmandeRoueDroite;
    bool _sensAvantRoueDroite;
    
    unsigned long tempsAttenteDeplacement;
    unsigned long debutAttenteDeplacement;
    
    Servo* servoArG;
    Servo* servoArD;
    
    bool _tourneFini;
    bool _pingReceived;
		
};

#endif // ROBOT_H

