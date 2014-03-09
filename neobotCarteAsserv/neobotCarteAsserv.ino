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
 * Date : Today
 * Author : Neobot
 */


//#include <arduino.h>
#include <math.h>
#include <Servo.h>
#include "QueueList.h" // fifo
#include <Task.h>
#include "pwm01.h"
#include <Wire.h>
#include "Adafruit_TCS34725.h"	// color sensors

#include "Protocol.h"
#include "Robot.h"
#include "com.h"
#include "Logger.h"
#include "Simulation.h"


/*********************************************************************************/
/*                                  Config                                       */
/*********************************************************************************/

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

#define PI 3.1415926535897
#define NB_PAS_TOUR 4096.0
#define DIAMETRE_ROUE_MM 57.6
#define COEFF_CONVERTION_PAS_RADIAN NB_PAS_TOUR / (2.0 * PI)
#define COEFF_CONVERTION_PAS_MM DIAMETRE_ROUE_MM / (2.0 * COEFF_CONVERTION_PAS_RADIAN)


#define PIN_SERVO_1 4
#define PIN_SERVO_2 5
#define PIN_SERVO_3 6
#define PIN_SERVO_4 7

// moteur commande on/off
#define PIN_MOTEUR_1 11
#define PIN_MOTEUR_2 12
#define PIN_MOTEUR_3 30
#define PIN_MOTEUR_4 31

#define PIN_PWM_COLOR_R 2		//ok
#define PIN_PWM_COLOR_G 3		//ok
#define PIN_PWM_COLOR_B 10		//ok

#define UM6_1 16
#define UM6_2 17

#define PIN_JACK 22
#define PIN_BOUTON_1 23
#define PIN_BOUTON_2 51

#define PIN_INTERRUPTEUR_COULEUR 24			//ok

#define PIN_CONTACTEUR_1 25
#define PIN_CONTACTEUR_2 26
#define PIN_CONTACTEUR_3 27
#define PIN_CONTACTEUR_4 28
#define PIN_CONTACTEUR_5 29
#define PIN_CONTACTEUR_6 30

#define PIN_MOTEUR_GAUCHE_SENS 9			//ok
#define PIN_MOTEUR_GAUCHE_PWM_DIGITAL 29	//ok
#define PIN_MOTEUR_GAUCHE_BREAK 28			//ok
#define PIN_MOTEUR_DROITE_SENS 8			//ok
#define PIN_MOTEUR_DROITE_PWM_DIGITAL 27	//ok
#define PIN_MOTEUR_DROITE_BREAK 26			//ok

//FPGA
<<<<<<< HEAD
#define PIN_FPGA_BIT11 41 //ok
#define PIN_FPGA_BIT10 42 //ok
#define PIN_FPGA_BIT09 43 //ok
#define PIN_FPGA_BIT08 44 //ok
#define PIN_FPGA_BIT07 45 //ok
#define PIN_FPGA_BIT06 47 //ok
#define PIN_FPGA_BIT05 48 //ok
#define PIN_FPGA_BIT04 49 //ok
#define PIN_FPGA_BIT03 50 //ok
#define PIN_FPGA_BIT02 51 //ok
#define PIN_FPGA_BIT01 52 //ok
#define PIN_FPGA_BIT00 53 //ok

#define PIN_FPGA_SEL3 38 //ok
#define PIN_FPGA_SEL2 39 //ok
#define PIN_FPGA_SEL1 40 //ok
#define PIN_FPGA_SEL0 46 //ok


// ****** sensors ***** //

=======
#define PIN_FPGA_BIT11 38
#define PIN_FPGA_BIT10 39
#define PIN_FPGA_BIT09 40
#define PIN_FPGA_BIT08 41
#define PIN_FPGA_BIT07 42
#define PIN_FPGA_BIT06 44
#define PIN_FPGA_BIT05 45
#define PIN_FPGA_BIT04 46
#define PIN_FPGA_BIT03 47
#define PIN_FPGA_BIT02 48
#define PIN_FPGA_BIT01 49
#define PIN_FPGA_BIT00 50

#define PIN_FPGA_SEL3 35
#define PIN_FPGA_SEL2 36
#define PIN_FPGA_SEL1 37
#define PIN_FPGA_SEL0 43


// sensors
>>>>>>> FETCH_HEAD
#define PIN_SHARP_1 0
#define PIN_SHARP_2 1
#define PIN_SHARP_3 2
#define PIN_SHARP_4 3
#define PIN_SHARP_5 4
#define PIN_SHARP_6 5

#define PIN_SONAR_AV_G 6 //ok
#define PIN_SONAR_AV_D 7 //ok
#define PIN_SONAR_AR_G 8 //ok
#define PIN_SONAR_AR_D 9 //ok


/*********************************************************************************/
/*                              Global objects                                   */
/*********************************************************************************/

Task asservissement(PERIODE_ASSERV_MS);
Task commLect(PERIODE_COM_LECTURE);
Task commEcrit(PERIODE_COM_ECRITURE);

Servo servoArG;
Servo servoArD;

Adafruit_TCS34725 colorSensor1 = Adafruit_TCS34725(&Wire, TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);
Adafruit_TCS34725 colorSensor2 = Adafruit_TCS34725(&Wire1, TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);

Robot batRobot(&servoArG, &servoArD, PERIODE_ASSERV_MS);
Comm batCom(&batRobot);
Logger batLogger(&batCom, ENABLE_DEBUG, ENABLE_PC_COMM);

#ifdef SIMULATION
	Simulation simMotorL(PERIODE_ASSERV_MS, COEFF_CONVERTION_PAS_MM, MAX_PWM_MOTORS, 1.4, 0.01);
	Simulation simMotorR(PERIODE_ASSERV_MS, COEFF_CONVERTION_PAS_MM, MAX_PWM_MOTORS, 1.4, 0.01);
#endif


unsigned int initEncodeurG;
unsigned int initEncodeurD;

unsigned long tempsMatch;


/*********************************************************************************/
/*                                 Functions                                     */
/*********************************************************************************/

unsigned int readOneEncodeurWord()
{
    unsigned int total = digitalRead(PIN_FPGA_BIT11);
    total = (total << 1) + digitalRead(PIN_FPGA_BIT10);
    total = (total << 1) + digitalRead(PIN_FPGA_BIT09);
    total = (total << 1) + digitalRead(PIN_FPGA_BIT08);
    total = (total << 1) + digitalRead(PIN_FPGA_BIT07);
    total = (total << 1) + digitalRead(PIN_FPGA_BIT06);
    total = (total << 1) + digitalRead(PIN_FPGA_BIT05);
    total = (total << 1) + digitalRead(PIN_FPGA_BIT04);
    total = (total << 1) + digitalRead(PIN_FPGA_BIT03);
    total = (total << 1) + digitalRead(PIN_FPGA_BIT02);
    total = (total << 1) + digitalRead(PIN_FPGA_BIT01);
    total = (total << 1) + digitalRead(PIN_FPGA_BIT00);

    return total;
}

unsigned int readEncoder(bool wheel, bool encoder, bool spd)
{
    digitalWrite(PIN_FPGA_SEL3, spd);
    digitalWrite(PIN_FPGA_SEL2, encoder);
    digitalWrite(PIN_FPGA_SEL1, wheel);

    digitalWrite(PIN_FPGA_SEL0, HIGH);
    unsigned int total = readOneEncodeurWord();

    digitalWrite(PIN_FPGA_SEL0, LOW);
    total = (total << 12) + readOneEncodeurWord();

    return total;
}

void MAJPosition()
{
    int dg, dd;

#ifndef SIMULATION
    dg = readEncoder(0, 0, 0) - initEncodeurG;
    dd = readEncoder(1, 0, 0) - initEncodeurD;
#else
    dg = simMotorL.getSteps();
    dd = simMotorR.getSteps();
#endif

#ifdef DEBUG_ENCODER
    batLogger.print("dg=");
    batLogger.print(dg);
    batLogger.print(" dd=");
    batLogger.print(dd);
    batLogger.println(" ");
#endif

    batRobot.majPosition((float) dg, (float) dd);
}

void envoiConsigne()
{
#ifndef SIMULATION
    digitalWrite(PIN_MOTEUR_GAUCHE_PWM_DIGITAL, batRobot._commandeRoueGauche == 0 ? LOW : HIGH);	// stop motor if command is zero
    digitalWrite(PIN_MOTEUR_DROITE_PWM_DIGITAL, batRobot._commandeRoueDroite == 0 ? LOW : HIGH);

	pwm_write_duty(PIN_MOTEUR_GAUCHE_SENS, batRobot._commandeRoueGauche);
	pwm_write_duty(PIN_MOTEUR_DROITE_SENS, batRobot._commandeRoueDroite);
#else
    simMotorL.setCommande(batRobot._commandeRoueGauche);
    simMotorR.setCommande(batRobot._commandeRoueDroite);
#endif
}

void litEtEnvoieSonar()
{
    int ag = analogRead(PIN_SONAR_AV_G);
    int ad = analogRead(PIN_SONAR_AV_D);
    int rg = analogRead(PIN_SONAR_AR_G);
    int rd = analogRead(PIN_SONAR_AR_D);

    int valmax = 397;

	ag = constrain(ag, 0, valmax);
	ad = constrain(ad, 0, valmax);
	rg = constrain(rg, 0, valmax);
	rd = constrain(rd, 0, valmax);

    ag = map(ag, 0, valmax, 0, 255);
    ad = map(ad, 0, valmax, 0, 255);
    rg = map(rg, 0, valmax, 0, 255);
    rd = map(rd, 0, valmax, 0, 255);

#ifdef DEBUG_ULTRASON
    batLogger.print("ag=");
    batLogger.print(ag);
    batLogger.print(" ad=");
    batLogger.print(ad);
    batLogger.print(" rg=");
    batLogger.print(rg);
    batLogger.print(" rd=");
    batLogger.print(rd);
    batLogger.println(" ");
#endif

#ifndef NO_PC_COMM
    batCom.sendSonars(ag, ad, rg, rd);
#endif
}

void litEtEnvoieColorSensors()
{
	uint16_t r1, g1, b1, r2, g2, b2;
	colorSensor1.getRawDataWithoutDelay(&r1, &g1, &b1);
	colorSensor2.getRawDataWithoutDelay(&r2, &g2, &b2);

#ifndef NO_PC_COMM
	batCom.sendColorSensors((int)r1, (int)g1, (int)b1, (int)r2, (int)g2, (int)b2);
#endif
}

void enableColorSensors(bool enable)
{
	colorSensor1.setInterrupt(!enable);
	colorSensor2.setInterrupt(!enable);
}


void bougeServo()
{

}

void setLedRGB(int r, int g, int b)
{
	r = constrain(r, 0, 255);
	g = constrain(g, 0, 255);
	b = constrain(b, 0, 255);

	r = map(r, 0, 255, 0, MAX_PWM);
	g = map(g, 0, 255, 0, MAX_PWM);
	b = map(b, 0, 255, 0, MAX_PWM);

	analogWrite(PIN_PWM_COLOR_R, r);
	analogWrite(PIN_PWM_COLOR_G, g);
	analogWrite(PIN_PWM_COLOR_B, b);
}

int readColor()
{
    int color = digitalRead(PIN_INTERRUPTEUR_COULEUR);
    if (color == HIGH)
		setLedRGB(255, 255, 0);    // jaune
    else
        setLedRGB(255, 0, 0);  // rouge

    return color;
}


/*********************************************************************************/
/*                               Initialisation                                  */
/*********************************************************************************/

void setup()
{
    analogWriteResolution(12);
    analogReadResolution(12);

	// motors
	pinMode(PIN_MOTEUR_GAUCHE_PWM_DIGITAL, OUTPUT);
    pinMode(PIN_MOTEUR_GAUCHE_BREAK, OUTPUT);
    pinMode(PIN_MOTEUR_DROITE_PWM_DIGITAL, OUTPUT);
    pinMode(PIN_MOTEUR_DROITE_BREAK, OUTPUT);
    digitalWrite(PIN_MOTEUR_GAUCHE_PWM_DIGITAL, LOW);
    digitalWrite(PIN_MOTEUR_GAUCHE_BREAK, LOW);
    digitalWrite(PIN_MOTEUR_DROITE_PWM_DIGITAL, LOW);
    digitalWrite(PIN_MOTEUR_DROITE_BREAK, LOW);
	pwm_set_resolution(16);							// for motors only (custom PWM frequency)
	pwm_setup(PIN_MOTEUR_GAUCHE_SENS, 40000, 2);	// 40 kHz (max recommended with this method)
	pwm_setup(PIN_MOTEUR_DROITE_SENS, 40000, 2);
	pwm_write_duty(PIN_MOTEUR_GAUCHE_SENS, 32767);
	pwm_write_duty(PIN_MOTEUR_DROITE_SENS, 32767);

	// moteur on/off
	pinMode(PIN_MOTEUR_1, OUTPUT);
	pinMode(PIN_MOTEUR_2, OUTPUT);
	pinMode(PIN_MOTEUR_3, OUTPUT);
	pinMode(PIN_MOTEUR_4, OUTPUT);

	digitalWrite(PIN_MOTEUR_1, LOW);
	digitalWrite(PIN_MOTEUR_2, HIGH);
	digitalWrite(PIN_MOTEUR_3, LOW);
	digitalWrite(PIN_MOTEUR_4, LOW);


    // FPGA
    pinMode(PIN_FPGA_SEL3, OUTPUT);
    pinMode(PIN_FPGA_SEL2, OUTPUT);
    pinMode(PIN_FPGA_SEL1, OUTPUT);
    pinMode(PIN_FPGA_SEL0, OUTPUT);
    pinMode(PIN_FPGA_BIT11, INPUT);
    pinMode(PIN_FPGA_BIT10, INPUT);
    pinMode(PIN_FPGA_BIT09, INPUT);
    pinMode(PIN_FPGA_BIT08, INPUT);
    pinMode(PIN_FPGA_BIT07, INPUT);
    pinMode(PIN_FPGA_BIT06, INPUT);
    pinMode(PIN_FPGA_BIT05, INPUT);
    pinMode(PIN_FPGA_BIT04, INPUT);
    pinMode(PIN_FPGA_BIT03, INPUT);
    pinMode(PIN_FPGA_BIT02, INPUT);
    pinMode(PIN_FPGA_BIT01, INPUT);
    pinMode(PIN_FPGA_BIT00, INPUT);

	// sensors
    //pinMode(PIN_MICROSWITCH_AD INPUT);
    //pinMode(PIN_MICROSWITCH_AG INPUT);
    //pinMode(PIN_SONAR_AV_G, INPUT);
    //pinMode(PIN_SONAR_AV_D, INPUT);
    //pinMode(PIN_SONAR_AR_G, INPUT);
    //pinMode(PIN_SONAR_AR_D, INPUT);

	// misc
    pinMode(PIN_JACK, INPUT);
    pinMode(PIN_INTERRUPTEUR_COULEUR, INPUT);

#ifdef DEBUG_RECEIVED_COMM_INSTRUCTION
    batCom.setLogger(&batLogger);
#endif

    //register parameters which can be changed trhough the comm
    //Max number of parameters is currently 10, defined in com.h
    batCom.registerParameter(&batRobot._pidDist._kp, "PID Distance P");
    batCom.registerParameter(&batRobot._pidDist._kd, "PID Distance D");
    batCom.registerParameter(&batRobot._pidOrientation._kp, "PID Orientation P");
    batCom.registerParameter(&batRobot._pidOrientation._kd, "PID Orientation D");

    //servoArG.attach(PIN_SERVO_G, 900, 2500);
    //servoArD.attach(PIN_SERVO_D, 900, 2500);

    //batRobot.MAJContaineur(true, 3);
    //batRobot.MAJContaineur(false, 3);

    //servoArG.detach();
    //servoArD.detach();

	//batRobot.ajoutPoint(200, -50, false);
    //batRobot.ajoutPoint(300, 0, true);
    //batRobot.ajoutPoint(400, 0, false);
    //batRobot.ajoutPoint(600, -50, false);
    //batRobot.ajoutPoint(800, -0, false);
    //batRobot.ajoutPoint(1000, -50, true);

	Serial.begin(115200);

	//int restartBtn = digitalRead(PIN_RESTART);
    //int oldRestartBtnValue = digitalRead(PIN_RESTART);

    /* while(!batRobot._pingReceived)
  {
    batCom.comm_read();
    restartBtn = digitalRead(PIN_RESTART);
    if (oldRestartBtnValue != restartBtn)
    {
      batCom.restart();
      oldRestartBtnValue = restartBtn;
      batLogger.println("Restart PC");
    }

  }*/

	delay(1000);
	batLogger.println("Restart arduino");

#ifdef SIMULATION
	batLogger.println("Mode : Simulation");
#endif

#ifndef NO_TPS_MATCH
	batLogger.print("Match duration : ");
	batLogger.print(TPS_MATCH / 1000);
	batLogger.println(" seconds");
#else
	batLogger.println("Match timer disabled");
#endif

#ifndef NO_JACK
    setLedRGB(0, 255, 0);	// vert

	bool jackPlugged = digitalRead(PIN_JACK) == LOW;

	if (!jackPlugged)
	{
		batLogger.println("Please plug the jack");

		while(!jackPlugged)
			jackPlugged = digitalRead(PIN_JACK) == LOW;
	}

	delay(2000);

	batLogger.println("Please unplug the jack");
    
    while(jackPlugged)
        jackPlugged = digitalRead(PIN_JACK) == LOW;
#endif

	bool estJaune = readColor();

	batLogger.print("Selected color: ");
	batLogger.println(estJaune ? "Yellow" : "Red");

#ifdef COUNTDOWN
    for(int i = 5; i > 0; --i)
    {
        batLogger.println(i);
        delay(1000);
    }
#endif

    tempsMatch = millis();
    initEncodeurD = readEncoder(1, 0, 0);
    initEncodeurG = readEncoder(0, 0, 0);
    batRobot.passageAuPointSuivant();
    batRobot.vaVersPointSuivant();

	enableColorSensors(true);

	batLogger.println("Here we gooooo!");

#ifndef NO_PC_COMM
    batCom.sendGo(estJaune);
#endif
}


/*********************************************************************************/
/*                                 Main loop                                     */
/*********************************************************************************/


void loop()
{
#ifndef NO_PC_COMM
    if (commLect.ready())
    {
        batCom.comm_read();
    }

    if (commEcrit.ready())
    {
        batCom.sendPosition();
<<<<<<< HEAD
		litEtEnvoieSonar();
		litEtEnvoieColorSensors();
=======
	   // litEtEnvoieSonar();
>>>>>>> FETCH_HEAD
    }
#endif

    if (asservissement.ready())
    {
        MAJPosition();
        batRobot.vaVersPointSuivant();
        batRobot.calculConsigne();
        batRobot.calculCommande();
        envoiConsigne();

#ifndef NO_PC_COMM
        if(batRobot.passageAuPointSuivant())
        {
            batCom.sendConsigne();
            batCom.sendIsArrived();
        }
#endif

#ifdef DEBUG_CONSIGNE_LIN
        //if (!batRobot._consigneDist.estArrive())
        {
            batLogger.print("D: Consigne:");
            batLogger.print(batRobot._consigneDist._consigne);
            batLogger.print(" dr:");
            batLogger.print(batRobot._consigneDist._distDemande - batRobot._consigneDist._distRealise);
            batLogger.print(" Dcc:");
            batLogger.print(batRobot._consigneDist._distDcc);
            batLogger.print(" Vc:");
            batLogger.print(batRobot._consigneDist._vitessCourrante);
            if (batRobot._consigneDist.estArrive())
                batLogger.print(" ARRIVE");
            batLogger.println(" ");
        }
#endif

#ifdef DEBUG_CONSIGNE_ROT
        //if (!batRobot._consigneOrientation.estArrive())
        {
            batLogger.print("R: Consigne:");
            batLogger.print(batRobot._consigneOrientation._consigne);
            batLogger.print(" dr:");
            batLogger.print(batRobot._consigneOrientation._distDemande - batRobot._consigneOrientation._distRealise);
            batLogger.print(" Dcc:");
            batLogger.print(batRobot._consigneOrientation._distDcc);
            batLogger.print(" Vc:");
            batLogger.print(batRobot._consigneOrientation._vitessCourrante);
            if (batRobot._consigneOrientation.estArrive())
                batLogger.print(" ARRIVE");
            batLogger.println(" ");
        }
#endif

#ifdef DEBUG_POSITION
        //if (batRobot._consigneDist.calcEstArrive() == false)
        {
            batLogger.print("xp=");
            batLogger.print(batRobot.pointSuivant.x);
            batLogger.print(" yp=");
            batLogger.print(batRobot.pointSuivant.y);
            batLogger.print(" x=");
            batLogger.print(batRobot.position.x);
            batLogger.print(" y=");
            batLogger.print(batRobot.position.y);
            batLogger.print(" t=");
            batLogger.print(batRobot.position.theta);
            batLogger.println(" ");
        }
#endif

#ifndef NO_TPS_MATCH
        if(millis() - tempsMatch >= TPS_MATCH)
        {
            digitalWrite(PIN_MOTEUR_GAUCHE_PWM_DIGITAL, LOW);
            digitalWrite(PIN_MOTEUR_DROITE_PWM_DIGITAL, LOW);

			batLogger.println("Timer elapsed - end of match");

    #ifdef DEBUG_POSITION
            //if (batRobot._consigneDist.calcEstArrive() == false)
            // servoArG.detach();
            //servoArD.detach();

            while(1)
            {
                batLogger.print("xp=");
                batLogger.print(batRobot.pointSuivant.x);
                batLogger.print(" yp=");
                batLogger.print(batRobot.pointSuivant.y);
                batLogger.print(" x=");
                batLogger.print(batRobot.position.x);
                batLogger.print(" y=");
                batLogger.print(batRobot.position.y);
                batLogger.print(" t=");
                batLogger.print(batRobot.position.theta);
                batLogger.println(" ");
            }
    #endif

    #ifdef DEBUG_ENCODER
            batLogger.print("g=");
            batLogger.print((int)readEncoder(0, 0, 0));
            batLogger.print(" r=");
            batLogger.print((int)readEncoder(1, 0, 0));
            batLogger.println(" ");
    #endif
            while(1)
            {
                setLedRGB(random(0, 255), random(0, 255), random(0, 255));
                delay(50);
            }
        } // millis() - tempsMatch >= TPS_MATCH
#endif
    }	// asservissement.ready()
}
