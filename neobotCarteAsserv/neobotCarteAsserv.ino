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


#include <math.h>
#include <Wire.h>				// i2c
#include <Servo.h>
#include "QueueList.h"			// fifo
#include "Task.h"
#include "pwm01.h"				// pwm with custom frequency (motors)
#include "Adafruit_TCS34725.h"	// color sensors

#include "IOConfig.h"
#include "Parameters.h"
#include "Protocol.h"
#include "Robot.h"
#include "Comm.h"
#include "Logger.h"
#include "Simulation.h"
#include "Instructions.h"


/*********************************************************************************/
/*                              Global objects                                   */
/*********************************************************************************/

Task asservissement(PERIODE_ASSERV_MS);
Task odometry(PERIODE_ODOMETRY_MS);
Task commLect(PERIODE_COM_LECTURE);
Task commEcrit(PERIODE_COM_ECRITURE);
Task sonar(PERIODE_COM_ECRITURE);
Task oneSecond(1000);
Task readColorSensors(PERIODE_READ_COLOR_SENSOR);
Task debugEnvoie(5);

Adafruit_TCS34725 colorSensor1(&Wire, TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);
Adafruit_TCS34725 colorSensor2(&Wire1, TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);

Robot batRobot(&colorSensor1, &colorSensor2, PERIODE_ASSERV_MS);
Comm batCom(&batRobot, ENABLE_PC_COMM);
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

	if (spd == 1)
		total -= 8388608;		// offset speed as value sent from FPGA always positive

    return total;
}

void MAJPosition()
{
    int dg, dd;
	int speedL, speedR, speedMotL, speedMotR;

#ifndef SIMULATION
    dg = readEncoder(0, 0, 0) - initEncodeurG;
    dd = readEncoder(1, 0, 0) - initEncodeurD;

	// read encoders speed (in mm/s)
	speedL = readEncoder(0, 0, 1);
	speedR = readEncoder(1, 0, 1);
	speedMotL = readEncoder(0, 1, 1);
	speedMotR = readEncoder(1, 1, 1);
#else
    dg = simMotorL.getSteps();
    dd = simMotorR.getSteps();
	speedL = dg;
	speedR = dd;
	speedMotL = dg;
	speedMotR = dd;
#endif

#ifdef DEBUG_ENCODER
    batLogger.print("dg=");
    batLogger.print(dg);
	batLogger.print(" dd=");
    batLogger.print(dd);
    batLogger.println(" ");
#endif

#ifdef DEBUG_SPEED
	batLogger.print("speedL=");
	batLogger.print(speedL);
	batLogger.print(" speedMotL=");
	batLogger.print(speedMotL);
	batLogger.print(" speedR=");
	batLogger.print(speedR);
	batLogger.print(" speedMotR=");
	batLogger.println(speedMotR);
#endif

    batRobot.majPosition((float) dg, (float) dd);

	/*if (batRobot.checkBlocked(speedL, speedMotL, speedR, speedMotR))
		batCom.sendEvent(EVENT_IS_BLOCKED);*/
}

void envoiConsigne()
{
#ifndef SIMULATION

    bool stopMotG = batRobot._commandeRoueGauche >= ((MAX_PWM_MOTORS - 1.0) / 2.0 - MINI_MOTOR) && batRobot._commandeRoueGauche <= ((MAX_PWM_MOTORS - 1.0) / 2.0 + MINI_MOTOR);
    bool stopMotD = batRobot._commandeRoueDroite >= ((MAX_PWM_MOTORS - 1.0) / 2.0 - MINI_MOTOR) && batRobot._commandeRoueDroite <= ((MAX_PWM_MOTORS - 1.0) / 2.0 + MINI_MOTOR);

    digitalWrite(PIN_MOTEUR_GAUCHE_PWM_DIGITAL, stopMotG ? LOW : HIGH);	// stop motor if command is zero
    digitalWrite(PIN_MOTEUR_DROITE_PWM_DIGITAL, stopMotD ? LOW : HIGH);

	pwm_write_duty(PIN_MOTEUR_GAUCHE_SENS, batRobot._commandeRoueGauche);
	pwm_write_duty(PIN_MOTEUR_DROITE_SENS, batRobot._commandeRoueDroite);
#else
    simMotorL.setCommande(batRobot._commandeRoueGauche);
    simMotorR.setCommande(batRobot._commandeRoueDroite);
#endif

#ifdef DEBUG_CONSIGNE_MOTEUR
	batLogger.print("Commande moteur: g=");
	batLogger.print(batRobot._commandeRoueGauche);
	batLogger.print(" d=");
	batLogger.println(batRobot._commandeRoueDroite);
#endif
}

void litEtEnvoieSonar()
{
	int ag = analogRead(PIN_SONAR_AV_G);	// max 4096 = 1024 cm (12 bits)
    int ad = analogRead(PIN_SONAR_AV_D);
    int rg = analogRead(PIN_SONAR_AR_G);
    int rd = analogRead(PIN_SONAR_AR_D);

	int valmax = 1020;		// truncate to 1020 = 255 cm

	ag = constrain(ag, 0, valmax);
	ad = constrain(ad, 0, valmax);
	rg = constrain(rg, 0, valmax);
	rd = constrain(rd, 0, valmax);

    ag = map(ag, 0, valmax, 0, 255);
    ad = map(ad, 0, valmax, 0, 255);
    rg = map(rg, 0, valmax, 0, 255);
    rd = map(rd, 0, valmax, 0, 255);

	//batCom.sendSonars(ag, ad, rg, rd);
	batRobot.MAJSonar(ag, ad, rg, rd);
	batRobot.detectObstacleFrein();
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
		setLedRGB(255, 128, 0);    // jaune
    else
        setLedRGB(255, 0, 0);  // rouge

    return color;
}

void setVitesseLineare(float value)
{
	//TODO
	batRobot._consigneDist._vitessMax = value;
}

void setVitesseRot(float value)
{
	//TODO
	batRobot._consigneOrientation._vitessMax = value;
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
	digitalWrite(PIN_MOTEUR_2, LOW);
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

	batRobot.setLogger(&batLogger);
	batRobot.setComm(&batCom);

    //register parameters which can be changed trhough the comm
	//Max number of parameters is currently 25, defined in comm.h
    batCom.registerParameter(&batRobot._pidDist._kp, "PID Distance P");
    batCom.registerParameter(&batRobot._pidDist._kd, "PID Distance D");
    batCom.registerParameter(&batRobot._pidOrientation._kp, "PID Orientation P");
    batCom.registerParameter(&batRobot._pidOrientation._kd, "PID Orientation D");
	batCom.registerParameter(&batRobot._consigneDist._accelerationMaxParcourt, "Acceleration lineaire");
	batCom.registerParameter(&batRobot._consigneDist._vitessMax, "Vitesse lineaire", &setVitesseLineare);
	batCom.registerParameter(&batRobot._consigneOrientation._accelerationMaxParcourt, "Acceleration rot");
	batCom.registerParameter(&batRobot._consigneOrientation._vitessMax, "Vitesse rot", &setVitesseRot);
	batCom.registerParameter(&batRobot._consigneDist._dccCoeff, "coeff freinage dist");
	batCom.registerParameter(&batRobot._consigneOrientation._dccCoeff, "coeff freinage rot");
	batCom.registerParameter(&batRobot._consigneDist._dccAugmetationDcc, "coeff augment deceleration");
	batCom.registerParameter(&batRobot.coeffDetectionObst, "coeff detection adv");

#ifdef GRAPH_VCC
	String vccParams[2] = {"vcc lin", "vcc rot"};
	batCom.registerGraph(VccGraph, CurveGraph, "Vitesse", vccParams);
#endif

#ifdef GRAPH_ULTRASON
	String ultrasonParams[4] = {"AV DR", "AV GA", "AR DR", "AR GA"};
	batCom.registerGraph(UltrasonGraph, BarGraph, "Sonars", ultrasonParams);
#endif

    //servoArG.attach(PIN_SERVO_G, 900, 2500);
    //servoArD.attach(PIN_SERVO_D, 900, 2500);

    //servoArG.detach();
    //servoArD.detach();

	// x => 430
	// y => 170
	// theta 36.87 => 0.64

	//batRobot.ajoutPoint(200, -50, false);
	//batRobot.ajoutPoint(600, 0, true);
    //batRobot.ajoutPoint(400, 0, false);
    //batRobot.ajoutPoint(600, -50, false);
    //batRobot.ajoutPoint(800, -0, false);
    //batRobot.ajoutPoint(1000, -50, true);

	Serial.begin(115200);

	delay(2000);
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

	setLedRGB(0, 0, 255);	// bleu = waiting for PC

	batRobot.disableColorSensor(Robot::ColorSensor1);
	batRobot.disableColorSensor(Robot::ColorSensor2);

	// wait for PC to be ready
	while (ENABLE_PC_COMM && !batRobot._pingReceived)
	{
		if (commLect.ready())
			batCom.comm_read();

		if (oneSecond.ready())
			batCom.sendInit();
	}



#ifndef NO_JACK
	bool jackPlugged = digitalRead(PIN_JACK) == HIGH;

	if (!jackPlugged)
	{
		batLogger.println("Please plug the jack");

		while(!jackPlugged)
		{
			jackPlugged = digitalRead(PIN_JACK) == HIGH;

			if (commLect.ready())
				batCom.comm_read();
		}
	}

	delay(2000);

	batLogger.println("Please unplug the jack");
    
    while(jackPlugged)
	{
		bool estJaune = readColor();
		jackPlugged = digitalRead(PIN_JACK) == HIGH;

		if (commLect.ready())
			batCom.comm_read();
	}
#endif

	bool estJaune = readColor();

	batLogger.print("Selected color: ");
	batLogger.println(estJaune ? "Yellow" : "Red");

	// x => 430
	// y => 170
	// theta 36.87 => 0.64

	Point pt;
	pt.x = 430;
	pt.y = estJaune ? 2830 : 170;
	pt.theta = estJaune ? -0.93 : 0.93;

	batRobot.teleport(pt);

	if (estJaune)
	{
		batRobot.ajoutPoint(600, 2703, true);
		batRobot.ajoutPoint(600, 1900, true);
		batRobot.ajoutPoint(600, 1100, true);
	}
	else
	{
		batRobot.ajoutPoint(600, 297, true);
		batRobot.ajoutPoint(600, 1100, true);
		batRobot.ajoutPoint(600, 1900, true);
	}

	//batRobot.ajoutPoint(1000, 0, true);

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

	batLogger.println("Here we gooooo!");

	batCom.sendGo(estJaune);
}

/*********************************************************************************/
/*                                 Main loop                                     */
/*********************************************************************************/


void loop()
{
	if (commLect.ready())
		batCom.comm_read();

	if (commEcrit.ready())
		batCom.sendPosition();

	if (sonar.ready())
		litEtEnvoieSonar();

	if (readColorSensors.ready())
		batCom.sendColorSensorsEvents();

	if (odometry.ready())
		MAJPosition();

    if (asservissement.ready())
    {
        batRobot.vaVersPointSuivant();
        batRobot.calculConsigne();
        batRobot.calculCommande();
        envoiConsigne();

		if(batRobot.passageAuPointSuivant())
        {
            batCom.sendConsigne();
			batCom.sendEvent(EVENT_IS_ARRIVED);
        }

		if(debugEnvoie.ready())
		{
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

			#ifdef GRAPH_VCC
				float vccValues[2] = {batRobot._consigneDist._distDcc, batRobot._consigneOrientation._distDcc};
				batCom.sendGraphValues(VccGraph, vccValues, 2);
			#endif

			#ifdef DEBUG_PID
			if (!batRobot._consigneDist.estArrive())
			{
						batLogger.print("D D: ");
						batLogger.print(batRobot._consigneDist._distDemande);
						batLogger.print(" R: ");
						batLogger.print(batRobot._consigneOrientation._distDemande);
						batLogger.print("C D: ");
						batLogger.print(batRobot._consigneDist._consigne);
						batLogger.print(" R: ");
						batLogger.print(batRobot._consigneOrientation._consigne);
						batLogger.print(" ");
						batLogger.print("P D: ");
						batLogger.print(batRobot._consigneDist._phase);
						batLogger.print(" R: ");
						batLogger.print(batRobot._consigneOrientation._phase);
						batLogger.print(" ");
						batLogger.print(" PCr D: ");
						batLogger.print(batRobot._pidDist._correction);
						batLogger.print(" R: ");
						batLogger.print(batRobot._pidOrientation._correction);
						batLogger.print(" ");
						batLogger.print(" PCo D:");
						batLogger.print(batRobot._pidDist._commande);
						batLogger.print(" R: ");
						batLogger.print(batRobot._pidOrientation._commande);
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

		}	// debugEnvoie

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

