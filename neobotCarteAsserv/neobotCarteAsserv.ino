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
Task commLect(PERIODE_COM_LECTURE);
Task commEcrit(PERIODE_COM_ECRITURE);
Task oneSecond(1000);
Task readColorSensors(PERIODE_READ_COLOR_SENSOR);
Task debugEnvoie(5);

Adafruit_TCS34725 colorSensor1(&Wire, TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);
Adafruit_TCS34725 colorSensor2(&Wire1, TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);

Robot batRobot(&colorSensor1, &colorSensor2, PERIODE_ASSERV_MS);
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
	int speedL, speedR, speedMotL, speedMotR;

#ifndef SIMULATION
    dg = readEncoder(0, 0, 0) - initEncodeurG;
    dd = readEncoder(1, 0, 0) - initEncodeurD;

	// read encoders speed (in mm/s)
	speedL = readEncoder(0, 0, 1) - 8388608;
	speedR = readEncoder(1, 0, 1) - 8388608;
	speedMotL = readEncoder(0, 1, 1) - 8388608;
	speedMotR = readEncoder(1, 1, 1) - 8388608;
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

	if (batRobot.checkBlocked(speedL, speedMotL, speedR, speedMotR))
		batCom.sendEvent(EVENT_IS_BLOCKED);
}

void envoiConsigne()
{
#ifndef SIMULATION

    bool stopMotG = batRobot._commandeRoueGauche >= ((MAX_PWM_MOTORS - 1.0) / 2.0 - MINI_MOTOR) && batRobot._commandeRoueGauche <= ((MAX_PWM_MOTORS - 1.0) / 2.0 + MINI_MOTOR);
    bool stopMotD = batRobot._commandeRoueDroite >= ((MAX_PWM_MOTORS - 1.0) / 2.0 - MINI_MOTOR) && batRobot._commandeRoueDroite <= ((MAX_PWM_MOTORS - 1.0) / 2.0 + MINI_MOTOR);

    digitalWrite(PIN_MOTEUR_GAUCHE_PWM_DIGITAL, stopMotG ? LOW : HIGH);	// stop motor if command is zero
    digitalWrite(PIN_MOTEUR_DROITE_PWM_DIGITAL, stopMotD ? LOW : HIGH);

#ifdef DEBUG_CONSIGNE_MOTEUR
  
    batLogger.print("g=");
    batLogger.print(batRobot._commandeRoueGauche);
    batLogger.print(" d=");
    batLogger.print(batRobot._commandeRoueDroite);
    batLogger.println(" ");
  
#endif
	pwm_write_duty(PIN_MOTEUR_GAUCHE_SENS, batRobot._commandeRoueGauche);
	pwm_write_duty(PIN_MOTEUR_DROITE_SENS, batRobot._commandeRoueDroite);
#else
    simMotorL.setCommande(batRobot._commandeRoueGauche);
    simMotorR.setCommande(batRobot._commandeRoueDroite);
#endif
}

void litEtEnvoieSonar()
{
	int ag = analogRead(PIN_SONAR_AV_G);	// max 4096 = 1024 cm
    int ad = analogRead(PIN_SONAR_AV_D);
    int rg = analogRead(PIN_SONAR_AR_G);
    int rd = analogRead(PIN_SONAR_AR_D);

	int valmax = 397;		// max 1 meter

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

    batCom.sendSonars(ag, ad, rg, rd);
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
		setLedRGB(255, 128, 0);    // jaune
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

    //register parameters which can be changed trhough the comm
	//Max number of parameters is currently 10, defined in comm.h
    batCom.registerParameter(&batRobot._pidDist._kp, "PID Distance P");
    batCom.registerParameter(&batRobot._pidDist._kd, "PID Distance D");
    batCom.registerParameter(&batRobot._pidOrientation._kp, "PID Orientation P");
    batCom.registerParameter(&batRobot._pidOrientation._kd, "PID Orientation D");
	batCom.registerParameter(&batRobot._consigneDist._accelerationMaxParcourt, "Acceleration lineaire");
	batCom.registerParameter(&batRobot._consigneDist._vitessMax, "Vitesse lineaire");
	batCom.registerParameter(&batRobot._consigneOrientation._accelerationMaxParcourt, "Acceleration rot");
	batCom.registerParameter(&batRobot._consigneOrientation._vitessMax, "Vitesse rot");
	batCom.registerParameter(&batRobot._consigneDist._dccCoeff, "coeff freinage dist");
	batCom.registerParameter(&batRobot._consigneOrientation._dccCoeff, "coeff freinage rot");
	batCom.registerParameter(&batRobot._consigneDist._dccAugmetationDcc, "coeff augment deceleration");


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
	while (!batRobot._pingReceived)
	{
		if (commLect.ready())
			batCom.comm_read();

		if (oneSecond.ready())
			batCom.sendInit();

		if (!ENABLE_PC_COMM)
			break;
	}

#ifndef NO_JACK
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
	{
		bool estJaune = readColor();
		jackPlugged = digitalRead(PIN_JACK) == LOW;
	}
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

	batLogger.println("Here we gooooo!");

	batCom.sendGo(estJaune);
}

/*********************************************************************************/
/*                                 Main loop                                     */
/*********************************************************************************/


void loop()
{
	if (ENABLE_PC_COMM)
	{
		if (commLect.ready())
			batCom.comm_read();

		if (commEcrit.ready())
		{
			batCom.sendPosition();
			litEtEnvoieSonar();
		}
	}

	if (readColorSensors.ready())
	{
		batCom.sendColorSensorsEvents();
	}

    if (asservissement.ready())
    {
		MAJPosition();
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

		}

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

