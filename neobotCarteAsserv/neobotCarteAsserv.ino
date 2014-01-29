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
 * Date : 17/04/2012
 * Author : Neobot
 */



/* 
 *
 * !!!!!! IMPORTANT !!!!!!
 *
 *  Ne pas oublier de changer la frequence de la pwm dans :
 *   arduino-1.5.1r2\hardware\arduino\sam\variants\arduino_due_x\variant.h L182
 *
 * !!!!!! IMPORTANT !!!!!!
 */


//#include <arduino.h>
#include <math.h>
#include <Servo.h>
#include "QueueList.h" // fifo
#include <Task.h>
#include "Protocol.h"
#include "Robot.h"
#include "com.h"
#include "logger.h"



/************* DEBUG ***************/

//#define DEBUG_ENCODER
//#define DEBUG_ENCODER_DIFF
//#define DEBUG_POSITION

//#define DEBUG_CONSIGNE_LIN
//#define DEBUG_CONSIGNE_ROT
#define DEBUG_ULTRASON
//#define DEBUG_ULTRASON
//#define DEBUG_COUNTDOWN
//#define DEBUG_NO_JACK
//#define NO_TPS_MATCH

#define USE_PC_COMM_DEBUG true //if true, debug is sent to the PC through the "sendLog" instruction



/*****************************************************
 *                   Config                          *
 *****************************************************/

#define PIN_SENS_MOTEUR_GAUCHE 2
#define PIN_SENS_MOTEUR_DROITE 3

#define PIN_SERVO_1 4
#define PIN_SERVO_2 5
#define PIN_SERVO_3 6
#define PIN_SERVO_4 7

#define PIN_PWM_COLOR_R 8 //ok
#define PIN_PWM_COLOR_B 9 //ok
#define PIN_PWM_COLOR_G 10 //ok

#define UM6_1 16
#define UM6_2 17

#define PIN_JACK 22 //ok
#define PIN_BOUTON_1 23

#define PIN_INTERUPTEUR_COULEUR 24 //ok

#define PIN_CONTACTEUR_1 25
#define PIN_CONTACTEUR_2 26
#define PIN_CONTACTEUR_3 27
#define PIN_CONTACTEUR_4 28
#define PIN_CONTACTEUR_5 29
#define PIN_CONTACTEUR_6 30

#define PIN_MOTEUR_1_PWM_DIGITAL 31
#define PIN_MOTEUR_1_BREAK 32
#define PIN_MOTEUR_2_PWM_DIGITAL 33
#define PIN_MOTEUR_2_BREAK 34

//FPGA 35->50
#define PIN_FIRST_PART 35		//select 0
#define PIN_SECOND_PART 36		//select 1
#define PIN_WHEEL 37			//select 2

#define PIN_WORD7 38
#define PIN_WORD6 39
#define PIN_WORD5 40
#define PIN_WORD4 41
#define PIN_WORD3 42
#define PIN_WORD2 43
#define PIN_WORD1 44
#define PIN_WORD0 45

#define PIN_BOUTON_2 51


#define PIN_SHARP_1 0 //ok
#define PIN_SHARP_2 1 //ok
#define PIN_SHARP_3 2 //ok
#define PIN_SHARP_4 3 //ok
#define PIN_SHARP_5 4 //ok
#define PIN_SHARP_6 5 //ok

#define PIN_SONAR_AV_G 6 //ok
#define PIN_SONAR_AV_D 7 //ok
#define PIN_SONAR_AR_G 8 //ok
#define PIN_SONAR_AR_D 9 //ok


#define PERIODE_ASSERV_MS 5.0
#define PERIODE_COM_LECTURE 50.0
#define PERIODE_COM_ECRITURE 50.0

#define TPS_MATCH 90000

/************* Task ***************/

Task asservissement(PERIODE_ASSERV_MS);
Task commLect(PERIODE_COM_LECTURE);
Task commEcrit(PERIODE_COM_ECRITURE);

/************* Global ***************/

Robot *batRobot;
Comm *batCom;
Logger* batLogger;

Servo servoArG;
Servo servoArD;

bool estViolet = true;

unsigned int initEncodeurG = (1 << 30);
unsigned int initEncodeurD = (1 << 30);

unsigned long tempsMatch;

/************* Function ***************/

unsigned int readOneEncodeurWord()
{
    unsigned int total = digitalRead(PIN_WORD7);
    total = (total << 1) + digitalRead(PIN_WORD6);
    total = (total << 1) + digitalRead(PIN_WORD5);
    total = (total << 1) + digitalRead(PIN_WORD4);
    total = (total << 1) + digitalRead(PIN_WORD3);
    total = (total << 1) + digitalRead(PIN_WORD2);
    total = (total << 1) + digitalRead(PIN_WORD1);
    total = (total << 1) + digitalRead(PIN_WORD0);

    return total;
}

unsigned int readEncoder(bool wheel)
{
    digitalWrite(PIN_WHEEL, wheel);
    digitalWrite(PIN_SECOND_PART, HIGH);
    digitalWrite(PIN_FIRST_PART, HIGH);

    unsigned int total = readOneEncodeurWord();

    digitalWrite(PIN_FIRST_PART, LOW);

    total = (total << 8) + readOneEncodeurWord();

    digitalWrite(PIN_SECOND_PART, LOW);
    digitalWrite(PIN_FIRST_PART, HIGH);

    total = (total << 8) + readOneEncodeurWord();

    digitalWrite(PIN_FIRST_PART, LOW);

    total = (total << 8) + readOneEncodeurWord();

    return total;
}

void MAJPosition()
{
    // En deux fois à cause du cast qui fait peter la resoltion (si direct float => pas = 128)
    int ddg = -initEncodeurG;
    int dg = readEncoder(0) + ddg;

    int ddd = - readEncoder(1);
    int dd = initEncodeurD + ddd;

#ifdef DEBUG_ENCODER_DIFF
    batLogger->print("dg=");
    batLogger->print(dg);
    batLogger->print(" dd=");
    batLogger->print(dd);
    batLogger->println(" ");
#endif

    batRobot->majPosition((float) dg, (float) dd);
}

void envoiConsigne()
{
    //digitalWrite(PIN_SENS_MOTEUR_GAUCHE, !batRobot->_sensAvantRoueGauche);
    //digitalWrite(PIN_SENS_MOTEUR_DROITE, batRobot->_sensAvantRoueDroite);

    analogWrite(PIN_SENS_MOTEUR_GAUCHE, batRobot->_commmandeRoueGauche);
    analogWrite(PIN_SENS_MOTEUR_DROITE, batRobot->_commmandeRoueDroite);

}


int seuil(int mini, int value, int maxi)
{
    value = max(mini, value);
    value = min(maxi, value);

    return value;
}

void litEtEnvoieSonar()
{
    int ag = analogRead(PIN_SONAR_AV_G);
    int ad = analogRead(PIN_SONAR_AV_D);
    int rg = analogRead(PIN_SONAR_AR_G);
    int rd = analogRead(PIN_SONAR_AR_D);

    int valmax = 397;

    ag = seuil(0, ag, valmax);
    ad = seuil(0, ad, valmax);
    rg = seuil(0, rg, valmax);
    rd = seuil(0, rd, valmax);

    ag = map(ag, 0, valmax, 0, 255);
    ad = map(ad, 0, valmax, 0, 255);
    rg = map(rg, 0, valmax, 0, 255);
    rd = map(rd, 0, valmax, 0, 255);

#ifdef DEBUG_ULTRASON
    batLogger->print("ag=");
    batLogger->print(ag);
    batLogger->print(" ad=");
    batLogger->print(ad);
    batLogger->print(" rg=");
    batLogger->print(rg);
    batLogger->print(" rd=");
    batLogger->print(rd);
    batLogger->println(" ");
#endif

    batCom->sendSonars(ag, ad, rg, rd);
}

void bougeServo()
{

}

void setLedRGB(int r, int g, int b)
{
    r = seuil(0, r, 255);
    g = seuil(0, g, 255);
    b = seuil(0, b, 255);

    r = map(r, 0, 255, 0, 65535);
    g = map(g, 0, 255, 0, 65535);
    b = map(b, 0, 255, 0, 65535);

    analogWrite(PIN_PWM_COLOR_R, r);
    analogWrite(PIN_PWM_COLOR_G, g);
    analogWrite(PIN_PWM_COLOR_B, b);
}

int readColor()
{
    int color = digitalRead(PIN_INTERUPTEUR_COULEUR);
    if (color == HIGH)
        setLedRGB(255, 0, 0);
    else
        setLedRGB(0, 0, 255);

    return color;
}

/*****************************/
/* Initialisation du syteme  */
/*****************************/

void setup()
{

    pinMode(PIN_WHEEL, OUTPUT);
    pinMode(PIN_FIRST_PART, OUTPUT);
    pinMode(PIN_SECOND_PART, OUTPUT);

    //pinMode(PIN_SENS_MOTEUR_GAUCHE, OUTPUT);
    //pinMode(PIN_SENS_MOTEUR_DROITE, OUTPUT);

    pinMode(PIN_MOTEUR_1_PWM_DIGITAL, OUTPUT);
    pinMode(PIN_MOTEUR_1_BREAK, OUTPUT);

    digitalWrite(PIN_MOTEUR_1_PWM_DIGITAL, HIGH);
    digitalWrite(PIN_MOTEUR_1_BREAK, LOW);

    pinMode(PIN_MOTEUR_2_PWM_DIGITAL, OUTPUT);
    pinMode(PIN_MOTEUR_2_BREAK, OUTPUT);

    digitalWrite(PIN_MOTEUR_2_PWM_DIGITAL, HIGH);
    digitalWrite(PIN_MOTEUR_2_BREAK, LOW);

    analogWriteResolution(16);
    analogReadResolution(12);

    analogWrite(PIN_SENS_MOTEUR_GAUCHE, 32767);
    analogWrite(PIN_SENS_MOTEUR_DROITE, 32767);

    pinMode(PIN_WORD7, INPUT);
    pinMode(PIN_WORD6, INPUT);
    pinMode(PIN_WORD5, INPUT);
    pinMode(PIN_WORD4, INPUT);
    pinMode(PIN_WORD3, INPUT);
    pinMode(PIN_WORD2, INPUT);
    pinMode(PIN_WORD1, INPUT);
    pinMode(PIN_WORD0, INPUT);

    //pinMode(PIN_OUTPOUT_MICROSWITCH, OUTPUT);
    //digitalWrite(PIN_OUTPOUT_MICROSWITCH, HIGH);
    //pinMode(PIN_MICROSWITCH_AD, INPUT_PULLUP);
    //pinMode(PIN_MICROSWITCH_AG, INPUT_PULLUP);
    //digitalWrite(PIN_MICROSWITCH_AD, LOW); //pull-down
    //digitalWrite(PIN_MICROSWITCH_AG, LOW); //pull-down

    //pinMode(PIN_RESART_GROUND, OUTPUT);
    //digitalWrite(PIN_RESART_GROUND, LOW);
    //pinMode(PIN_RESTART, INPUT);
    //digitalWrite(PIN_RESTART, HIGH);

    Serial.begin(115200);
    SerialUSB.begin(115200);

    batLogger->println("Veuillez plugger le jack.");

    //pinMode(PIN_SONAR_AV_G, INPUT);
    //pinMode(PIN_SONAR_AV_D, INPUT);
    //pinMode(PIN_SONAR_AR_G, INPUT);
    //pinMode(PIN_SONAR_AR_D, INPUT);
    setLedRGB(0, 255, 0); //vert


    pinMode(PIN_JACK, INPUT_PULLUP);
    //digitalWrite(PIN_JACK, LOW); //pull-down

    pinMode(PIN_INTERUPTEUR_COULEUR, INPUT);
    digitalWrite(PIN_INTERUPTEUR_COULEUR, HIGH); //pull-up

    //servoArG.attach(PIN_SERVO_G, 900, 2500);
    //servoArD.attach(PIN_SERVO_D, 900, 2500);

    //Serial.println("He we gooooo");
    batRobot->MAJContaineur(true, 3);
    batRobot->MAJContaineur(false, 3);

    servoArG.detach();
    servoArD.detach();

    batRobot = new Robot(&servoArG, &servoArD, PERIODE_ASSERV_MS);
    batCom = new Comm(batRobot);
    batLogger = new Logger(batCom, USE_PC_COMM_DEBUG);

    //batRobot->ajoutPoint(200, -50, false);
    //batRobot->ajoutPoint(300, 0, true);
    //batRobot->ajoutPoint(400, 0, false);
    //batRobot->ajoutPoint(600, -50, false);
    //batRobot->ajoutPoint(800, -0, false);
    //batRobot->ajoutPoint(1000, -50, true);

    setLedRGB(0, 255, 0);

    //int restartBtn = digitalRead(PIN_RESTART);
    //int oldRestartBtnValue = digitalRead(PIN_RESTART);

    /*while(!batRobot->_pingReceived)
  {
    batCom->comm_read();
    restartBtn = digitalRead(PIN_RESTART);
    if (oldRestartBtnValue != restartBtn)
    {
      batCom->restart();
      oldRestartBtnValue = restartBtn;
      batLogger->println("Restart PC");
    }

  }*/


#ifndef DEBUG_NO_JACK
    setLedRGB(0, 255, 0);
    bool jackPlugged = true;

    //delay(2000);
    batLogger->println("Veuillez de-plugger le jack.");
    
    while(jackPlugged)
    {
        readColor();
        jackPlugged = digitalRead(PIN_JACK) == LOW;
    }
#endif

    batLogger->println("He we gooooo");
    batLogger->print(batRobot->position.x);
    batLogger->print(" ");
    batLogger->print(batRobot->position.y);
    batLogger->print(" ");
    batLogger->print(batRobot->position.theta);
    batLogger->println(" ");

    batLogger->println("");

    initEncodeurD = readEncoder(1);
    initEncodeurG = readEncoder(0);
    tempsMatch = millis();

#ifdef DEBUG_COUNTDOWN
    for(int i = 5; i > 0; --i)
    {
        batLogger->println(i);
        delay(1000);
    }
#endif

    initEncodeurD = readEncoder(1);
    initEncodeurG = readEncoder(0);
    batRobot->passageAuPointSuivant();
    batRobot->vaVersPointSuivant();
    batCom->sendGo(estViolet);

    int color = readColor(); //HIGH = RED, LOW = BLUE
    batCom->sendGo(color == LOW);

}

void loop()
{
    if (commLect.ready())
    {
        batCom->comm_read();
    }

    if (commEcrit.ready())
    {
        batCom->sendPosition();
        litEtEnvoieSonar();
    }

    if (asservissement.ready())
    {
        MAJPosition();
        batRobot->vaVersPointSuivant();
        batRobot->calculConsigne();
        batRobot->calculCommande();
        envoiConsigne();
        if(batRobot->passageAuPointSuivant())
        {
            batCom->sendConsigne();
            batCom->sendIsArrived();
        }

#ifdef DEBUG_CONSIGNE_LIN
        if (!batRobot->_consigneDist->estArrive())
        {
            batLogger->print("D: Consigne:");
            batLogger->print(batRobot->_consigneDist->_consigne);
            batLogger->print(" dr:");
            batLogger->print(batRobot->_consigneDist->_distDemande - batRobot->_consigneDist->_distRealise);
            batLogger->print(" Dcc:");
            batLogger->print(batRobot->_consigneDist->_distDcc);
            batLogger->print(" Vc:");
            batLogger->print(batRobot->_consigneDist->_vitessCourrante);
            if (batRobot->_consigneDist->estArrive())
                batLogger->print(" ARRIVE");
            batLogger->println(" ");
#ifdef DEBUG_CONSIGNE_LIN
            if (!batRobot->_consigneDist->estArrive())
            {
                batLogger->print("D: Consigne:");
                batLogger->print(batRobot->_consigneDist->_consigne);
                batLogger->print(" dr:");
                batLogger->print(batRobot->_consigneDist->_distDemande - batRobot->_consigneDist->_distRealise);
                batLogger->print(" Dcc:");
                batLogger->print(batRobot->_consigneDist->_distDcc);
                batLogger->print(" Vc:");
                batLogger->print(batRobot->_consigneDist->_vitessCourrante);
                if (batRobot->_consigneDist->estArrive())
                    batLogger->print(" ARRIVE");
                batLogger->println(" ");
            }
#endif

#ifdef DEBUG_CONSIGNE_ROT
            if (!batRobot->_consigneOrientation->estArrive())
            {
                batLogger->print("R: Consigne:");
                batLogger->print(batRobot->_consigneOrientation->_consigne);
                batLogger->print(" dr:");
                batLogger->print(batRobot->_consigneOrientation->_distDemande - batRobot->_consigneOrientation->_distRealise);
                batLogger->print(" Dcc:");
                batLogger->print(batRobot->_consigneOrientation->_distDcc);
                batLogger->print(" Vc:");
                batLogger->print(batRobot->_consigneOrientation->_vitessCourrante);
                if (batRobot->_consigneOrientation->estArrive())
                    batLogger->print(" ARRIVE");
                batLogger->println(" ");
            }
#endif

#ifdef DEBUG_POSITION
            //if (batRobot->_consigneDist->calcEstArrive() == false)
            {
                batLogger->print("xp=");
                batLogger->print(batRobot->pointSuivant.x);
                batLogger->print(" yp=");
                batLogger->print(batRobot->pointSuivant.y);
                batLogger->print(" x=");
                batLogger->print(batRobot->position.x);
                batLogger->print(" y=");
                batLogger->print(batRobot->position.y);
                batLogger->print(" t=");
                batLogger->print(batRobot->position.theta);
                batLogger->println(" ");
            }
#endif

#ifdef DEBUG_ENCODER
            batLogger->print("g=");
            batLogger->print(readEncoder(0));
            batLogger->print(" r=");
            batLogger->print(readEncoder(1));
            batLogger->println(" ");
#endif
        }
#endif

#ifdef DEBUG_CONSIGNE_ROT
        if (!batRobot->_consigneOrientation->estArrive())
            /*
  if(digitalRead(PIN_MICROSWITCH_AD) == LOW || digitalRead(PIN_MICROSWITCH_AG) == LOW)
  {
      batLogger->print("R: Consigne:");
      batLogger->print(batRobot->_consigneOrientation->_consigne);
      batLogger->print(" dr:");
      batLogger->print(batRobot->_consigneOrientation->_distDemande - batRobot->_consigneOrientation->_distRealise);
      batLogger->print(" Dcc:");
      batLogger->print(batRobot->_consigneOrientation->_distDcc);
      batLogger->print(" Vc:");
      batLogger->print(batRobot->_consigneOrientation->_vitessCourrante);
      if (batRobot->_consigneOrientation->estArrive())
        batLogger->print(" ARRIVE");
      batLogger->println(" ");
  }
#endif
    //batCom->sendMicroswitch(digitalRead(PIN_MICROSWITCH_AG) == LOW, digitalRead(PIN_MICROSWITCH_AD) == LOW);
    batLogger->print("d: ");
    batLogger->print(digitalRead(PIN_MICROSWITCH_AD));
    batLogger->print(" g: ");
    batLogger->print(digitalRead(PIN_MICROSWITCH_AG));
    batLogger->println(" ");
  }*/

#endif

#ifndef NO_TPS_MATCH
            if(millis() - tempsMatch >= TPS_MATCH)
            {
                digitalWrite(PIN_MOTEUR_1_PWM_DIGITAL, LOW);
                digitalWrite(PIN_MOTEUR_2_PWM_DIGITAL, LOW);

#ifdef DEBUG_POSITION
                //if (batRobot->_consigneDist->calcEstArrive() == false)
                // servoArG.detach();
                //servoArD.detach();

                batLogger->println("C'est fini");

                while(1)
                {
                    batLogger->print("xp=");
                    batLogger->print(batRobot->pointSuivant.x);
                    batLogger->print(" yp=");
                    batLogger->print(batRobot->pointSuivant.y);
                    batLogger->print(" x=");
                    batLogger->print(batRobot->position.x);
                    batLogger->print(" y=");
                    batLogger->print(batRobot->position.y);
                    batLogger->print(" t=");
                    batLogger->print(batRobot->position.theta);
                    batLogger->println(" ");
                }
#endif

#ifdef DEBUG_ENCODER
                batLogger->print("g=");
                batLogger->print(readEncoder(0));
                batLogger->print(" r=");
                batLogger->print(readEncoder(1));
                batLogger->println(" ");
#endif
                setLedRGB(random(0, 255), random(0, 255), random(0, 255));
                delay(50);
            } //beurkkkk mais bon ...
    }
#endif

}
