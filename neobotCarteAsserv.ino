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
 
 
 #include <arduino.h>
 #include <math.h>
 #include <Servo.h>
 #include "QueueList.h" // fifo
 #include <Task.h>
 #include "Protocol.h"
 #include "Robot.h"
 #include "com.h"
 
 

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

#define SerialDEBUG Serial



  
/*****************************************************
 *                   Config                          * 
 *****************************************************/
 
 #define PIN_FIRST_PART 40		//select 0
 #define PIN_SECOND_PART 38		//select 1
 #define PIN_WHEEL 39			//select 2
 
 #define PIN_WORD7 37
 #define PIN_WORD6 36
 #define PIN_WORD5 35
 #define PIN_WORD4 34
 #define PIN_WORD3 33
 #define PIN_WORD2 32
 #define PIN_WORD1 31
 #define PIN_WORD0 30
 
 #define PIN_PWM_GAUCHE 9
 #define PIN_PWM_DROITE 8
 
 #define PIN_SENS_MOTEUR_GAUCHE 23
 #define PIN_SENS_MOTEUR_DROITE 25
 
 #define PIN_SERVO_D 49
 #define PIN_SERVO_G 53
 
 #define PIN_SONAR_AV_G 4
 #define PIN_SONAR_AV_D 5
 #define PIN_SONAR_AR_G 6
 #define PIN_SONAR_AR_D 7



 #define PIN_JACK 24
 
 #define PIN_OUTPOUT_MICROSWITCH 28
 #define PIN_MICROSWITCH_AD 29
 #define PIN_MICROSWITCH_AG 52

 #define PIN_INTERUPTEUR_COULEUR 22
 #define PIN_PWM_COLOR_R 2
 #define PIN_PWM_COLOR_B 3
 #define PIN_PWM_COLOR_G 4
 
 #define PIN_RESART_GROUND 48
 #define PIN_RESTART 50
 
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
      SerialDEBUG.print("dg=");
      SerialDEBUG.print(dg);
      SerialDEBUG.print(" dd=");
      SerialDEBUG.print(dd);
      SerialDEBUG.println(" ");
  #endif

  batRobot->majPosition((float) dg, (float) dd);
}

void envoiConsigne()
{

  digitalWrite(PIN_SENS_MOTEUR_GAUCHE, !batRobot->_sensAvantRoueGauche);
  digitalWrite(PIN_SENS_MOTEUR_DROITE, batRobot->_sensAvantRoueDroite);

  analogWrite(PIN_PWM_GAUCHE, batRobot->_commmandeRoueGauche);
  analogWrite(PIN_PWM_DROITE, batRobot->_commmandeRoueDroite);

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
  SerialDEBUG.print("ag=");
  SerialDEBUG.print(ag);
  SerialDEBUG.print(" ad=");
  SerialDEBUG.print(ad);
  SerialDEBUG.print(" rg=");
  SerialDEBUG.print(rg);
  SerialDEBUG.print(" rd=");
  SerialDEBUG.print(rd);
  SerialDEBUG.println(" ");
#endif
  
  batCom->sendSonars(ag, ad, rg, rd);
}

void bougeServo()
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
  
  pinMode(PIN_SENS_MOTEUR_GAUCHE, OUTPUT);
  pinMode(PIN_SENS_MOTEUR_DROITE, OUTPUT);
  
  pinMode(PIN_PWM_GAUCHE, OUTPUT);
  pinMode(PIN_PWM_DROITE, OUTPUT);
  
  //pinMode(PIN_PWM_GAUCHE, OUTPUT);
  //pinMode(PIN_PWM_DROITE, OUTPUT);
  
  analogWriteResolution(16);
  analogReadResolution(12);
  
  analogWrite(PIN_PWM_GAUCHE, 0);
  analogWrite(PIN_PWM_DROITE, 0);
   
  pinMode(PIN_WORD7, INPUT);
  pinMode(PIN_WORD6, INPUT);
  pinMode(PIN_WORD5, INPUT);
  pinMode(PIN_WORD4, INPUT);
  pinMode(PIN_WORD3, INPUT);
  pinMode(PIN_WORD2, INPUT);
  pinMode(PIN_WORD1, INPUT);
  pinMode(PIN_WORD0, INPUT);
  
  pinMode(PIN_OUTPOUT_MICROSWITCH, OUTPUT);
  digitalWrite(PIN_OUTPOUT_MICROSWITCH, HIGH);
  pinMode(PIN_MICROSWITCH_AD, INPUT_PULLUP);
  pinMode(PIN_MICROSWITCH_AG, INPUT_PULLUP);
  digitalWrite(PIN_MICROSWITCH_AD, LOW); //pull-down
  digitalWrite(PIN_MICROSWITCH_AG, LOW); //pull-down
  
  pinMode(PIN_RESART_GROUND, OUTPUT);
  digitalWrite(PIN_RESART_GROUND, LOW);
  pinMode(PIN_RESTART, INPUT);
  digitalWrite(PIN_RESTART, HIGH);
  
  Serial.begin(115200);
  SerialUSB.begin(115200);
  Serial3.begin(115200);
  
  SerialDEBUG.println("Veuillez plugger le jack.");

  pinMode(PIN_SONAR_AV_G, INPUT);
  pinMode(PIN_SONAR_AV_D, INPUT);
  pinMode(PIN_SONAR_AR_G, INPUT);
  pinMode(PIN_SONAR_AR_D, INPUT);
  setLedRGB(0, 255, 0); //vert


  pinMode(PIN_JACK, INPUT_PULLUP);
  //digitalWrite(PIN_JACK, LOW); //pull-down

  pinMode(PIN_INTERUPTEUR_COULEUR, INPUT);
  digitalWrite(PIN_INTERUPTEUR_COULEUR, HIGH); //pull-up
  
  servoArG.attach(PIN_SERVO_G, 900, 2500);
  servoArD.attach(PIN_SERVO_D, 900, 2500);
  
  Serial.begin(115200);
  SerialUSB.begin(115200);
  Serial3.begin(115200);
  Serial.println("He we gooooo");
  batRobot->MAJContaineur(true, 3);
  batRobot->MAJContaineur(false, 3);
  
  servoArG.detach();
  servoArD.detach();
  
  batRobot = new Robot(&servoArG, &servoArD, PERIODE_ASSERV_MS);
  batCom = new Comm(batRobot);
  //batRobot->ajoutPoint(200, -50, false);
  //batRobot->ajoutPoint(300, 0, true);
  //batRobot->ajoutPoint(400, 0, false);
  //batRobot->ajoutPoint(600, -50, false);
  //batRobot->ajoutPoint(800, -0, false);
  //batRobot->ajoutPoint(1000, -50, true);
  
  setLedRGB(0, 255, 0);
  
  int restartBtn = digitalRead(PIN_RESTART);
  int oldRestartBtnValue = digitalRead(PIN_RESTART);
  
  while(!batRobot->_pingReceived)
  {
    batCom->comm_read();
    restartBtn = digitalRead(PIN_RESTART);
    if (oldRestartBtnValue != restartBtn)
    {
      batCom->restart();
      oldRestartBtnValue = restartBtn;
      SerialDEBUG.println("Restart PC");
    }
      
  }
  
  
  #ifndef DEBUG_NO_JACK
    setLedRGB(0, 255, 0);
    bool jackPlugged = true;
 
    //delay(2000);
    SerialDEBUG.println("Veuillez de-plugger le jack.");
    
    while(jackPlugged)
    {
  	  readColor();
  	  jackPlugged = digitalRead(PIN_JACK) == LOW;
    }
  #endif
  
  SerialDEBUG.println("He we gooooo");
  SerialDEBUG.print(batRobot->position.x);
  SerialDEBUG.print(" ");
  SerialDEBUG.print(batRobot->position.y);
  SerialDEBUG.print(" ");
  SerialDEBUG.print(batRobot->position.theta);
  SerialDEBUG.println(" ");

  SerialDEBUG.println("");
  
  initEncodeurD = readEncoder(1);
  initEncodeurG = readEncoder(0);
  tempsMatch = millis();
  
#ifdef DEBUG_COUNTDOWN
  for(int i = 5; i > 0; --i)
  {
    SerialDEBUG.println(i);
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
      SerialDEBUG.print("D: Consigne:");
      SerialDEBUG.print(batRobot->_consigneDist->_consigne);
      SerialDEBUG.print(" dr:");
      SerialDEBUG.print(batRobot->_consigneDist->_distDemande - batRobot->_consigneDist->_distRealise);
      SerialDEBUG.print(" Dcc:");
      SerialDEBUG.print(batRobot->_consigneDist->_distDcc);
      SerialDEBUG.print(" Vc:");
      SerialDEBUG.print(batRobot->_consigneDist->_vitessCourrante);
      if (batRobot->_consigneDist->estArrive())
        SerialDEBUG.print(" ARRIVE");
      SerialDEBUG.println(" ");
  #ifdef DEBUG_CONSIGNE_LIN
    if (!batRobot->_consigneDist->estArrive())
    {
        SerialDEBUG.print("D: Consigne:");
        SerialDEBUG.print(batRobot->_consigneDist->_consigne);
        SerialDEBUG.print(" dr:");
        SerialDEBUG.print(batRobot->_consigneDist->_distDemande - batRobot->_consigneDist->_distRealise);
        SerialDEBUG.print(" Dcc:");
        SerialDEBUG.print(batRobot->_consigneDist->_distDcc);
        SerialDEBUG.print(" Vc:");
        SerialDEBUG.print(batRobot->_consigneDist->_vitessCourrante);
        if (batRobot->_consigneDist->estArrive())
          SerialDEBUG.print(" ARRIVE");
        SerialDEBUG.println(" ");
    }
  #endif

  #ifdef DEBUG_CONSIGNE_ROT
    if (!batRobot->_consigneOrientation->estArrive())
    {
        SerialDEBUG.print("R: Consigne:");
        SerialDEBUG.print(batRobot->_consigneOrientation->_consigne);
        SerialDEBUG.print(" dr:");
        SerialDEBUG.print(batRobot->_consigneOrientation->_distDemande - batRobot->_consigneOrientation->_distRealise);
        SerialDEBUG.print(" Dcc:");
        SerialDEBUG.print(batRobot->_consigneOrientation->_distDcc);
        SerialDEBUG.print(" Vc:");
        SerialDEBUG.print(batRobot->_consigneOrientation->_vitessCourrante);
        if (batRobot->_consigneOrientation->estArrive())
          SerialDEBUG.print(" ARRIVE");
        SerialDEBUG.println(" ");
    }
  #endif
    
  #ifdef DEBUG_POSITION
      //if (batRobot->_consigneDist->calcEstArrive() == false)
      {
        SerialDEBUG.print("xp=");
        SerialDEBUG.print(batRobot->pointSuivant.x);
        SerialDEBUG.print(" yp=");
        SerialDEBUG.print(batRobot->pointSuivant.y);
        SerialDEBUG.print(" x=");
        SerialDEBUG.print(batRobot->position.x);
        SerialDEBUG.print(" y=");
        SerialDEBUG.print(batRobot->position.y);
        SerialDEBUG.print(" t=");
        SerialDEBUG.print(batRobot->position.theta);
        SerialDEBUG.println(" ");
      }    
   #endif

  #ifdef DEBUG_ENCODER
    SerialDEBUG.print("g=");
    SerialDEBUG.print(readEncoder(0));
    SerialDEBUG.print(" r=");
    SerialDEBUG.print(readEncoder(1));
    SerialDEBUG.println(" ");
  #endif
  }
#endif

#ifdef DEBUG_CONSIGNE_ROT
  if (!batRobot->_consigneOrientation->estArrive())
  /*
  if(digitalRead(PIN_MICROSWITCH_AD) == LOW || digitalRead(PIN_MICROSWITCH_AG) == LOW)
  {
      SerialDEBUG.print("R: Consigne:");
      SerialDEBUG.print(batRobot->_consigneOrientation->_consigne);
      SerialDEBUG.print(" dr:");
      SerialDEBUG.print(batRobot->_consigneOrientation->_distDemande - batRobot->_consigneOrientation->_distRealise);
      SerialDEBUG.print(" Dcc:");
      SerialDEBUG.print(batRobot->_consigneOrientation->_distDcc);
      SerialDEBUG.print(" Vc:");
      SerialDEBUG.print(batRobot->_consigneOrientation->_vitessCourrante);
      if (batRobot->_consigneOrientation->estArrive())
        SerialDEBUG.print(" ARRIVE");
      SerialDEBUG.println(" ");
  }
#endif
    //batCom->sendMicroswitch(digitalRead(PIN_MICROSWITCH_AG) == LOW, digitalRead(PIN_MICROSWITCH_AD) == LOW);
    SerialDEBUG.print("d: ");
    SerialDEBUG.print(digitalRead(PIN_MICROSWITCH_AD));
    SerialDEBUG.print(" g: ");
    SerialDEBUG.print(digitalRead(PIN_MICROSWITCH_AG));
    SerialDEBUG.println(" ");
  }*/
  
  #ifndef NO_TPS_MATCH
  if(millis() - tempsMatch >= TPS_MATCH)
  {
    analogWrite(PIN_PWM_GAUCHE, 0);
    analogWrite(PIN_PWM_DROITE, 0);
    
#ifdef DEBUG_POSITION
    //if (batRobot->_consigneDist->calcEstArrive() == false)
    servoArG.detach();
    servoArD.detach();
    
    SerialDEBUG.println("C'est fini");
    
    while(1)
    {
      SerialDEBUG.print("xp=");
      SerialDEBUG.print(batRobot->pointSuivant.x);
      SerialDEBUG.print(" yp=");
      SerialDEBUG.print(batRobot->pointSuivant.y);
      SerialDEBUG.print(" x=");
      SerialDEBUG.print(batRobot->position.x);
      SerialDEBUG.print(" y=");
      SerialDEBUG.print(batRobot->position.y);
      SerialDEBUG.print(" t=");
      SerialDEBUG.print(batRobot->position.theta);
      SerialDEBUG.println(" ");
    }    
 #endif

#ifdef DEBUG_ENCODER
  SerialDEBUG.print("g=");
  SerialDEBUG.print(readEncoder(0));
  SerialDEBUG.print(" r=");
  SerialDEBUG.print(readEncoder(1));
  SerialDEBUG.println(" ");
#endif
      setLedRGB(random(0, 255), random(0, 255), random(0, 255));
      delay(50);
    } //beurkkkk mais bon ...
  }
  #endif
  
}
