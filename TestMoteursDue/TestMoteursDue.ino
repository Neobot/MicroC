#include "pwm01.h"

/* 
!!!!!!

ne pas oublier de changer la frequence de la pwm dans :
arduino-1.5.5\hardware\arduino\sam\variants\arduino_due_x\variant.h L182
!!!!
*/

#define PIN_MOTEUR_GAUCHE_SENS 9
#define PIN_MOTEUR_GAUCHE_PWM_DIGITAL 29
#define PIN_MOTEUR_GAUCHE_BREAK 28
#define PIN_MOTEUR_DROITE_SENS 8
#define PIN_MOTEUR_DROITE_PWM_DIGITAL 27
#define PIN_MOTEUR_DROITE_BREAK 26

const int minPwm = 0;
const int neutralPwm = 32767;
const int maxPwm = 65535;
const int step = 10;
const int tempo = 1;

void setup() 
{   
   pinMode(PIN_MOTEUR_GAUCHE_PWM_DIGITAL, OUTPUT);
   pinMode(PIN_MOTEUR_DROITE_PWM_DIGITAL, OUTPUT);
   pinMode(PIN_MOTEUR_GAUCHE_BREAK, OUTPUT);
   pinMode(PIN_MOTEUR_DROITE_BREAK, OUTPUT);
   
   digitalWrite(PIN_MOTEUR_GAUCHE_PWM_DIGITAL, HIGH); //HIGH = on, LOW = off
   digitalWrite(PIN_MOTEUR_DROITE_PWM_DIGITAL, HIGH); //HIGH = on, LOW = off
   digitalWrite(PIN_MOTEUR_GAUCHE_BREAK, LOW);
   digitalWrite(PIN_MOTEUR_DROITE_BREAK, LOW);
   
   //analogWriteResolution(12);
   pwm_set_resolution(16);
	pwm_setup(PIN_MOTEUR_GAUCHE_SENS, 40000, 2);
	pwm_setup(PIN_MOTEUR_DROITE_SENS, 40000, 2);

	//analogWrite(sensG, neutralPwm);
   //analogWrite(sensD, neutralPwm);
     
   Serial.begin(115200);
   
   Serial.println("Start your engines");
   
  /*for(int i = 5; i > 0; --i)
  {
    Serial.println(i);
    delay(1000);
  }*/
   
   Serial.println("Go go go");
    
  //pwm_write_duty(sensG, 32767);
 // pwm_write_duty(sensD, 32767);
  //analogWrite(sensG, neutralPwm + (maxPwm - neutralPwm) * .2);
  //analogWrite(sensD, neutralPwm + (maxPwm - neutralPwm) * .2);
}

void loop() 
{
  

    for(int i = neutralPwm; i < maxPwm; i+=step)
    {
	  pwm_write_duty(PIN_MOTEUR_GAUCHE_SENS, i);
	  pwm_write_duty(PIN_MOTEUR_DROITE_SENS, i);
      delay(tempo);
      Serial.println(i);
    }
    
    delay(2000);
    
    for(int i = maxPwm; i >= neutralPwm; i-=step)
    {
	  pwm_write_duty(PIN_MOTEUR_GAUCHE_SENS, i);
	  pwm_write_duty(PIN_MOTEUR_DROITE_SENS, i);
      delay(tempo);
      Serial.println(i);
    }
    
	pwm_write_duty(PIN_MOTEUR_GAUCHE_SENS, neutralPwm);
	pwm_write_duty(PIN_MOTEUR_DROITE_SENS, neutralPwm);
    
    delay(5000);
    
    
     for(int i = neutralPwm; i < minPwm; i-=step)
    {
	  pwm_write_duty(PIN_MOTEUR_GAUCHE_SENS, i);
	  pwm_write_duty(PIN_MOTEUR_DROITE_SENS, i);
      delay(tempo);
      Serial.println(i);
    }
    
    delay(2000);
    
    for(int i = minPwm; i >= neutralPwm; i+=step)
    {
		pwm_write_duty(PIN_MOTEUR_GAUCHE_SENS, i);
	  pwm_write_duty(PIN_MOTEUR_DROITE_SENS, i);
      delay(tempo);
      Serial.println(i);
    }
    
	pwm_write_duty(PIN_MOTEUR_GAUCHE_SENS, neutralPwm);
	pwm_write_duty(PIN_MOTEUR_DROITE_SENS, neutralPwm);
    
    delay(5000);

}
