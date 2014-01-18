
/* 
!!!!!!

ne pas oublier de changer la frequence de la pwm dans :
arduino-1.5.5\hardware\arduino\sam\variants\arduino_due_x\variant.h L182
!!!!
*/

const int sensG = 8; //2
const int pwmG = 31;
const int bG = 32;
const int sensD = 9; //3
const int pwmD = 33;
const int bD = 34;

const int minPwm = 0;
const int neutralPwm = 2047;
const int maxPwm = 4095;
const int step = 1;
const int tempo = 1;

void setup() 
{   
   pinMode(pwmG, OUTPUT);  
   pinMode(pwmD, OUTPUT);  
   pinMode(bG, OUTPUT);  
   pinMode(bD, OUTPUT);  
   
   digitalWrite(pwmG, HIGH); //HIGH = on, LOW = off
   digitalWrite(pwmD, HIGH); //HIGH = on, LOW = off
   digitalWrite(bG, LOW);
   digitalWrite(bD, LOW); 
   
   analogWriteResolution(12);
   //analogWrite(sensG, neutralPwm);
   //analogWrite(sensD, neutralPwm);
     
   Serial.begin(115200);
   
   Serial.print("Start your engines");
   
  /*for(int i = 5; i > 0; --i)
  {
    Serial.println(i);
    delay(1000);
  }*/
   
   Serial.print("Go go go");
    
  analogWrite(sensG, neutralPwm + (maxPwm - neutralPwm) * .2);
  analogWrite(sensD, neutralPwm + (maxPwm - neutralPwm) * .2);
}

void loop() 
{
  
    /*
    for(int i = neutralPwm; i < maxPwm; i+=step)
    {
      analogWrite(sensG, i);
      analogWrite(sensD, i);
      delay(tempo);
      Serial.println(i);
    }
    
    delay(2000);
    
    for(int i = maxPwm; i >= neutralPwm; i-=step)
    {
      analogWrite(sensG, i);
      analogWrite(sensD, i);
      delay(tempo);
      Serial.println(i);
    }
    
    analogWrite(sensG, neutralPwm);
    analogWrite(sensD, neutralPwm);
    
    delay(5000);
    
    
     for(int i = neutralPwm; i < minPwm; i-=step)
    {
      analogWrite(sensG, i);
      analogWrite(sensD, i);
      delay(tempo);
      Serial.println(i);
    }
    
    delay(2000);
    
    for(int i = minPwm; i >= neutralPwm; i+=step)
    {
      analogWrite(sensG, i);
      analogWrite(sensD, i);
      delay(tempo);
      Serial.println(i);
    }
    
    analogWrite(sensG, neutralPwm);
    analogWrite(sensD, neutralPwm);
    
    delay(5000);
    */
}
