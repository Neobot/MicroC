
/* 
!!!!!!

ne pas oublier de changer la frequence de la pwm dans :
arduino-1.5.1r2\hardware\arduino\sam\variants\arduino_due_x\variant.h L182
!!!!
*/

const int SENSG = 23;
const int pwmG = 9;

const int SENSD = 25;
const int pwmD = 8;

const int maxPwm = 65000;
const int step = 500;

void setup() 
{   
   pinMode(SENSG, OUTPUT);
   pinMode(pwmG, OUTPUT);  
   
   pinMode(SENSD, OUTPUT);
   pinMode(pwmD, OUTPUT);  
   
   digitalWrite(SENSG, LOW); //HIGH = avancer; LOW = reculer
   digitalWrite(SENSD, LOW); //LOW = avancer; HIGH = reculer
   
   analogWriteResolution(16);
   analogWrite(pwmG, 0);
   analogWrite(pwmD, 0);
     
   Serial.begin(115200);
   
   Serial.print("Start your engines");
   
  for(int i = 5; i > 0; --i)
  {
    Serial.println(i);
    delay(1000);
  }
   
   Serial.print("Go go go");
   

}

void loop() 
{
    for(int i = 0; i < maxPwm; i+=step)
    {
      analogWrite(pwmG, i);
      analogWrite(pwmD, i);
      delay(5);
      Serial.println(i);
    }
    
    delay(2000);
    /*
    for(int i = maxPwm; i >= 0; i-=step)
    {
      analogWrite(pwmG, i);
      analogWrite(pwmD, i);
      delay(5);
      Serial.println(i);
    }*/
    
    analogWrite(pwmG, 0);
    analogWrite(pwmD, 0);
    
    delay(5000);
    
    
     for(int i = 0; i < maxPwm; i+=step)
    {
      analogWrite(pwmG, i);
      analogWrite(pwmD, i);
      delay(5);
      Serial.println(i);
    }
    
    delay(2000);
    /*
    for(int i = maxPwm; i >= 0; i-=step)
    {
      analogWrite(pwmG, i);
      analogWrite(pwmD, i);
      delay(5);
      Serial.println(i);
    }*/
    
    analogWrite(pwmG, 1010);
    analogWrite(pwmD, 1010);
    
    delay(5000);
    
         for(int i = 0; i < maxPwm; i+=step)
    {
      analogWrite(pwmG, i);
      analogWrite(pwmD, i);
      delay(5);
      Serial.println(i);
    }
    
    delay(2000);
    /*
    for(int i = maxPwm; i >= 0; i-=step)
    {
      analogWrite(pwmG, i);
      analogWrite(pwmD, i);
      delay(5);
      Serial.println(i);
    }*/
    
    analogWrite(pwmG, -maxPwm);
    analogWrite(pwmD, -maxPwm);
    
    delay(5000);
}
