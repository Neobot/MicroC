#include <Servo.h>
 
 #define PIN_SERVO_1 47
 #define PIN_SERVO_2 49
 #define PIN_SERVO_3 51
 #define PIN_SERVO_4 53
  
  
  Servo servo1;
  Servo servo2;
  Servo servo3;
  Servo servo4;

void setup() {
  // put your setup code here, to run once:
  //servo1.attach(PIN_SERVO_1);
  servo2.attach(PIN_SERVO_2, 900, 2500);
  servo3.attach(PIN_SERVO_4, 900, 2500);

  delay(5000);
}

void loop() {

  /*
  servo2.write(40);
  servo3.write(155);
  delay(2000);
  servo2.write(110);
  servo3.write(85);
  delay(2000);
  servo2.write(175);
  servo3.write(10);
  delay(2000);
  */
  /*servo3.write(15);
  delay(2000);
  servo3.write(100);
  delay(2000);
  servo3.write(80);*/
  /*
  for( int i=80; i >=0;--i)
  {
   servo3.write(i); 
   delay(1000);
  }*/
  
  servo2.write(160);
  
}
