#include <Wire.h>
#include "Adafruit_TCS34725.h"

/* Example code for the Adafruit TCS34725 breakout library */

/* Connect SCL    to analog 5
   Connect SDA    to analog 4
   Connect VDD    to 3.3V DC
   Connect GROUND to common ground */
   
/* Initialise with default values (int time = 2.4ms, gain = 1x) */
// Adafruit_TCS34725 tcs = Adafruit_TCS34725();

/* Initialise with specific int time and gain values */
Adafruit_TCS34725 tcs1 = Adafruit_TCS34725(&Wire, TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_1X);
Adafruit_TCS34725 tcs2 = Adafruit_TCS34725(&Wire1, TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_1X);

void setup(void) {
  Serial.begin(115200);
  bool ok = true;

  if (tcs1.begin()) {
	Serial.println("Found sensor 1");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
	ok = false;
  }

  if (tcs2.begin()) {
	Serial.println("Found sensor 2");
  } else {
	Serial.println("No TCS34725 found ... check your connections");
	ok = false;
  }

  if (!ok)
	  while (1);
  
  tcs1.setInterrupt(true); // disable led
  tcs2.setInterrupt(true); // disable led
}

void loop(void) {
  float h1, s1, l1;
  float h2, s2, l2;
  
  tcs1.setInterrupt(false); // enable led
  tcs2.setInterrupt(false); // enable led
  delay(170); // give time to power up + take one meassurement
  tcs1.getColorInHSL(&h1, &s1, &l1);  // returns hue in degree (0=360=red, 60=yellow, 120=green, 180=cyan, 240=blue, 300=magenta)
  tcs2.getColorInHSL(&h2, &s2, &l2);  // returns hue in degree (0=360=red, 60=yellow, 120=green, 180=cyan, 240=blue, 300=magenta)
  tcs1.setInterrupt(true);
  tcs2.setInterrupt(true);

  Serial.print("hue 1: "); Serial.print(h1, 3);
  Serial.print(" hue 2: "); Serial.print(h2, 3);
  Serial.print(" color 1: ");
  
  if (h1 > 30 and h1 < 90)
    Serial.print("yellow");
   else if (h1 < 150)
    Serial.print("green");
   else if (h1 < 210)
    Serial.print("cyan");
   else if (h1 < 270)
    Serial.print("blue");
   else if (h1 < 330)
    Serial.print("magenta");
   else
    Serial.print("red");

  Serial.print(" color 2: ");

  if (h2 > 30 and h2 < 90)
	Serial.print("yellow");
   else if (h2 < 150)
	Serial.print("green");
   else if (h2 < 210)
	Serial.print("cyan");
   else if (h2 < 270)
	Serial.print("blue");
   else if (h2 < 330)
	Serial.print("magenta");
   else
	Serial.print("red");
  
  Serial.println();
  
  delay(1000);
}
