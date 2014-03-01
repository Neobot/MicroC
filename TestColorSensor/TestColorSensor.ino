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
  
  if (tcs1.begin()) {
	Serial.println("Found sensor 1");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }

  if (tcs2.begin()) {
	Serial.println("Found sensor 2");
  } else {
	Serial.println("No TCS34725 found ... check your connections");
	while (1);
  }
  
  tcs1.setInterrupt(true); // disable led
  tcs2.setInterrupt(true); // disable led
}

void loop(void) {
  float hue1, hue2;
  
  tcs1.setInterrupt(false); // enable led
  tcs2.setInterrupt(false); // enable led
  delay(170); // give time to power up + take one meassurement
  hue1 = tcs1.getColor();  // returns hue in degree (0=360=red, 60=yellow, 120=green, 180=cyan, 240=blue, 300=magenta)
  hue2 = tcs2.getColor();  // returns hue in degree (0=360=red, 60=yellow, 120=green, 180=cyan, 240=blue, 300=magenta)
  tcs1.setInterrupt(true);
  tcs2.setInterrupt(true);

  Serial.print("hue 1: "); Serial.print(hue1, 3);
  Serial.print(" hue 2: "); Serial.print(hue2, 3);
  Serial.print(" color 1: ");
  
  if (hue1 > 30 and hue1 < 90)
    Serial.print("yellow");
   else if (hue1 < 150)
    Serial.print("green");
   else if (hue1 < 210)
    Serial.print("cyan");
   else if (hue1 < 270)
    Serial.print("blue");
   else if (hue1 < 330)
    Serial.print("magenta");
   else
    Serial.print("red");

  Serial.print(" color 2: ");

  if (hue2 > 30 and hue2 < 90)
	Serial.print("yellow");
   else if (hue2 < 150)
	Serial.print("green");
   else if (hue2 < 210)
	Serial.print("cyan");
   else if (hue2 < 270)
	Serial.print("blue");
   else if (hue2 < 330)
	Serial.print("magenta");
   else
	Serial.print("red");
  
  Serial.println();
  
  delay(1000);
}
