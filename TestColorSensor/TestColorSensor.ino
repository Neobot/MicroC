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
Adafruit_TCS34725 tcs = Adafruit_TCS34725(&Wire, TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_1X);

void setup(void) {
  Serial.begin(115200);
  
  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }
  
  tcs.setInterrupt(true); // disable led
}

void loop(void) {
  float hue;
  
  tcs.setInterrupt(false); // enable led
  delay(170); // give time to power up + take one meassurement
  hue = tcs.getColor();  // returns hue in degree (0=360=red, 60=yellow, 120=green, 180=cyan, 240=blue, 300=magenta)
  tcs.setInterrupt(true);

  Serial.print("hue: "); Serial.print(hue, 3);
  Serial.print(" color: ");
  
  if (hue > 30 and hue < 90)
    Serial.print("yellow");
   else if (hue < 150)
    Serial.print("green");
   else if (hue < 210)
    Serial.print("cyan");
   else if (hue < 270)
    Serial.print("blue");
   else if (hue < 330)
    Serial.print("magenta");
   else
    Serial.print("red");
  
  Serial.println();
  
  delay(1000);
}
