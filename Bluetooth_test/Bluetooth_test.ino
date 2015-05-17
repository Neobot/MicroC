void setup()
{
  Serial.begin(115200);  // Begin the serial monitor at 9600bps

  Serial2.begin(115200);  // The Bluetooth Mate defaults to 115200bps
}

void loop()
{
  if(Serial2.available())  // If the bluetooth sent any characters
  {
    // Send any characters the bluetooth prints to the serial monitor
    Serial.print((char)Serial2.read());  
  }
  if(Serial.available())  // If stuff was typed in the serial monitor
  {
    // Send any characters the Serial monitor prints to the bluetooth
    Serial2.print((char)Serial.read());
  }
  // and loop forever and ever!
}
