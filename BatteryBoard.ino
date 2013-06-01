#include <Wire.h>
#include <avr/sleep.h>

#define PIN_OK 8
#define PIN_NOK 9
#define PIN_OUT 10
#define PIN_IN 11

void sendCommand(byte code)
{
  Wire.beginTransmission(0x55);
  Wire.write(code);   
  Wire.endTransmission();  
  delay(0.1);
}

void readI2C(int length, byte result[])
{
  int i = 0;
 
  for (int j = 0; j<20 ; j++)
    result[j] = 0;
  
  Wire.requestFrom(0x55, length);
  while(Wire.available())  
  { 
    byte c = Wire.read(); 
    
    if (i < length)
    {
      result[i] = c;
      i++;
    }   
  }
}

byte readOneI2C()
{
  byte c;
  
  Wire.requestFrom(0x55, 1);
  while(Wire.available())  
  { 
    c = Wire.read(); 
  }
  
  return c;
}

void readAndDisplayStatus()
{
  // standard data commands
  byte length = 0;
  byte data[20];
  
  // SOC
  sendCommand(0x02);
  readI2C(2, data);
  Serial.print("SOC:            ");
  Serial.println((byte)data[0] + (byte)data[1]*256);
  
  // remaining capacity
  sendCommand(0x04);
  readI2C(2, data);
  Serial.print("Rem. capacity:  ");
  Serial.println((byte)data[0] + (byte)data[1]*256);
  
  // full charge capacity
  sendCommand(0x06);
  readI2C(2, data);
  Serial.print("Full capacity:  ");
  Serial.println((byte)data[0] + (byte)data[1]*256);
  
  // voltage
  sendCommand(0x08);
  readI2C(2, data);
  Serial.print("Voltage:        ");
  Serial.println((byte)data[0] + (byte)data[1]*256);
  
  // average current
  sendCommand(0x0a);
  readI2C(2, data);
  Serial.print("Average current:");
  Serial.println((byte)data[0] + (byte)data[1]*256);
  
  // temperature
  sendCommand(0x0c);
  readI2C(2, data);
  Serial.print("Temperature:    ");
  Serial.println((double)((byte)data[0] + (byte)data[1]*256)/10 - 273.15);
  
  // flags
  sendCommand(0x0e);
  readI2C(2, data);
  Serial.print("Flags:          ");
  Serial.println((byte)data[0] + (byte)data[1]*256, 2);
  
  // manufacturer name length
  sendCommand(0x6d);
  length = readOneI2C();
  
  // manufacturer name
  sendCommand(0x6e);
  readI2C(length, data);
  Serial.print("Manuf. Name:    ");
  Serial.println((char *)data);
  
  // device chemistry length
  sendCommand(0x79);
  length = readOneI2C();
  
  // device chemistry
  sendCommand(0x7a);
  readI2C(length, data);
  Serial.print("Chemistry:      ");
  Serial.println((char *)data);
  
  // internal temperature
  sendCommand(0x2a);
  readI2C(2, data);
  Serial.print("Int temperature:");
  Serial.println((double)((byte)data[0] + (byte)data[1]*256)/10 - 273.15);
  
  // design capacity
  sendCommand(0x3c);
  readI2C(2, data);
  Serial.print("Design capacity:");
  Serial.println((byte)data[0] + (byte)data[1]*256);
  
  Serial.println();
}

void setPrescaler(uint8_t mode)
{
  cli();
  CLKPR = bit(CLKPCE);
  CLKPR = mode;
  sei();
}

ISR(WDT_vect) {}  // watchdog timer interrupt

void setup() {
  Serial.begin(115200);  // start serial for output
  Wire.begin();        // join i2c bus (address optional for master)
  
  pinMode(PIN_OK, OUTPUT);
  pinMode(PIN_NOK, OUTPUT);
  pinMode(PIN_IN, OUTPUT);
  pinMode(PIN_OUT, OUTPUT);
  
  analogWrite(PIN_IN, 0);
  analogWrite(PIN_OUT, 0);
  digitalWrite(PIN_OK, LOW);
  digitalWrite(PIN_NOK, LOW);
  
  delay(500);
  
  analogWrite(PIN_IN, 255);
  analogWrite(PIN_OUT, 255);
  digitalWrite(PIN_OK, HIGH);
  digitalWrite(PIN_NOK, HIGH);
    
  ADCSRA = 0;  // disable ADC (saves energy)
  
  // set up the watchdog timer to generate interrupts
  MCUSR &= ~(1<<WDRF);
  cli();
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  WDTCSR = 6;  // 1 second
  WDTCSR |= bit(WDIE);
  sei();
}




void loop() 
{
  //setPrescaler(0);
  readAndDisplayStatus();
  //setPrescaler(6);

  byte data[20];
  
  sendCommand(0x0a);
  readI2C(2, data);    // read current
  int current = (byte)data[0] + (byte)data[1]*256;
  
  sendCommand(0x0e);
  readI2C(2, data);  // read flags
  
  int nok = (data[1] >> 3) > 0 || ((data[0] >> 1) & 0x01) || ((data[0] >> 2) & 0x01) || ((data[0] >> 6) & 0x01);
  int ok = !nok;
  
  if (ok)
    digitalWrite(PIN_OK, LOW);
  else
    digitalWrite(PIN_OK, HIGH);
    
  if (nok)
    analogWrite(PIN_NOK, 250); // enough for the LED to be lit
  else
    analogWrite(PIN_NOK, 255);
  
  if (current > 0)
  {
    analogWrite(PIN_OUT, 255);
    analogWrite(PIN_IN, 255-min(current, 255));
  }
  else
  {
    analogWrite(PIN_OUT, 255-min(-current, 255));
    analogWrite(PIN_IN, 255);
  }
  
  // go to sleep (will be awaken by watchdog timer)
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_mode();
}
