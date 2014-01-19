#define PIN_WORD11 38
#define PIN_WORD10 39
#define PIN_WORD09 40
#define PIN_WORD08 41
#define PIN_WORD07 42
#define PIN_WORD06 44
#define PIN_WORD05 45
#define PIN_WORD04 46
#define PIN_WORD03 47
#define PIN_WORD02 48
#define PIN_WORD01 49
#define PIN_WORD00 50

#define PIN_SEL3 35
#define PIN_SEL2 36
#define PIN_SEL1 37
#define PIN_SEL0 43

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

unsigned int readOneEncodeurWord()
{
  unsigned int total = digitalRead(PIN_WORD11);
  total = (total << 1) + digitalRead(PIN_WORD10);
  total = (total << 1) + digitalRead(PIN_WORD09);
  total = (total << 1) + digitalRead(PIN_WORD08);
  total = (total << 1) + digitalRead(PIN_WORD07);
  total = (total << 1) + digitalRead(PIN_WORD06);
  total = (total << 1) + digitalRead(PIN_WORD05);
  total = (total << 1) + digitalRead(PIN_WORD04);
  total = (total << 1) + digitalRead(PIN_WORD03);
  total = (total << 1) + digitalRead(PIN_WORD02);
  total = (total << 1) + digitalRead(PIN_WORD01);
  total = (total << 1) + digitalRead(PIN_WORD00);
  
  return total;
}

unsigned int readEncoder(bool wheel, bool encoder, bool spd)
{
  digitalWrite(PIN_SEL3, spd);
  digitalWrite(PIN_SEL2, encoder);
  digitalWrite(PIN_SEL1, wheel);
  
  digitalWrite(PIN_SEL0, HIGH);
  unsigned int total = readOneEncodeurWord();
  
  digitalWrite(PIN_SEL0, LOW);
  total = (total << 12) + readOneEncodeurWord();
  
  return total;
}

void setup() {
  pinMode(PIN_WORD11, INPUT);
  pinMode(PIN_WORD10, INPUT);
  pinMode(PIN_WORD09, INPUT);
  pinMode(PIN_WORD08, INPUT);
  pinMode(PIN_WORD07, INPUT);
  pinMode(PIN_WORD06, INPUT);
  pinMode(PIN_WORD05, INPUT);
  pinMode(PIN_WORD04, INPUT);
  pinMode(PIN_WORD03, INPUT);
  pinMode(PIN_WORD02, INPUT);
  pinMode(PIN_WORD01, INPUT);
  pinMode(PIN_WORD00, INPUT);
  
  pinMode(PIN_SEL3, OUTPUT);
  pinMode(PIN_SEL2, OUTPUT);
  pinMode(PIN_SEL1, OUTPUT);
  pinMode(PIN_SEL0, OUTPUT);

  digitalWrite(PIN_SEL3, HIGH);
  digitalWrite(PIN_SEL2, HIGH);
  digitalWrite(PIN_SEL1, HIGH);
  digitalWrite(PIN_SEL0, HIGH);
  
  pinMode(pwmG, OUTPUT);  
  pinMode(pwmD, OUTPUT);  
  pinMode(bG, OUTPUT);  
  pinMode(bD, OUTPUT);  
   
  digitalWrite(pwmG, HIGH); //HIGH = on, LOW = off
  digitalWrite(pwmD, HIGH); //HIGH = on, LOW = off
  digitalWrite(bG, LOW);
  digitalWrite(bD, LOW); 
  analogWriteResolution(12);

  Serial.begin(115200);
  
  analogWrite(sensG, maxPwm);
  analogWrite(sensD, maxPwm);
}

void loop() {
  Serial.print(" l=");
  Serial.print(readEncoder(0, 0, 0));
  Serial.print(" r=");
  Serial.print(readEncoder(1, 0, 0));
  Serial.print(" ls=");
  Serial.print(readEncoder(0, 0, 1) - 8388608);
  Serial.print(" rs=");
  Serial.print(readEncoder(1, 0, 1) - 8388608);
  Serial.println(" ");
  
  Serial.print(" l2=");
  Serial.print(readEncoder(0, 1, 0));
  Serial.print(" r2=");
  Serial.print(readEncoder(1, 1, 0));
  Serial.print(" ls2=");
  Serial.print(readEncoder(0, 1, 1) - 8388608);
  Serial.print(" rs2=");
  Serial.print(readEncoder(1, 1, 1) - 8388608);
  Serial.println(" ");
  delay(2000);
}
