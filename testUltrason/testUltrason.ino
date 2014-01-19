 #define PIN_SONAR_AV_G 4
 #define PIN_SONAR_AV_D 5
 #define PIN_SONAR_AR_G 6
 #define PIN_SONAR_AR_D 7

void setup() {
  // put your setup code here, to run once:
  
  pinMode(PIN_SONAR_AV_G, INPUT);
  pinMode(PIN_SONAR_AV_D, INPUT);
  pinMode(PIN_SONAR_AR_G, INPUT);
  pinMode(PIN_SONAR_AR_D, INPUT);
  
  Serial.begin(115200);
}

void loop() {
  int ag = analogRead(PIN_SONAR_AV_G);
  int ad = analogRead(PIN_SONAR_AV_D);
  int rg = analogRead(PIN_SONAR_AR_G);
  int rd = analogRead(PIN_SONAR_AR_D);
  
  Serial.print("ag=");
  Serial.print(ag);
  Serial.print(" ad=");
  Serial.print(ad);
  Serial.print(" rg=");
  Serial.print(rg);
  Serial.print(" rd=");
  Serial.print(rd);
  Serial.println(" ");
  delay(500);
}
