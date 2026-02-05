int ledRed = 13;
int ledYellow = 12;
int ledGreen = 11;

void setup(){
  pinMode(ledRed, OUTPUT);
  pinMode(ledYellow, OUTPUT);
  pinMode(ledGreen, OUTPUT);
}

void loop(){
  digitalWrite(ledRed, LOW);
  delay(3000);
  digitalWrite(ledRed, HIGH);
  delay(500);

   digitalWrite(ledYellow, LOW);
  delay(1000);
  digitalWrite(ledYellow, HIGH);
  delay(500);

   digitalWrite(ledGreen, LOW);
  delay(4000);
  digitalWrite(ledGreen, HIGH);
  delay(500);
  
}
