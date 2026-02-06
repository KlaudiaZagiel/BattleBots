int ledRed = 13;
int ledGreen = 11;
int ledYellow = 12;
int button = 10;
bool ledRedState = true; // this checks if red light is on or off


void setup() {
 Serial.begin(9600); // this tells me what the status is of my software 
 pinMode(ledRed, OUTPUT);
 pinMode(ledGreen, OUTPUT);
 pinMode(ledYellow, OUTPUT);
 pinMode(button, INPUT);
 digitalWrite(ledRed, LOW); //red light starts on
 digitalWrite(ledGreen, HIGH); //green light starts off
 digitalWrite(ledYellow, HIGH); //yellow light starts off
}

void loop() {
  int buttonState = digitalRead(button);  
  if (buttonState == LOW){
    digitalWrite(ledRed, HIGH);
    ledRedState = false;
    Serial.print(ledRedState);
  }
 if (ledRedState == false){
  delay(1000);
  digitalWrite(ledGreen, LOW);
  delay(3000);
  digitalWrite(ledGreen, HIGH); //after 3s turn off the green led
  digitalWrite(ledYellow, LOW);
  delay(1000);
  digitalWrite(ledYellow, HIGH);
  digitalWrite(ledRed, LOW);
  ledRedState = true; // this checks if the light is on (i said the light is LOW before so - on.)
 }
 
}