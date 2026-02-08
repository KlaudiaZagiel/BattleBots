const int LEDRED = 13;
const int LEDGREEN = 11;
const int LEDYELLOW = 12;
const int BUTTON = 10;
int buttonState = 0;

void setup() {
 Serial.begin(9600); // this tells me what the status is of my software 
 pinMode(LEDRED, OUTPUT);
 pinMode(LEDGREEN, OUTPUT);
 pinMode(LEDYELLOW, OUTPUT);
 pinMode(BUTTON, INPUT);
 digitalWrite(LEDRED, LOW); //red light starts on
 digitalWrite(LEDGREEN, HIGH); //green light starts off
 digitalWrite(LEDYELLOW, HIGH); //yellow light starts off
}

void loop() {
  int buttonState = digitalRead(BUTTON); //read whatever the button is doing and store it
  if (buttonState == LOW){ 
    digitalWrite(LEDRED, HIGH);
    digitalWrite(LEDGREEN, LOW);
    delay(3000);
    digitalWrite(LEDGREEN, HIGH); //after 3s turn off the green led
    digitalWrite(LEDYELLOW, LOW);
    delay(1000);
    digitalWrite(LEDYELLOW, HIGH);
    digitalWrite(LEDRED, LOW);
 }
}
