const int LEDRED = 13;
const int BUTTON1 = 10;
const int BUTTON2 = 9;
bool fasterButton1 = false; //button not pressed yet
bool slowerButton2 = false;


void setup() {
  Serial.begin(9600);
  pinMode (LEDRED, OUTPUT);
  pinMode (BUTTON1, INPUT);
  pinMode (BUTTON2, INPUT);
  digitalWrite (LEDRED, LOW);
}

void loop() {
  int buttonState1 = digitalRead(BUTTON1); //detects if button is clicked or not

  if(buttonState1 == LOW) {
    fasterButton1 = true;
    slowerButton2 = false;
    Serial.print("Fast loop started!\n");
    delay(200);
 }

  int buttonState2 = digitalRead(BUTTON2); //current state of button 2

  if(buttonState2 == LOW){
    fasterButton1 = false;
    slowerButton2 = true;
    Serial.print("Slow loop started!\n");
    delay(200);
  }

  if (!fasterButton1 && !slowerButton2) {
    // Normal blink (1 second)
    digitalWrite(LEDRED, HIGH);
    delay(1000);
    digitalWrite(LEDRED, LOW);
    delay(1000);
  }
  else if (fasterButton1) {
    // Fast blink (2 a second)
    digitalWrite(LEDRED, HIGH);
    delay(500);
    digitalWrite(LEDRED, LOW);
    delay(500);
  }
  else if (slowerButton2) {
    // Slow blink (2 seconds)
    digitalWrite(LEDRED, HIGH);
    delay(2000);
    digitalWrite(LEDRED, LOW);
    delay(2000);
  }
}