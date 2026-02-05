int ledRed = 13;
int button1 = 10;
int button2 = 11;
bool buttonClick1 = false; //button not pressed yet
bool buttonClick2 = false;


void setup() {
  Serial.begin(9600);
  pinMode (ledRed, OUTPUT);
  pinMode (button1, INPUT);
  pinMode (button2, INPUT);
  digitalWrite (ledRed, LOW);

}

void loop() {
  if(buttonClick1 == false && buttonClick2 == false){
    digitalWrite(ledRed, HIGH);
    delay(1000);
    digitalWrite(ledRed, LOW);
    delay(1000);
    Serial.print(buttonClick1);
  } 

int buttonState1 = digitalRead(button1); //detects if button is clicked or not
  if(buttonState1 == LOW) {
    Serial.print("loop entered");
    buttonClick1 = true;
 }
  if(buttonClick1 == true){
    digitalWrite(ledRed, HIGH);
    delay(500);
    digitalWrite(ledRed, LOW);
    delay(500);
    buttonClick2 = false;
  }
  int buttonState2 = digitalRead(button2); //current state of button 2
  if(buttonState2 == LOW){
    Serial.print("loop 2 entered");
    buttonClick2 = true; //switch that holds the click
  }
  if(buttonClick2 == true){
    digitalWrite(ledRed, HIGH);
    delay(2000);
    digitalWrite(ledRed, LOW);
    delay(2000);
    buttonClick1 = false;
  }
}
