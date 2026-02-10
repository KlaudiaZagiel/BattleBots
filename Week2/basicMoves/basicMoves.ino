// variable declarations that match the physical wiring
const int MOTORLEFTFORWARD = 10; // A1
const int MOTORRIGHTFORWARD = 9; //
const int MOTORLEFTBACKWARD = 11; //
const int MOTORRIGHTBACKWARD = 6; // A2

// 11 is left backwards
// 9 is right forward
// 10 is left forward
// 6 is right backwards

void setup() {
  pinMode(MOTORLEFTFORWARD, OUTPUT);
  pinMode(MOTORRIGHTFORWARD, OUTPUT);
  pinMode(MOTORLEFTBACKWARD, OUTPUT);
  pinMode(MOTORRIGHTBACKWARD, OUTPUT);
}

// This is just a test loop I did really fast to see if my connections were working
void loop() {
  // Turn both motors on
  digitalWrite(MOTORRIGHTBACKWARD, LOW);

  delay(3000); // motors run for 3 seconds

  // Turn both motors off
  digitalWrite (MOTORRIGHTBACKWARD, HIGH);

  delay(3000); // wait before repeating
}
