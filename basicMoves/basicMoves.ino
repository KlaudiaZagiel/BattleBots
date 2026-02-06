// variable declarations that match the physical wiring
int motorLeft = 8;
int motorRight = 7;

void setup() {
  pinMode(motorLeft, OUTPUT);
  pinMode(motorRight, OUTPUT);

  // Make sure motors are off at the start
  digitalWrite(motorLeft, HIGH);
  digitalWrite(motorRight, HIGH);
}

// This is just a test loop I did really fast to see if my connections were working
void loop() {
  // Turn both motors on
  digitalWrite(motorLeft, LOW);
  digitalWrite(motorRight, LOW);

  delay(3000); // motors run for 3 seconds

  // Turn both motors off
  digitalWrite (motorLeft, HIGH);
  digitalWrite(motorRight, HIGH);

  delay(5000); // wait before repeating
}

// The real functions are these
void moveForward() {
  // Turn both motors on
  digitalWrite(motorLeft, LOW);
  digitalWrite(motorRight, LOW);

  // Time the motors run
  delay(3000);

  // Turn both motors off
  digitalWrite(motorLeft, HIGH);
  digitalWrite(motorRight, HIGH);

  // Wait before running any other code for safety
  delay(5000);
}

void moveBackward() {
}

void turnLeft90() {
}

void turnRight90() {
}

void stopMotors() {
  // disable both motors
}