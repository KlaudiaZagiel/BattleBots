const int TRIGGERPIN = 5;
const int ECHOPIN = 13;
const int MOTORLEFT = 10;
const int MOTORRIGHT = 9;

long duration; // in microseconds
float distance; // in cm

int calibrationTime = 200;
int moveToTheSidesTime = 1000;
int moveAroundObject = 1500;

// Echo needs to be high to work (HIGH = on)
void setup () {
  Serial.begin(9600);
  
  pinMode(TRIGGERPIN, OUTPUT); // pulse sent by the sensor (trigger)
  pinMode(ECHOPIN, INPUT); // signal that goes and returns
  pinMode(MOTORLEFT, OUTPUT);
  pinMode(MOTORRIGHT, OUTPUT);

  stopMotors();
}

void loop() {
  distance = measureDistance();

  Serial.print("Distance: ");
  Serial.println(distance);

  if (distance > 20) {
    moveForward();
  } else {
    avoidObstacle();
  }

  delay(100); // for stability
}

float measureDistance() {
  digitalWrite(TRIGGERPIN, HIGH);
  delayMicroseconds(2);

  digitalWrite(TRIGGERPIN, LOW);
  delayMicroseconds(10);

  digitalWrite(TRIGGERPIN, HIGH);

  /* Here pulseIn is reading how long the echo pin was on in microseconds */
  duration = pulseIn(ECHOPIN, HIGH);

  /* speed of sound is around 0.0343 and we divide by 2 because the echo is
  the signal sent and retuned*/
  float distance = duration * 0.034 / 2;

  return distance;
}

void moveForward() {
  digitalWrite(MOTORLEFT, HIGH);
  digitalWrite(MOTORRIGHT, HIGH);
}

void stopMotors() {
  digitalWrite(MOTORLEFT, LOW);
  digitalWrite(MOTORRIGHT, LOW);
}

void turnLeft90() {
  digitalWrite(MOTORLEFT, LOW);
  digitalWrite(MOTORRIGHT, HIGH);

  delay(calibrationTime); // so it doesn't go immediately to the next action

  stopMotors();
}

void turnRight90() {
  digitalWrite(MOTORLEFT, HIGH);
  digitalWrite(MOTORRIGHT, LOW);

  delay(calibrationTime); // so it doesn't go immediately to the next action

  stopMotors();
}

void avoidObstacle() {
  // Step 1: stop
  stopMotors();
  delay(2000);

  // Step 2: turn right
  turnRight90();

  // Step 3: move around object
  moveForward();
  delay(moveToTheSidesTime);

  // Step 4: turn left
  turnLeft90();

  // Step 5: pass by the object
  moveForward();
  delay(moveAroundObject);

  // Step 6: return to original direction
  turnLeft90();
  moveForward();
  delay(moveToTheSidesTime);

  // Step 7: continue as before
  turnRight90();
}
