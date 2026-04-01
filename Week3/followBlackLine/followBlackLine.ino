const int MOTORLEFTFORWARD = 10;
const int MOTORLEFTBACKWARD = 11;
const int MOTORRIGHTFORWARD = 9;
const int MOTORRIGHTBACKWARD = 6;

const int numSensors = 8;
const int sensorPins[numSensors] = {A0,A1,A2,A3,A4,A5,A6,A7};

const int threshold = 650;   // <-- Need to calibrate using the track at school

void setup() {
  Serial.begin(9600);

  pinMode(MOTORLEFTFORWARD, OUTPUT);
  pinMode(MOTORLEFTBACKWARD, OUTPUT);
  pinMode(MOTORRIGHTFORWARD, OUTPUT);
  pinMode(MOTORRIGHTBACKWARD, OUTPUT);
  // On Arduino, analog pins automatically behave as inputs when using analogRead()
  // Therefore I do not need to "set them up here"
}

void loop() {
  // Counter variables
  int leftDetected = 0;
  int centerDetected = 0;
  int rightDetected = 0;

  // Loop through the every sensor before looping again
  for(int i = 0; i < numSensors; i++) {
    // Read pin value variable
    int value = analogRead(sensorPins[i]);

    // If the pin value is inferior to our threshold, the sensor is detecting black line
    if(value > threshold) {
      // If the sensor detects black on the two sensors on the right,
      // we increase the detection on the right
      if(i <= 1) {
        rightDetected++;
      }
      // If sensor detects black on the middle sensors, we increase
      // the detection on the center
      else if(i <= 5) {
        centerDetected++;
      }
      // If sensor detects black on the two sensors on the left, we
      // increase the detection on the left
      else {
        leftDetected++;
      }
    }
  }

  // With the previous detections we make now decisions and change robot states

  // If any center sensor detects black line, we move forward
  if(centerDetected > 0) {
    moveForward();
    Serial.println("CENTER - Forward");
  }
  // If left sensors detect black line, we turn left for adjustment
  else if(leftDetected > 0) {
    turnLeft();
    Serial.println("LEFT - Turning Left");
  }
  // If right sensors detect black line, we turn right for adjustment
  else if(rightDetected > 0) {
    turnRight();
    Serial.println("RIGHT - Turning Right");
  }
  // If none apply, line was lost and robot stops
  else {
    stopMotors();
    Serial.println("Line Lost");
  }

  delay(50);  // small stabilization delay
}

// Basic movements functions
void moveForward() {
  digitalWrite(MOTORLEFTFORWARD, HIGH);
  digitalWrite(MOTORLEFTBACKWARD, LOW);
  digitalWrite(MOTORRIGHTFORWARD, HIGH);
  digitalWrite(MOTORRIGHTBACKWARD, LOW);
}

void turnLeft() {
  digitalWrite(MOTORLEFTFORWARD, LOW);
  digitalWrite(MOTORLEFTBACKWARD, LOW);
  digitalWrite(MOTORRIGHTFORWARD, HIGH);
  digitalWrite(MOTORRIGHTBACKWARD, LOW);
}

void turnRight() {
  digitalWrite(MOTORLEFTFORWARD, HIGH);
  digitalWrite(MOTORLEFTBACKWARD, LOW);
  digitalWrite(MOTORRIGHTFORWARD, LOW);
  digitalWrite(MOTORRIGHTBACKWARD, LOW);
}

void stopMotors() {
  digitalWrite(MOTORLEFTFORWARD, LOW);
  digitalWrite(MOTORLEFTBACKWARD, LOW);
  digitalWrite(MOTORRIGHTFORWARD, LOW);
  digitalWrite(MOTORRIGHTBACKWARD, LOW);
}
