#include <Adafruit_NeoPixel.h>

// ==================================================
// Function Prototypes
// ==================================================
void updateWheelPulses();

int readDistanceCm();

void resetPulseCounts();
void stopMotors();

void setForwardSpeeds(int leftSpeed, int rightSpeed);
void startMotorsForward();
void startMotorsForwardPickup();
void startMotorsForwardSearch();
void startMotorsBackward();
void startMotorLeft();
void startMotorRight();

void updateGripperSignal(int newPulse);

void turnAllLedsOff();
void showLeftBlink();
void showRightBlink();
void showBackRed();

int readLineSensorCount();
void readLinePosition(long &weightedTotalSum, long &totalSum);

// ==================================================
// Calibration Zone
// ==================================================

// -------------------------
// LED settings
// -------------------------
const uint8_t ledPin = 8;
const uint8_t numLeds = 4;

const unsigned long blinkIntervalMs = 250;

// IMPORTANT LED ORDER (based on your current working mapping):
// 0 = Left Back
// 1 = Right Back
// 2 = Right Wing / Front Right
// 3 = Left Wing / Front Left

// -------------------------
// Motor pins
// -------------------------
const uint8_t motor1BackwardPin = 11;
const uint8_t motor1ForwardPin = 10;
const uint8_t motor2ForwardPin = 9;
const uint8_t motor2BackwardPin = 6;

// -------------------------
// Wheel encoder / rotation sensors
// -------------------------
const uint8_t rotationWheel1Pin = 2;  // Sensor 1
const uint8_t rotationWheel2Pin = 3;  // Sensor 2

// -------------------------
// Start button
// -------------------------
const uint8_t startButtonPin = 7;

// -------------------------
// Ultrasonic sensor pins
// -------------------------
const uint8_t triggerPin = 4;
const uint8_t echoPin = 13;

// -------------------------
// Servo / gripper
// -------------------------
const uint8_t servoPin = 12;
const int gripperOpenPulse = 1600;
const int gripperClosePulse = 950;

// -------------------------
// Line sensor pins
// -------------------------
const uint8_t sensorPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};
int sensorWeights[8] = {-7, -5, -3, -1, 1, 3, 5, 7};

// -------------------------
// Line following tuning
// -------------------------
float kp = 26.0f;
float kd = 12.0f;
const int lineThreshold = 650;

// Base speed is chosen automatically from curve sharpness
const int baseSpeedStraight = 255;
const int baseSpeedGentle = 220;
const int baseSpeedMedium = 185;
const int baseSpeedSharp = 135;

// -------------------------
// Finish line detection
// -------------------------
const unsigned long finishConfirmMs = 65;

// -------------------------
// Finish action
// -------------------------
const int finishBackwardPulses = 40;

// -------------------------
// Obstacle detection
// -------------------------
const int obstacleDistanceCm = 15;

// -------------------------
// Obstacle avoidance pulses
// -------------------------
const int avoidBackwardPulses = 5;
const int turnLeft45Pulses = 9;
const int avoidForwardDiagonalPulses = 18;
const int turnRight90Pulses = 15;
const int searchLineForwardPulses = 60;

// -------------------------
// Start / pickup stage
// -------------------------
const int startFlagClearDistanceCm = 20;
const int approachObjectPulses = 18;
const int forwardAfterPickupPulses = 5;
const int turnLeftToLinePulses = 16;
const int startSearchLineForwardPulses = 45;

const unsigned long gripperActionTimeMs = 700;

// -------------------------
// Motor speeds (PWM 0-255)
// -------------------------
const int forwardSpeedMotor1 = 250;
const int forwardSpeedMotor2 = 255;

const int backwardSpeedMotor1 = 250;
const int backwardSpeedMotor2 = 255;

const int turnLeftSpeed = 255;
const int turnRightSpeed = 250;

const int searchSpeedMotor1 = 220;
const int searchSpeedMotor2 = 225;

const int pickupSpeedMotor1 = 200;
const int pickupSpeedMotor2 = 205;

// ==================================================
// End of Calibration Zone
// ==================================================

// -------------------------
// NeoPixel object
// -------------------------
Adafruit_NeoPixel strip(numLeds, ledPin, NEO_RGB + NEO_KHZ800);

// -------------------------
// LED blink variables
// -------------------------
unsigned long previousBlinkTime = 0;
bool blinkState = false;

// -------------------------
// Line following variables
// -------------------------
float lastError = 0.0f;
unsigned long finishTimer = 0;
bool finishDetected = false;

// -------------------------
// Pulse tracking
// -------------------------
int pulseCount1 = 0;
int pulseCount2 = 0;

bool lastStateRotation1 = LOW;
bool lastStateRotation2 = LOW;

// -------------------------
// Button / sequence control
// -------------------------
bool sequenceStarted = false;

// -------------------------
// Servo timing helper
// -------------------------
unsigned long gripperStateStartTime = 0;

// ==================================================
// Robot State Machine
// ==================================================
enum RobotState {
  IDLE,

  // Start / pickup stage
  WAIT_FOR_FLAG,
  START_OPEN_GRIPPER,
  OPEN_GRIPPER_WAIT,
  START_APPROACH_OBJECT,
  APPROACH_OBJECT,
  START_CLOSE_GRIPPER,
  CLOSE_GRIPPER_WAIT,
  START_FORWARD_AFTER_PICKUP,
  FORWARD_AFTER_PICKUP,
  START_TURN_LEFT_TO_LINE,
  TURN_LEFT_TO_LINE,
  START_SEARCH_FIRST_LINE,
  SEARCH_FIRST_LINE,

  // Main stage
  LINE_FOLLOWING,

  START_AVOID_BACKWARD,
  AVOID_BACKWARD,

  START_AVOID_TURN_LEFT_45,
  AVOID_TURN_LEFT_45,

  START_AVOID_FORWARD_DIAGONAL,
  AVOID_FORWARD_DIAGONAL,

  START_AVOID_TURN_RIGHT_90,
  AVOID_TURN_RIGHT_90,

  START_AVOID_SEARCH_LINE,
  AVOID_SEARCH_LINE,

  // Finish stage
  START_FINISH_OPEN_GRIPPER,
  FINISH_OPEN_GRIPPER_WAIT,
  START_FINISH_BACKWARD,
  FINISH_BACKWARD,

  DONE
};

RobotState state = IDLE;

// ==================================================
// Setup
// ==================================================
void setup() {
  // Motor pins
  pinMode(motor1BackwardPin, OUTPUT);
  pinMode(motor1ForwardPin, OUTPUT);
  pinMode(motor2ForwardPin, OUTPUT);
  pinMode(motor2BackwardPin, OUTPUT);

  // Rotation sensors
  pinMode(rotationWheel1Pin, INPUT);
  pinMode(rotationWheel2Pin, INPUT);

  // Start button
  pinMode(startButtonPin, INPUT_PULLUP);

  // Ultrasonic
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Servo
  pinMode(servoPin, OUTPUT);
  digitalWrite(servoPin, LOW);

  Serial.begin(9600);

  // LEDs
  strip.begin();
  strip.show();

  stopMotors();
  turnAllLedsOff();
}

// ==================================================
// Main Loop
// ==================================================
void loop() {
  updateWheelPulses();

  int forwardState = digitalRead(startButtonPin);

  // Start only once when button is pressed
  if (state == IDLE && forwardState == LOW && !sequenceStarted) {
    sequenceStarted = true;
    state = WAIT_FOR_FLAG;
    Serial.println("Button pressed -> Waiting for raised flag");
  }

  // Allow restart after DONE and button released
  if (state == DONE && forwardState == HIGH) {
    sequenceStarted = false;
    state = IDLE;
    Serial.println("Ready for next button press");
  }

  switch (state) {
    case IDLE:
      stopMotors();
      turnAllLedsOff();
      break;

    // ==================================================
    // Start / Pickup Stage
    // ==================================================

    case WAIT_FOR_FLAG: {
      stopMotors();
      turnAllLedsOff();

      int distance = readDistanceCm();

      Serial.print("Flag distance: ");
      Serial.print(distance);
      Serial.println(" cm");

      if (distance > startFlagClearDistanceCm) {
        Serial.println("Flag raised! Opening gripper...");
        state = START_OPEN_GRIPPER;
      }

      delay(100);
      break;
    }

    case START_OPEN_GRIPPER:
      gripperStateStartTime = millis();
      Serial.println("START_OPEN_GRIPPER");
      state = OPEN_GRIPPER_WAIT;
      break;

    case OPEN_GRIPPER_WAIT:
      updateGripperSignal(gripperOpenPulse);
      if (millis() - gripperStateStartTime >= gripperActionTimeMs) {
        Serial.println("Gripper opened");
        state = START_APPROACH_OBJECT;
      }
      break;

    case START_APPROACH_OBJECT:
      resetPulseCounts();
      Serial.println("START_APPROACH_OBJECT");
      state = APPROACH_OBJECT;
      break;

    case APPROACH_OBJECT:
      startMotorsForwardPickup();
      turnAllLedsOff();

      if (pulseCount1 >= approachObjectPulses && pulseCount2 >= approachObjectPulses) {
        stopMotors();
        Serial.println("Reached object");
        state = START_CLOSE_GRIPPER;
      }
      break;

    case START_CLOSE_GRIPPER:
      gripperStateStartTime = millis();
      Serial.println("START_CLOSE_GRIPPER");
      state = CLOSE_GRIPPER_WAIT;
      break;

    case CLOSE_GRIPPER_WAIT:
      updateGripperSignal(gripperClosePulse);
      if (millis() - gripperStateStartTime >= gripperActionTimeMs) {
        Serial.println("Gripper closed");
        state = START_FORWARD_AFTER_PICKUP;
      }
      break;

    case START_FORWARD_AFTER_PICKUP:
      resetPulseCounts();
      Serial.println("START_FORWARD_AFTER_PICKUP");
      state = FORWARD_AFTER_PICKUP;
      break;

    case FORWARD_AFTER_PICKUP:
      startMotorsForwardPickup();
      turnAllLedsOff();

      if (pulseCount1 >= forwardAfterPickupPulses && pulseCount2 >= forwardAfterPickupPulses) {
        stopMotors();
        Serial.println("FORWARD_AFTER_PICKUP complete");
        state = START_TURN_LEFT_TO_LINE;
      }
      break;

    case START_TURN_LEFT_TO_LINE:
      resetPulseCounts();
      Serial.println("START_TURN_LEFT_TO_LINE");
      state = TURN_LEFT_TO_LINE;
      break;

    case TURN_LEFT_TO_LINE:
      startMotorLeft();
      showLeftBlink();

      // LEFT turn uses pulseCount1 on your robot
      if (pulseCount1 >= turnLeftToLinePulses) {
        stopMotors();
        Serial.println("TURN_LEFT_TO_LINE complete");
        state = START_SEARCH_FIRST_LINE;
      }
      break;

    case START_SEARCH_FIRST_LINE:
      resetPulseCounts();
      Serial.println("START_SEARCH_FIRST_LINE");
      state = SEARCH_FIRST_LINE;
      break;

    case SEARCH_FIRST_LINE: {
      startMotorsForwardSearch();
      turnAllLedsOff();

      long totalSum = readLineSensorCount();

      if (totalSum > 0) {
        stopMotors();
        Serial.println("First line found! Entering LINE_FOLLOWING");

        lastError = 0.0f;
        finishDetected = false;
        state = LINE_FOLLOWING;
      } else if (pulseCount1 >= startSearchLineForwardPulses &&
                 pulseCount2 >= startSearchLineForwardPulses) {
        stopMotors();
        Serial.println("Could not find first line");
        state = DONE;
      }

      break;
    }

    // ==================================================
    // Main Line Following
    // ==================================================

    case LINE_FOLLOWING: {
      long weightedTotalSum = 0;
      long totalSum = 0;
      readLinePosition(weightedTotalSum, totalSum);

      // Finish line detection
      if (totalSum == 8) {
        if (!finishDetected) {
          finishDetected = true;
          finishTimer = millis();
        }

        if (millis() - finishTimer > finishConfirmMs) {
          stopMotors();
          turnAllLedsOff();

          Serial.println("Finish Line Detected -> Starting finish sequence");
          state = START_FINISH_OPEN_GRIPPER;
          break;
        }
      } else {
        finishDetected = false;
      }

      // Obstacle detection
      int distance = readDistanceCm();

      Serial.print("Distance: ");
      Serial.print(distance);
      Serial.println(" cm");

      if (distance > 0 && distance <= obstacleDistanceCm) {
        stopMotors();
        turnAllLedsOff();

        Serial.println("Obstacle detected! Starting diagonal avoidance...");
        state = START_AVOID_BACKWARD;
        break;
      }

      // Line-following speed control
      int baseSpeed = 0;
      float position = 0.0f;

      if (totalSum != 0) {
        position = (float)weightedTotalSum / totalSum;
        float absError = abs(position);

        if (absError < 0.5f) {
          baseSpeed = baseSpeedStraight;
        } else if (absError < 2.0f) {
          baseSpeed = baseSpeedGentle;
        } else if (absError < 4.0f) {
          baseSpeed = baseSpeedMedium;
        } else {
          baseSpeed = baseSpeedSharp;
        }
      } else {
        baseSpeed = 0;
      }

      float error = position;
      float correction = kp * error + kd * (error - lastError);
      lastError = error;

      int leftSpeed = baseSpeed - correction;
      int rightSpeed = baseSpeed + correction;

      leftSpeed = constrain(leftSpeed, 0, 255);
      rightSpeed = constrain(rightSpeed, 0, 255);

      setForwardSpeeds(leftSpeed, rightSpeed);

      // LED behavior based on steering
      if (leftSpeed == 0 && rightSpeed == 0) {
        turnAllLedsOff();
      } else if (rightSpeed > leftSpeed + 20) {
        showLeftBlink();
      } else if (leftSpeed > rightSpeed + 20) {
        showRightBlink();
      } else {
        turnAllLedsOff();
      }

      Serial.print("Error: ");
      Serial.println(error);

      break;
    }

    // ==================================================
    // Obstacle Avoidance
    // back -> left 45 -> forward diagonal -> right 90 -> search line
    // ==================================================

    case START_AVOID_BACKWARD:
      resetPulseCounts();
      Serial.println("START_AVOID_BACKWARD");
      state = AVOID_BACKWARD;
      break;

    case AVOID_BACKWARD:
      startMotorsBackward();
      showBackRed();
      if (pulseCount1 >= avoidBackwardPulses && pulseCount2 >= avoidBackwardPulses) {
        stopMotors();
        Serial.println("AVOID_BACKWARD complete");
        state = START_AVOID_TURN_LEFT_45;
      }
      break;

    case START_AVOID_TURN_LEFT_45:
      resetPulseCounts();
      Serial.println("START_AVOID_TURN_LEFT_45");
      state = AVOID_TURN_LEFT_45;
      break;

    case AVOID_TURN_LEFT_45:
      startMotorLeft();
      showLeftBlink();
      if (pulseCount1 >= turnLeft45Pulses) {
        stopMotors();
        Serial.println("AVOID_TURN_LEFT_45 complete");
        state = START_AVOID_FORWARD_DIAGONAL;
      }
      break;

    case START_AVOID_FORWARD_DIAGONAL:
      resetPulseCounts();
      Serial.println("START_AVOID_FORWARD_DIAGONAL");
      state = AVOID_FORWARD_DIAGONAL;
      break;

    case AVOID_FORWARD_DIAGONAL:
      startMotorsForward();
      turnAllLedsOff();
      if (pulseCount1 >= avoidForwardDiagonalPulses &&
          pulseCount2 >= avoidForwardDiagonalPulses) {
        stopMotors();
        Serial.println("AVOID_FORWARD_DIAGONAL complete");
        state = START_AVOID_TURN_RIGHT_90;
      }
      break;

    case START_AVOID_TURN_RIGHT_90:
      resetPulseCounts();
      Serial.println("START_AVOID_TURN_RIGHT_90");
      state = AVOID_TURN_RIGHT_90;
      break;

    case AVOID_TURN_RIGHT_90:
      startMotorRight();
      showRightBlink();
      if (pulseCount2 >= turnRight90Pulses) {
        stopMotors();
        Serial.println("AVOID_TURN_RIGHT_90 complete");
        state = START_AVOID_SEARCH_LINE;
      }
      break;

    case START_AVOID_SEARCH_LINE:
      resetPulseCounts();
      Serial.println("START_AVOID_SEARCH_LINE");
      state = AVOID_SEARCH_LINE;
      break;

    case AVOID_SEARCH_LINE: {
      startMotorsForwardSearch();
      turnAllLedsOff();

      long totalSum = readLineSensorCount();

      if (totalSum > 0) {
        stopMotors();
        Serial.println("Line found again! Returning to LINE_FOLLOWING");

        lastError = 0.0f;
        finishDetected = false;
        state = LINE_FOLLOWING;
      } else if (pulseCount1 >= searchLineForwardPulses &&
                 pulseCount2 >= searchLineForwardPulses) {
        stopMotors();
        Serial.println("Search distance limit reached - line not found");
        state = DONE;
      }

      break;
    }

    // ==================================================
    // Finish Stage
    // ==================================================

    case START_FINISH_OPEN_GRIPPER:
      gripperStateStartTime = millis();
      Serial.println("START_FINISH_OPEN_GRIPPER");
      state = FINISH_OPEN_GRIPPER_WAIT;
      break;

    case FINISH_OPEN_GRIPPER_WAIT:
      updateGripperSignal(gripperOpenPulse);
      if (millis() - gripperStateStartTime >= gripperActionTimeMs) {
        Serial.println("Finish gripper opened");
        state = START_FINISH_BACKWARD;
      }
      break;

    case START_FINISH_BACKWARD:
      resetPulseCounts();
      Serial.println("START_FINISH_BACKWARD");
      state = FINISH_BACKWARD;
      break;

    case FINISH_BACKWARD:
      startMotorsBackward();
      showBackRed();

      if (pulseCount1 >= finishBackwardPulses && pulseCount2 >= finishBackwardPulses) {
        stopMotors();
        turnAllLedsOff();
        Serial.println("FINISH_BACKWARD complete");
        state = DONE;
      }
      break;

    case DONE:
      stopMotors();
      turnAllLedsOff();
      break;
  }
}

// ==================================================
// Sensor Functions
// ==================================================
int readDistanceCm() {
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);

  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);

  if (duration == 0) {
    return -1;
  }

  int distance = duration * 0.034 / 2;
  return distance;
}

int readLineSensorCount() {
  long totalSum = 0;

  for (int i = 0; i < 8; i++) {
    int sensorRead = analogRead(sensorPins[i]);
    int adcValue = sensorRead - lineThreshold;

    if (adcValue > 0) {
      adcValue = 1;
    } else {
      adcValue = 0;
    }

    totalSum += adcValue;
  }

  return totalSum;
}

void readLinePosition(long &weightedTotalSum, long &totalSum) {
  weightedTotalSum = 0;
  totalSum = 0;

  for (int i = 0; i < 8; i++) {
    int sensorRead = analogRead(sensorPins[i]);
    int adcValue = sensorRead - lineThreshold;

    if (adcValue > 0) {
      adcValue = 1;
    } else {
      adcValue = 0;
    }

    weightedTotalSum += adcValue * sensorWeights[i];
    totalSum += adcValue;
  }
}

// ==================================================
// Pulse Counting
// ==================================================
void updateWheelPulses() {
  bool currentStateRotation1 = digitalRead(rotationWheel1Pin);
  bool currentStateRotation2 = digitalRead(rotationWheel2Pin);

  // Rising edge detection for wheel 1
  if (currentStateRotation1 == HIGH && lastStateRotation1 == LOW) {
    pulseCount1++;
    Serial.print("Pulse 1: ");
    Serial.println(pulseCount1);
  }

  // Rising edge detection for wheel 2
  if (currentStateRotation2 == HIGH && lastStateRotation2 == LOW) {
    pulseCount2++;
    Serial.print("Pulse 2: ");
    Serial.println(pulseCount2);
  }

  lastStateRotation1 = currentStateRotation1;
  lastStateRotation2 = currentStateRotation2;
}

// ==================================================
// Utility Functions
// ==================================================
void resetPulseCounts() {
  pulseCount1 = 0;
  pulseCount2 = 0;
}

void stopMotors() {
  digitalWrite(motor1BackwardPin, LOW);
  digitalWrite(motor1ForwardPin, LOW);
  digitalWrite(motor2ForwardPin, LOW);
  digitalWrite(motor2BackwardPin, LOW);
}

// ==================================================
// Motor Functions
// ==================================================
void setForwardSpeeds(int leftSpeed, int rightSpeed) {
  // Left motor = MOTOR1
  digitalWrite(motor1BackwardPin, LOW);
  analogWrite(motor1ForwardPin, leftSpeed);

  // Right motor = MOTOR2
  analogWrite(motor2ForwardPin, rightSpeed);
  digitalWrite(motor2BackwardPin, LOW);
}

void startMotorsForward() {
  digitalWrite(motor1BackwardPin, LOW);
  analogWrite(motor1ForwardPin, forwardSpeedMotor1);

  analogWrite(motor2ForwardPin, forwardSpeedMotor2);
  digitalWrite(motor2BackwardPin, LOW);
}

void startMotorsForwardPickup() {
  digitalWrite(motor1BackwardPin, LOW);
  analogWrite(motor1ForwardPin, pickupSpeedMotor1);

  analogWrite(motor2ForwardPin, pickupSpeedMotor2);
  digitalWrite(motor2BackwardPin, LOW);
}

void startMotorsForwardSearch() {
  digitalWrite(motor1BackwardPin, LOW);
  analogWrite(motor1ForwardPin, searchSpeedMotor1);

  analogWrite(motor2ForwardPin, searchSpeedMotor2);
  digitalWrite(motor2BackwardPin, LOW);
}

void startMotorsBackward() {
  analogWrite(motor1BackwardPin, backwardSpeedMotor1);
  digitalWrite(motor1ForwardPin, LOW);

  digitalWrite(motor2ForwardPin, LOW);
  analogWrite(motor2BackwardPin, backwardSpeedMotor2);
}

// LEFT turn: only motor 2 moves
// Your real robot: LEFT turn checks pulseCount1
void startMotorLeft() {
  digitalWrite(motor1BackwardPin, LOW);
  digitalWrite(motor1ForwardPin, LOW);

  analogWrite(motor2ForwardPin, turnLeftSpeed);
  digitalWrite(motor2BackwardPin, LOW);
}

// RIGHT turn: only motor 1 moves
// Your real robot: RIGHT turn checks pulseCount2
void startMotorRight() {
  digitalWrite(motor1BackwardPin, LOW);
  analogWrite(motor1ForwardPin, turnRightSpeed);

  digitalWrite(motor2ForwardPin, LOW);
  digitalWrite(motor2BackwardPin, LOW);
}

// ==================================================
// Servo Function
// ==================================================
void updateGripperSignal(int newPulse) {
  static unsigned long timer = 0;
  static int pulse = gripperOpenPulse;

  if (millis() > timer) {
    if (newPulse > 0) {
      pulse = newPulse;
    }

    digitalWrite(servoPin, HIGH);
    delayMicroseconds(pulse);
    digitalWrite(servoPin, LOW);

    // Servo signal repeats every 20 ms
    timer = millis() + 20;
  }
}

// ==================================================
// LED Functions
// ==================================================
void turnAllLedsOff() {
  for (int i = 0; i < numLeds; i++) {
    strip.setPixelColor(i, strip.Color(0, 0, 0));
  }

  strip.show();
}

void showLeftBlink() {
  unsigned long currentTime = millis();

  if (currentTime - previousBlinkTime >= blinkIntervalMs) {
    previousBlinkTime = currentTime;
    blinkState = !blinkState;
  }

  if (blinkState) {
    // Left side ON
    strip.setPixelColor(0, strip.Color(255, 165, 0));  // Left Back
    strip.setPixelColor(3, strip.Color(255, 165, 0));  // Left Wing / Front Left

    // Right side OFF
    strip.setPixelColor(1, strip.Color(0, 0, 0));      // Right Back
    strip.setPixelColor(2, strip.Color(0, 0, 0));      // Right Wing / Front Right
  } else {
    // Left side OFF
    strip.setPixelColor(0, strip.Color(0, 0, 0));
    strip.setPixelColor(3, strip.Color(0, 0, 0));

    // Keep right OFF too
    strip.setPixelColor(1, strip.Color(0, 0, 0));
    strip.setPixelColor(2, strip.Color(0, 0, 0));
  }

  strip.show();
}

void showRightBlink() {
  unsigned long currentTime = millis();

  if (currentTime - previousBlinkTime >= blinkIntervalMs) {
    previousBlinkTime = currentTime;
    blinkState = !blinkState;
  }

  if (blinkState) {
    // Right side ON
    strip.setPixelColor(1, strip.Color(255, 165, 0));  // Right Back
    strip.setPixelColor(2, strip.Color(255, 165, 0));  // Right Wing / Front Right

    // Left side OFF
    strip.setPixelColor(0, strip.Color(0, 0, 0));      // Left Back
    strip.setPixelColor(3, strip.Color(0, 0, 0));      // Left Wing / Front Left
  } else {
    // Right side OFF
    strip.setPixelColor(1, strip.Color(0, 0, 0));
    strip.setPixelColor(2, strip.Color(0, 0, 0));

    // Keep left OFF too
    strip.setPixelColor(0, strip.Color(0, 0, 0));
    strip.setPixelColor(3, strip.Color(0, 0, 0));
  }

  strip.show();
}

void showBackRed() {
  // Based on your current physical mapping:
  // 0 = Left Back
  // 1 = Right Back
  // 2 = Right Wing / Front Right
  // 3 = Left Wing / Front Left

  // Back LEDs red
  strip.setPixelColor(0, strip.Color(255, 0, 0));  // Left Back
  strip.setPixelColor(1, strip.Color(255, 0, 0));  // Right Back

  // Front LEDs OFF during reverse
  strip.setPixelColor(2, strip.Color(0, 0, 0));    // Front Right
  strip.setPixelColor(3, strip.Color(0, 0, 0));    // Front Left

  strip.show();
}
