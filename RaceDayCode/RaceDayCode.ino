#include <Adafruit_NeoPixel.h>

// ==================================================
// =============== CALIBRATION ZONE =================
// ==================================================

// =========================
// LED settings
// =========================
#define LED_PIN 8
#define NUM_LEDS 4

const int BLINK_INTERVAL = 250;   // turn signal blink speed (ms)

// IMPORTANT LED ORDER (based on your current working mapping):
// 0 = Left Back
// 1 = Right Back
// 2 = Right Wing / Front Right
// 3 = Left Wing / Front Left

// =========================
// Motor pins
// =========================
const int MOTOR1BACKWARD = 11;
const int MOTOR1FORWARD  = 10;
const int MOTOR2FORWARD  = 9;
const int MOTOR2BACKWARD = 6;

// =========================
// Wheel encoder / rotation sensors
// =========================
const int ROTATIONWHEEL1 = 2; // Sensor 1
const int ROTATIONWHEEL2 = 3; // Sensor 2

// =========================
// Start button
// =========================
const int BUTTONFORWARD = 7;

// =========================
// Ultrasonic sensor pins
// =========================
const int TRIGGERPIN = 4;
const int ECHOPIN    = 13;

// =========================
// Servo / gripper
// =========================
#define SERVO_PIN       12
#define GRIPPER_OPEN    1600
#define GRIPPER_CLOSE   950

// =========================
// Line sensor pins
// =========================
const int sensorPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};
int weights[8] = {-7, -5, -3, -1, 1, 3, 5, 7};

// =========================
// Line following tuning
// =========================
float Kp = 20.0;
float Kd = 12.0;
const int threshold = 650;

// Base speed is chosen automatically from curve sharpness,
// but you can still adjust the ranges below if needed.
const int BASE_SPEED_STRAIGHT = 255;
const int BASE_SPEED_GENTLE   = 220;
const int BASE_SPEED_MEDIUM   = 190;
const int BASE_SPEED_SHARP    = 145;

// =========================
// Finish line detection
// =========================
const int FINISH_CONFIRM_MS = 75; // all 8 sensors on line for this long

// =========================
// FINISH ACTION (NEW)
// =========================
const int FINISH_BACKWARD_PULSES = 40;   // TUNE THIS yourself

// =========================
// Obstacle detection
// =========================
const int OBSTACLE_DISTANCE_CM = 15;  // detect obstacle if <= this distance

// =========================
// Obstacle avoidance pulses
// =========================
const int AVOID_BACKWARD_PULSES          = 5;   // small reverse before detour
const int TURN_LEFT_45_PULSES            = 9;   // ~45 degree left turn (tune this)
const int AVOID_FORWARD_DIAGONAL_PULSES  = 20;  // forward after LEFT turn (tune this)
const int TURN_RIGHT_90_PULSES           = 18;  // ~90 degree right turn (tune this)
const int SEARCH_LINE_FORWARD_PULSES     = 60;  // tune this separately

// =========================
// START / PICKUP STAGE (NEW)
// =========================
const int START_FLAG_CLEAR_DISTANCE_CM      = 20;  // if distance > this, flag is considered raised
const int APPROACH_OBJECT_PULSES            = 18;  // how far to drive forward to reach object
const int FORWARD_AFTER_PICKUP_PULSES       = 8;   // move forward after grabbing object (important!)
const int TURN_LEFT_TO_LINE_PULSES          = 16;  // left turn to aim toward line
const int START_SEARCH_LINE_FORWARD_PULSES  = 45;  // forward while searching for first line

// How long to keep gripper command active before moving on
const unsigned long GRIPPER_ACTION_TIME_MS = 700;

// =========================
// Motor speeds (PWM 0-255)
// =========================

// Forward speeds (used for line following base + avoidance forward)
const int FORWARD_SPEED_MOTOR1 = 250;
const int FORWARD_SPEED_MOTOR2 = 255;

// Backward speeds
const int BACKWARD_SPEED_MOTOR1 = 250;
const int BACKWARD_SPEED_MOTOR2 = 255;

// Turn speeds
// LEFT turn = only motor 2 moves
const int TURN_LEFT_SPEED = 255;

// RIGHT turn = only motor 1 moves
const int TURN_RIGHT_SPEED = 250;

// Search line forward speed (slightly safer while re-entering)
const int SEARCH_SPEED_MOTOR1 = 220;
const int SEARCH_SPEED_MOTOR2 = 225;

// Slightly safer approach speed for pickup stage
const int PICKUP_SPEED_MOTOR1 = 200;
const int PICKUP_SPEED_MOTOR2 = 205;

// ==================================================
// ============ END OF CALIBRATION ZONE =============
// ==================================================


// =========================
// NeoPixel object
// =========================
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// =========================
// LED blink variables
// =========================
unsigned long previousBlinkTime = 0;
bool blinkState = false;

// =========================
// Line following variables
// =========================
float lastError = 0;
unsigned long finishTimer = 0;
bool finishDetected = false;

// =========================
// Pulse tracking
// =========================
int pulseCount1 = 0;
int pulseCount2 = 0;

bool lastStateRotation1 = LOW;
bool lastStateRotation2 = LOW;

// =========================
// Button / sequence control
// =========================
bool sequenceStarted = false;

// =========================
// Servo timing helpers
// =========================
unsigned long gripperStateStartTime = 0;

// =========================
// Robot state machine
// =========================
enum RobotState {
  IDLE,

  // NEW START / PICKUP STAGE
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

  // MAIN STAGE
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

  // FINISH STAGE (NEW)
  START_FINISH_OPEN_GRIPPER,
  FINISH_OPEN_GRIPPER_WAIT,
  START_FINISH_BACKWARD,
  FINISH_BACKWARD,

  DONE
};

RobotState state = IDLE;

// ==================================================
// ===================== SETUP =======================
// ==================================================
void setup()
{
  // Motor pins
  pinMode(MOTOR1BACKWARD, OUTPUT);
  pinMode(MOTOR1FORWARD, OUTPUT);
  pinMode(MOTOR2FORWARD, OUTPUT);
  pinMode(MOTOR2BACKWARD, OUTPUT);

  // Rotation sensors
  pinMode(ROTATIONWHEEL1, INPUT);
  pinMode(ROTATIONWHEEL2, INPUT);

  // Stable button input
  pinMode(BUTTONFORWARD, INPUT_PULLUP);

  // Ultrasonic
  pinMode(TRIGGERPIN, OUTPUT);
  pinMode(ECHOPIN, INPUT);

  // Servo
  pinMode(SERVO_PIN, OUTPUT);
  digitalWrite(SERVO_PIN, LOW);

  Serial.begin(9600);

  // LEDs
  strip.begin();
  strip.show();

  stopMotors();
  turnAllLedsOff();
}

// ==================================================
// ====================== LOOP =======================
// ==================================================
void loop()
{
  updateWheelPulses();

  int forwardState = digitalRead(BUTTONFORWARD);

  // Start only once when button pressed
  if (state == IDLE && forwardState == LOW && !sequenceStarted)
  {
    sequenceStarted = true;
    state = WAIT_FOR_FLAG;
    Serial.println("Button pressed -> Waiting for raised flag");
  }

  // Allow restart after DONE and button released
  if (state == DONE && forwardState == HIGH)
  {
    sequenceStarted = false;
    state = IDLE;
    Serial.println("Ready for next button press");
  }

  switch (state)
  {
    case IDLE:
      stopMotors();
      turnAllLedsOff();
      break;

    // ==================================================
    // =============== START / PICKUP STAGE =============
    // ==================================================

    case WAIT_FOR_FLAG:
    {
      stopMotors();
      turnAllLedsOff();

      int distance = readDistanceCM();

      Serial.print("Flag distance: ");
      Serial.print(distance);
      Serial.println(" cm");

      // Same idea as teammate: if distance > 20, flag is raised / path is clear
      if (distance > START_FLAG_CLEAR_DISTANCE_CM)
      {
        Serial.println("Flag raised! Opening gripper...");
        state = START_OPEN_GRIPPER;
      }

      delay(100); // same spirit as teammate code
      break;
    }

    case START_OPEN_GRIPPER:
      gripperStateStartTime = millis();
      Serial.println("START_OPEN_GRIPPER");
      state = OPEN_GRIPPER_WAIT;
      break;

    case OPEN_GRIPPER_WAIT:
      closeGripper(GRIPPER_OPEN);
      if (millis() - gripperStateStartTime >= GRIPPER_ACTION_TIME_MS)
      {
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

      if (pulseCount1 >= APPROACH_OBJECT_PULSES && pulseCount2 >= APPROACH_OBJECT_PULSES)
      {
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
      closeGripper(GRIPPER_CLOSE);
      if (millis() - gripperStateStartTime >= GRIPPER_ACTION_TIME_MS)
      {
        Serial.println("Gripper closed");
        state = START_FORWARD_AFTER_PICKUP;
      }
      break;

    // IMPORTANT extra step you remembered
    case START_FORWARD_AFTER_PICKUP:
      resetPulseCounts();
      Serial.println("START_FORWARD_AFTER_PICKUP");
      state = FORWARD_AFTER_PICKUP;
      break;

    case FORWARD_AFTER_PICKUP:
      startMotorsForwardPickup();
      turnAllLedsOff();

      if (pulseCount1 >= FORWARD_AFTER_PICKUP_PULSES && pulseCount2 >= FORWARD_AFTER_PICKUP_PULSES)
      {
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

      // LEFT turn uses pulseCount1 on your robot (same convention as your code)
      if (pulseCount1 >= TURN_LEFT_TO_LINE_PULSES)
      {
        stopMotors();
        Serial.println("TURN_LEFT_TO_LINE complete");
        state = START_SEARCH_FIRST_LINE;
      }
      break;

    // Move forward while searching for the first line
    case START_SEARCH_FIRST_LINE:
      resetPulseCounts();
      Serial.println("START_SEARCH_FIRST_LINE");
      state = SEARCH_FIRST_LINE;
      break;

    case SEARCH_FIRST_LINE:
    {
      startMotorsForwardSearch();
      turnAllLedsOff();

      long totalSum = 0;

      for (int i = 0; i < 8; i++)
      {
        int sensorRead = analogRead(sensorPins[i]);

        int ADCvalue = sensorRead - threshold;

        if (ADCvalue > 0)
          ADCvalue = 1;
        else
          ADCvalue = 0;

        totalSum += ADCvalue;
      }

      // If any sensor sees the line, start your normal line following
      if (totalSum > 0)
      {
        stopMotors();
        Serial.println("First line found! Entering LINE_FOLLOWING");

        lastError = 0;
        finishDetected = false;
        state = LINE_FOLLOWING;
      }
      else if (pulseCount1 >= START_SEARCH_LINE_FORWARD_PULSES &&
               pulseCount2 >= START_SEARCH_LINE_FORWARD_PULSES)
      {
        stopMotors();
        Serial.println("Could not find first line");
        state = DONE;
      }

      break;
    }

    // ==================================================
    // ================ NORMAL LINE FOLLOWING ============
    // ==================================================
    case LINE_FOLLOWING:
    {
      // --------- 1) Read line sensors ----------
      long weightedTotalSum = 0;
      long totalSum = 0;

      for (int i = 0; i < 8; i++)
      {
        int sensorRead = analogRead(sensorPins[i]);

        int ADCvalue = sensorRead - threshold;

        if (ADCvalue > 0)
          ADCvalue = 1;
        else
          ADCvalue = 0;

        weightedTotalSum += ADCvalue * weights[i];
        totalSum += ADCvalue;
      }

      // --------- 2) Finish line detection ----------
      if (totalSum == 8)
      {
        if (!finishDetected)
        {
          finishDetected = true;
          finishTimer = millis();
        }

        if (millis() - finishTimer > FINISH_CONFIRM_MS)
        {
          stopMotors();
          turnAllLedsOff();

          Serial.println("Finish Line Detected -> Starting finish sequence");
          state = START_FINISH_OPEN_GRIPPER;
          break;
        }
      }
      else
      {
        finishDetected = false;
      }

      // --------- 3) Obstacle detection ----------
      int distance = readDistanceCM();

      Serial.print("Distance: ");
      Serial.print(distance);
      Serial.println(" cm");

      if (distance > 0 && distance <= OBSTACLE_DISTANCE_CM)
      {
        stopMotors();
        turnAllLedsOff();

        Serial.println("Obstacle detected! Starting diagonal avoidance...");
        state = START_AVOID_BACKWARD;
        break;
      }

      // --------- 4) Line-following speed control ----------
      int baseSpeed = 0;
      float position = 0;

      if (totalSum != 0)
      {
        position = (float)weightedTotalSum / totalSum;
        float absError = abs(position);

        if (absError < 0.5)
        {
          baseSpeed = BASE_SPEED_STRAIGHT;
        }
        else if (absError < 2)
        {
          baseSpeed = BASE_SPEED_GENTLE;
        }
        else if (absError < 4)
        {
          baseSpeed = BASE_SPEED_MEDIUM;
        }
        else
        {
          baseSpeed = BASE_SPEED_SHARP;
        }
      }
      else
      {
        // If line is lost briefly, stop instead of guessing
        baseSpeed = 0;
      }

      float error = position;
      float correction = Kp * error + Kd * (error - lastError);
      lastError = error;

      int leftSpeed  = baseSpeed - correction;
      int rightSpeed = baseSpeed + correction;

      leftSpeed  = constrain(leftSpeed, 0, 255);
      rightSpeed = constrain(rightSpeed, 0, 255);

      // Left motor = MOTOR1
      // Right motor = MOTOR2
      setForwardSpeeds(leftSpeed, rightSpeed);

      // --------- 5) LED behavior based on steering ----------
      if (leftSpeed == 0 && rightSpeed == 0)
      {
        turnAllLedsOff();
      }
      else if (rightSpeed > leftSpeed + 20)
      {
        // Steering LEFT
        showLeftBlink();
      }
      else if (leftSpeed > rightSpeed + 20)
      {
        // Steering RIGHT
        showRightBlink();
      }
      else
      {
        turnAllLedsOff();
      }

      Serial.print("Error: ");
      Serial.println(error);

      break;
    }

    // ==================================================
    // ============ DIAGONAL MIRROR AVOIDANCE ============
    // back -> left 45 -> forward diagonal -> right 90 -> search line
    // ==================================================

    // ---------- Small backward before detour ----------
    case START_AVOID_BACKWARD:
      resetPulseCounts();
      Serial.println("START_AVOID_BACKWARD");
      state = AVOID_BACKWARD;
      break;

    case AVOID_BACKWARD:
      startMotorsBackward();
      showBackRed();
      if (pulseCount1 >= AVOID_BACKWARD_PULSES && pulseCount2 >= AVOID_BACKWARD_PULSES)
      {
        stopMotors();
        Serial.println("AVOID_BACKWARD complete");
        state = START_AVOID_TURN_LEFT_45;
      }
      break;

    // ---------- TURN LEFT ~45 ----------
    // LEFT turn uses pulseCount1 on your robot
    case START_AVOID_TURN_LEFT_45:
      resetPulseCounts();
      Serial.println("START_AVOID_TURN_LEFT_45");
      state = AVOID_TURN_LEFT_45;
      break;

    case AVOID_TURN_LEFT_45:
      startMotorLeft();
      showLeftBlink();
      if (pulseCount1 >= TURN_LEFT_45_PULSES)
      {
        stopMotors();
        Serial.println("AVOID_TURN_LEFT_45 complete");
        state = START_AVOID_FORWARD_DIAGONAL;
      }
      break;

    // ---------- FORWARD DIAGONAL (after LEFT turn) ----------
    case START_AVOID_FORWARD_DIAGONAL:
      resetPulseCounts();
      Serial.println("START_AVOID_FORWARD_DIAGONAL");
      state = AVOID_FORWARD_DIAGONAL;
      break;

    case AVOID_FORWARD_DIAGONAL:
      startMotorsForward();
      turnAllLedsOff();
      if (pulseCount1 >= AVOID_FORWARD_DIAGONAL_PULSES && pulseCount2 >= AVOID_FORWARD_DIAGONAL_PULSES)
      {
        stopMotors();
        Serial.println("AVOID_FORWARD_DIAGONAL complete");
        state = START_AVOID_TURN_RIGHT_90;
      }
      break;

    // ---------- TURN RIGHT ~90 ----------
    // RIGHT turn uses pulseCount2 on your robot
    case START_AVOID_TURN_RIGHT_90:
      resetPulseCounts();
      Serial.println("START_AVOID_TURN_RIGHT_90");
      state = AVOID_TURN_RIGHT_90;
      break;

    case AVOID_TURN_RIGHT_90:
      startMotorRight();
      showRightBlink();
      if (pulseCount2 >= TURN_RIGHT_90_PULSES)
      {
        stopMotors();
        Serial.println("AVOID_TURN_RIGHT_90 complete");
        state = START_AVOID_SEARCH_LINE;
      }
      break;

    // ---------- FORWARD WHILE SEARCHING FOR LINE (after RIGHT turn) ----------
    case START_AVOID_SEARCH_LINE:
      resetPulseCounts();   // separate pulse counter for return/search distance
      Serial.println("START_AVOID_SEARCH_LINE");
      state = AVOID_SEARCH_LINE;
      break;

    case AVOID_SEARCH_LINE:
    {
      // Move forward while searching for the line
      startMotorsForwardSearch();
      turnAllLedsOff();

      long totalSum = 0;

      for (int i = 0; i < 8; i++)
      {
        int sensorRead = analogRead(sensorPins[i]);

        int ADCvalue = sensorRead - threshold;

        if (ADCvalue > 0)
          ADCvalue = 1;
        else
          ADCvalue = 0;

        totalSum += ADCvalue;
      }

      // 1) If any sensor sees the line again, resume line following immediately
      if (totalSum > 0)
      {
        stopMotors();
        Serial.println("Line found again! Returning to LINE_FOLLOWING");

        lastError = 0;
        finishDetected = false;
        state = LINE_FOLLOWING;
      }
      // 2) If no line found yet, only search up to SEARCH_LINE_FORWARD_PULSES
      else if (pulseCount1 >= SEARCH_LINE_FORWARD_PULSES &&
               pulseCount2 >= SEARCH_LINE_FORWARD_PULSES)
      {
        stopMotors();
        Serial.println("Search distance limit reached - line not found");
        state = DONE;
      }

      break;
    }

    // ==================================================
    // ================= FINISH STAGE ===================
    // ==================================================

    case START_FINISH_OPEN_GRIPPER:
      gripperStateStartTime = millis();
      Serial.println("START_FINISH_OPEN_GRIPPER");
      state = FINISH_OPEN_GRIPPER_WAIT;
      break;

    case FINISH_OPEN_GRIPPER_WAIT:
      closeGripper(GRIPPER_OPEN);
      if (millis() - gripperStateStartTime >= GRIPPER_ACTION_TIME_MS)
      {
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

      if (pulseCount1 >= FINISH_BACKWARD_PULSES && pulseCount2 >= FINISH_BACKWARD_PULSES)
      {
        stopMotors();
        turnAllLedsOff();
        Serial.println("FINISH_BACKWARD complete");
        state = DONE;
      }
      break;

    // ==================================================
    // ====================== DONE =======================
    // ==================================================
    case DONE:
      stopMotors();
      turnAllLedsOff();
      break;
  }
}

// ==================================================
// =============== ULTRASONIC FUNCTION ==============
// ==================================================
int readDistanceCM()
{
  digitalWrite(TRIGGERPIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIGGERPIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGERPIN, LOW);

  long duration = pulseIn(ECHOPIN, HIGH, 30000);

  if (duration == 0)
  {
    return -1;
  }

  int distance = duration * 0.034 / 2;
  return distance;
}

// ==================================================
// ================= PULSE COUNTING =================
// ==================================================
void updateWheelPulses()
{
  bool currentStateRotation1 = digitalRead(ROTATIONWHEEL1);
  bool currentStateRotation2 = digitalRead(ROTATIONWHEEL2);

  // Rising edge detection for wheel 1
  if (currentStateRotation1 == HIGH && lastStateRotation1 == LOW)
  {
    pulseCount1++;
    Serial.print("Pulse 1: ");
    Serial.println(pulseCount1);
  }

  // Rising edge detection for wheel 2
  if (currentStateRotation2 == HIGH && lastStateRotation2 == LOW)
  {
    pulseCount2++;
    Serial.print("Pulse 2: ");
    Serial.println(pulseCount2);
  }

  lastStateRotation1 = currentStateRotation1;
  lastStateRotation2 = currentStateRotation2;
}

// ==================================================
// ==================== UTILITIES ====================
// ==================================================
void resetPulseCounts()
{
  pulseCount1 = 0;
  pulseCount2 = 0;
}

void stopMotors()
{
  digitalWrite(MOTOR1BACKWARD, LOW);
  digitalWrite(MOTOR1FORWARD, LOW);
  digitalWrite(MOTOR2FORWARD, LOW);
  digitalWrite(MOTOR2BACKWARD, LOW);
}

// ==================================================
// ================= MOTOR ACTIONS ===================
// ==================================================

// For line following: set independent left/right forward PWM
// Left motor = MOTOR1
// Right motor = MOTOR2
void setForwardSpeeds(int leftSpeed, int rightSpeed)
{
  digitalWrite(MOTOR1BACKWARD, LOW);
  analogWrite(MOTOR1FORWARD, leftSpeed);

  analogWrite(MOTOR2FORWARD, rightSpeed);
  digitalWrite(MOTOR2BACKWARD, LOW);
}

// Avoidance forward using calibration speeds
void startMotorsForward()
{
  digitalWrite(MOTOR1BACKWARD, LOW);
  analogWrite(MOTOR1FORWARD, FORWARD_SPEED_MOTOR1);

  analogWrite(MOTOR2FORWARD, FORWARD_SPEED_MOTOR2);
  digitalWrite(MOTOR2BACKWARD, LOW);
}

// Slower forward for pickup stage
void startMotorsForwardPickup()
{
  digitalWrite(MOTOR1BACKWARD, LOW);
  analogWrite(MOTOR1FORWARD, PICKUP_SPEED_MOTOR1);

  analogWrite(MOTOR2FORWARD, PICKUP_SPEED_MOTOR2);
  digitalWrite(MOTOR2BACKWARD, LOW);
}

// Safer forward while searching for the line again
void startMotorsForwardSearch()
{
  digitalWrite(MOTOR1BACKWARD, LOW);
  analogWrite(MOTOR1FORWARD, SEARCH_SPEED_MOTOR1);

  analogWrite(MOTOR2FORWARD, SEARCH_SPEED_MOTOR2);
  digitalWrite(MOTOR2BACKWARD, LOW);
}

void startMotorsBackward()
{
  analogWrite(MOTOR1BACKWARD, BACKWARD_SPEED_MOTOR1);
  digitalWrite(MOTOR1FORWARD, LOW);

  digitalWrite(MOTOR2FORWARD, LOW);
  analogWrite(MOTOR2BACKWARD, BACKWARD_SPEED_MOTOR2);
}

// LEFT turn: only motor 2 moves
// Your real robot: LEFT turn checks pulseCount1
void startMotorLeft()
{
  digitalWrite(MOTOR1BACKWARD, LOW);
  digitalWrite(MOTOR1FORWARD, LOW);

  analogWrite(MOTOR2FORWARD, TURN_LEFT_SPEED);
  digitalWrite(MOTOR2BACKWARD, LOW);
}

// RIGHT turn: only motor 1 moves
// Your real robot: RIGHT turn checks pulseCount2
void startMotorRight()
{
  digitalWrite(MOTOR1BACKWARD, LOW);
  analogWrite(MOTOR1FORWARD, TURN_RIGHT_SPEED);

  digitalWrite(MOTOR2FORWARD, LOW);
  digitalWrite(MOTOR2BACKWARD, LOW);
}

// ==================================================
// ================= SERVO FUNCTION ==================
// ==================================================
void closeGripper(int newPulse)
{
  static unsigned long timer = 0;
  static int pulse = GRIPPER_OPEN;

  if (millis() > timer)
  {
    if (newPulse > 0)
    {
      pulse = newPulse;
    }

    digitalWrite(SERVO_PIN, HIGH);
    delayMicroseconds(pulse);
    digitalWrite(SERVO_PIN, LOW);

    timer = millis() + 20;   // servo signal repeats every 20ms
  }
}

// ==================================================
// ================= LED FUNCTIONS ===================
// ==================================================
void turnAllLedsOff()
{
  for (int i = 0; i < NUM_LEDS; i++)
  {
    strip.setPixelColor(i, strip.Color(0, 0, 0));
  }
  strip.show();
}

void showLeftBlink()
{
  unsigned long currentTime = millis();

  if (currentTime - previousBlinkTime >= BLINK_INTERVAL)
  {
    previousBlinkTime = currentTime;
    blinkState = !blinkState;
  }

  if (blinkState)
  {
    // Left side ON
    strip.setPixelColor(0, strip.Color(255, 255, 0)); // Left Back
    strip.setPixelColor(3, strip.Color(255, 255, 0)); // Left Wing / Front Left

    // Right side OFF
    strip.setPixelColor(1, strip.Color(0, 0, 0)); // Right Back
    strip.setPixelColor(2, strip.Color(0, 0, 0)); // Right Wing / Front Right
  }
  else
  {
    // Left side OFF
    strip.setPixelColor(0, strip.Color(0, 0, 0));
    strip.setPixelColor(3, strip.Color(0, 0, 0));

    // Keep right OFF too
    strip.setPixelColor(1, strip.Color(0, 0, 0));
    strip.setPixelColor(2, strip.Color(0, 0, 0));
  }

  strip.show();
}

void showRightBlink()
{
  unsigned long currentTime = millis();

  if (currentTime - previousBlinkTime >= BLINK_INTERVAL)
  {
    previousBlinkTime = currentTime;
    blinkState = !blinkState;
  }

  if (blinkState)
  {
    // Right side ON
    strip.setPixelColor(1, strip.Color(255, 255, 0)); // Right Back
    strip.setPixelColor(2, strip.Color(255, 255, 0)); // Right Wing / Front Right

    // Left side OFF
    strip.setPixelColor(0, strip.Color(0, 0, 0)); // Left Back
    strip.setPixelColor(3, strip.Color(0, 0, 0)); // Left Wing / Front Left
  }
  else
  {
    // Right side OFF
    strip.setPixelColor(1, strip.Color(0, 0, 0));
    strip.setPixelColor(2, strip.Color(0, 0, 0));

    // Keep left OFF too
    strip.setPixelColor(0, strip.Color(0, 0, 0));
    strip.setPixelColor(3, strip.Color(0, 0, 0));
  }

  strip.show();
}

void showBackRed()
{
  // Based on your current physical mapping:
  // 0 = Left Back
  // 1 = Right Back
  // 2 = Right Wing / Front Right
  // 3 = Left Wing / Front Left

  // Back LEDs red
  strip.setPixelColor(0, strip.Color(255, 0, 0)); // Left Back
  strip.setPixelColor(1, strip.Color(255, 0, 0)); // Right Back

  // Front LEDs OFF during reverse
  strip.setPixelColor(2, strip.Color(0, 0, 0));   // Front Right
  strip.setPixelColor(3, strip.Color(0, 0, 0));   // Front Left

  strip.show();
}
