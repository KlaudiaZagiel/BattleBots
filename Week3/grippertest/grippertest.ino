#define SERVO 12          // Servo control pin (gripper)
#define GRIPPER_OPEN 1600 // pulses needed for the gripper to be open (calculated previously)
#define GRIPPER_CLOSE 500
const int MOTOR_1_BACKWARD = 11;
const int MOTOR_1_FORWARD = 10;
const int MOTOR_2_FORWARD = 9;
const int MOTOR_2_BACKWARD = 6;
const int MOTOR_1_PWM = 255;
const int MOTOR_2_PWM = 255;
const int BUTTON = 7;
const int ROTATIONWHEEL1 = 3; // rotation sensor.
const int ROTATIONWHEEL2 = 2;
bool lastStateRotation2 = LOW; // this prevents counting the same pulse multiple times
bool lastStateRotation1 = LOW;
bool buttonPressed = false;
int pulseCount2 = 0;
int pulseCount1 = 0;
int stage = 1;

void setup()
{
    pinMode(SERVO, OUTPUT);
    digitalWrite(SERVO, 0);

    Serial.begin(9600);
}

void loop()
{
    pulseCounter();

    if (buttonPressed == false) {
        gripper(GRIPPER_OPEN);
    }

    if (buttonPressed == true && stage != 1) {
        gripper(GRIPPER_CLOSE);
    }

    // Button//
    int buttonState = digitalRead(BUTTON);

    if (buttonState == LOW) {
        buttonPressed = true;
    }

    // Switch case//
    switch (stage) {

    case 1:
        if (buttonPressed == true) {

            if (pulseCount1 <= 24 && pulseCount2 <= 24) {
                startMotorsForward();
            }else{
                stopMotors();
                pulseCount2 = 0;
                pulseCount1 = 0;
                stage++;
            }
        }

        break;

    case 2:                                           //1 second delay before moving to the next stage
        static unsigned long startTime = millis();   //timestamp to measure how much time passed

        if (millis() - startTime >= 1000) {
            stage++;
        }

        break;

    case 3:
        if (pulseCount1 <= 24 && pulseCount2 <= 24) {
            startMotorsForward();
        }else{
            stopMotors();
        }

        break;
    }
}

//SERVO control signal//
void gripper(int newPulse) {                 // this will be our 1600(gripper open)(in this case). int newPulse - represents the pulse in microseconds. int - input parameter
    static unsigned long timer;             //timer stores the next time a pulse is allowed to go
    static int pulse;                      //stores the pulse width in microseconds. it remembers the last servo position

    if (millis() > timer) {              //only generate a new servo pulse if 20 ms have passed since the last one.

        if (newPulse > 0) {             //If a valid pulse value is passed, update servo position. If 0 is passed, keep previous pulse width.
            pulse = newPulse;
        }

        //Creating servo signal//
        digitalWrite(SERVO, 1);
        delayMicroseconds(pulse);
        digitalWrite(SERVO, 0);
        timer = millis() + 20;       // 20 is in ms, this time is needed for the servo to process before receiving a new signal. Do not allow another pulse until 20 milliseconds have passed
    }
}

void startMotorsForward() {
    digitalWrite(MOTOR_1_BACKWARD, LOW);
    analogWrite(MOTOR_1_FORWARD, MOTOR_1_PWM);
    analogWrite(MOTOR_2_FORWARD, MOTOR_2_PWM);
    digitalWrite(MOTOR_2_BACKWARD, LOW);
}

void pulseCounter() {
    bool currentStateRotation2 = digitalRead(ROTATIONWHEEL2); // reads sensor 2

    if (currentStateRotation2 == HIGH && lastStateRotation2 == LOW) { // pulse detection - by comparing previous state, it prevents multiple pulse counts
        pulseCount2++;
    }

    lastStateRotation2 = currentStateRotation2; //

    bool currentStateRotation1 = digitalRead(ROTATIONWHEEL1);

    if (currentStateRotation1 == HIGH && lastStateRotation1 == LOW) {
        pulseCount1++;
    }

    lastStateRotation1 = currentStateRotation1;
}

void stopMotors() {
    digitalWrite(MOTOR_1_BACKWARD, LOW);
    digitalWrite(MOTOR_1_FORWARD, LOW);
    digitalWrite(MOTOR_2_FORWARD, LOW);
    digitalWrite(MOTOR_2_BACKWARD, LOW);
}