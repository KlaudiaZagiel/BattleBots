<<<<<<< HEAD:Week3/grippertest/grippertest.ino
#define SERVO 12          // Servo control pin (gripper)
#define GRIPPER_OPEN 1600 // pulses needed for the gripper to be open (calculated previously)
#define GRIPPER_CLOSE 500
=======
#define  SERVO          12  //GR stands for gripper
#define  GRIPPER_OPEN   1600 
#define  GRIPPER_CLOSE  950
>>>>>>> fa46b1cd7c97b333bed86011610a399242a0682e:Week4/gripperAssignment.ino
const int MOTOR_1_BACKWARD = 11;
const int MOTOR_1_FORWARD = 10;
const int MOTOR_2_FORWARD = 9;
const int MOTOR_2_BACKWARD = 6;
const int MOTOR_1_PWM = 255;
const int MOTOR_2_PWM = 255;
const int BUTTON = 7;
<<<<<<< HEAD:Week3/grippertest/grippertest.ino
const int ROTATIONWHEEL1 = 3; // rotation sensor.
const int ROTATIONWHEEL2 = 2;
bool lastStateRotation2 = LOW; // this prevents counting the same pulse multiple times
=======
const int ROTATIONWHEEL1 = 3;   //rotation sensor.
const int ROTATIONWHEEL2 = 2;
bool lastStateRotation2 = LOW;  //this prevents counting the same pulse multiple times
>>>>>>> fa46b1cd7c97b333bed86011610a399242a0682e:Week4/gripperAssignment.ino
bool lastStateRotation1 = LOW;
bool buttonPressed = false;
int pulseCount2 = 0;    //counts pulses from rotation sensor 2
int pulseCount1 = 0;   //counts pulses from rotation sensor 1
int stage = 1;        //controls the state machine of the robot

<<<<<<< HEAD:Week3/grippertest/grippertest.ino
void setup()
{
=======

void setup() {

    //Servo//
>>>>>>> fa46b1cd7c97b333bed86011610a399242a0682e:Week4/gripperAssignment.ino
    pinMode(SERVO, OUTPUT);
    digitalWrite(SERVO, 0);

    Serial.begin(9600);

    //Motors//
    pinMode(MOTOR_1_BACKWARD, LOW);
    pinMode(MOTOR_1_FORWARD, LOW);
    pinMode(MOTOR_2_FORWARD, LOW);
    pinMode(MOTOR_2_BACKWARD, LOW);

    //Sensors//
    pinMode(ROTATIONWHEEL1, INPUT);
    pinMode(ROTATIONWHEEL2, INPUT);

    //Button//
    pinMode(BUTTON, INPUT);
}

<<<<<<< HEAD:Week3/grippertest/grippertest.ino
void loop()
{
    pulseCounter();

    if (buttonPressed == false) {
        gripper(GRIPPER_OPEN);
    }

    if (buttonPressed == true && stage != 1) {
        gripper(GRIPPER_CLOSE);
=======
void loop() {

    int weightedTotalSum = 0;
    int totalSum = 0;
    pulseCounter();
     
    if(buttonPressed == false){
      gripper(GRIPPER_OPEN);
    }
    
    if(buttonPressed == true && stage != 1) {
      gripper(GRIPPER_CLOSE);
>>>>>>> fa46b1cd7c97b333bed86011610a399242a0682e:Week4/gripperAssignment.ino
    }

    // Button//
    int buttonState = digitalRead(BUTTON);

    if (buttonState == LOW) {
        buttonPressed = true;
    }

<<<<<<< HEAD:Week3/grippertest/grippertest.ino
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
=======
    //Switch case//
    switch(stage){

    //Start moving//
    case 1:
    if(buttonPressed == true){
        if(pulseCount1 <= 24 && pulseCount2 <= 24) {
            startMotorsForward();
    } else {
        stopMotors();
        pulseCount2 = 0;
        pulseCount1 = 0;
        stage++;
        } 
    }

    break;  

    //Wait 1 second//
    case 2:
    static unsigned long startTime = millis();
    if(millis() - startTime >= 1000) {
      stage++;
    }
    break;
>>>>>>> fa46b1cd7c97b333bed86011610a399242a0682e:Week4/gripperAssignment.ino

    //Move again//
    case 3:
        if (pulseCount1 <= 24 && pulseCount2 <= 24) {
            startMotorsForward();
<<<<<<< HEAD:Week3/grippertest/grippertest.ino
        }else{
            stopMotors();
        }

        break;
=======
    } else {
        stopMotors();
>>>>>>> fa46b1cd7c97b333bed86011610a399242a0682e:Week4/gripperAssignment.ino
    }
}

//SERVO control signal//
void gripper(int newPulse) {                 // this will be our 1600(gripper open)(in this case). int newPulse - represents the pulse in microseconds. int - input parameter
    static unsigned long timer;             //timer stores the next time a pulse is allowed to go
    static int pulse;                      //stores the pulse width in microseconds. it remembers the last servo position

<<<<<<< HEAD:Week3/grippertest/grippertest.ino
    if (millis() > timer) {              //only generate a new servo pulse if 20 ms have passed since the last one.

        if (newPulse > 0) {             //If a valid pulse value is passed, update servo position. If 0 is passed, keep previous pulse width.
            pulse = newPulse;
        }

        //Creating servo signal//
=======
void gripper(int newPulse) {       
    static unsigned long timer;   //stores timing between pulses
    static int pulse;            //stores the pulse duration
    if(millis() > timer) {      //checks if it's time to send a new servo pulse
        if(newPulse > 0) {     //updates pulses if a new value is provided
            pulse = newPulse;
        }
        
>>>>>>> fa46b1cd7c97b333bed86011610a399242a0682e:Week4/gripperAssignment.ino
        digitalWrite(SERVO, 1);
        delayMicroseconds(pulse); //keep it high for the needed pulse duration
        digitalWrite(SERVO, 0);
<<<<<<< HEAD:Week3/grippertest/grippertest.ino
        timer = millis() + 20;       // 20 is in ms, this time is needed for the servo to process before receiving a new signal. Do not allow another pulse until 20 milliseconds have passed
=======
        timer = millis() + 20;   //servo signal repeats every 20ms
>>>>>>> fa46b1cd7c97b333bed86011610a399242a0682e:Week4/gripperAssignment.ino
    }
}

void startMotorsForward() {
    digitalWrite(MOTOR_1_BACKWARD, LOW);
    analogWrite(MOTOR_1_FORWARD, MOTOR_1_PWM);
    analogWrite(MOTOR_2_FORWARD, MOTOR_2_PWM);
    digitalWrite(MOTOR_2_BACKWARD, LOW);
}

void pulseCounter() {
<<<<<<< HEAD:Week3/grippertest/grippertest.ino
    bool currentStateRotation2 = digitalRead(ROTATIONWHEEL2); // reads sensor 2

    if (currentStateRotation2 == HIGH && lastStateRotation2 == LOW) { // pulse detection - by comparing previous state, it prevents multiple pulse counts
        pulseCount2++;
    }

    lastStateRotation2 = currentStateRotation2; //

    bool currentStateRotation1 = digitalRead(ROTATIONWHEEL1);

    if (currentStateRotation1 == HIGH && lastStateRotation1 == LOW) {
        pulseCount1++;
    }

=======

    //Rotation wheel sensor state//
    bool currentStateRotation2 = digitalRead(ROTATIONWHEEL2); //reads sensor 2
        if(currentStateRotation2 == HIGH && lastStateRotation2 == LOW ){ //pulse detection - by comparing previous state, it prevents multiple pulse counts
            pulseCount2++; 
         }
         
    lastStateRotation2 = currentStateRotation2; //

    bool currentStateRotation1 = digitalRead(ROTATIONWHEEL1);
        if(currentStateRotation1 == HIGH && lastStateRotation1 == LOW ){  
             pulseCount1++;
         }
         
>>>>>>> fa46b1cd7c97b333bed86011610a399242a0682e:Week4/gripperAssignment.ino
    lastStateRotation1 = currentStateRotation1;
}

void stopMotors() {
    digitalWrite(MOTOR_1_BACKWARD, LOW);
    digitalWrite(MOTOR_1_FORWARD, LOW);
    digitalWrite(MOTOR_2_FORWARD, LOW);
    digitalWrite(MOTOR_2_BACKWARD, LOW);
}