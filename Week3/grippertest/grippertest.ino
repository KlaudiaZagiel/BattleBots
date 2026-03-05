#define  SERVO          12  
#define  GRIPPER_OPEN   1600 
#define  GRIPPER_CLOSE  500
const int MOTOR_1_BACKWARD = 11;
const int MOTOR_1_FORWARD = 10;
const int MOTOR_2_FORWARD = 9;
const int MOTOR_2_BACKWARD = 6;
const int MOTOR_1_PWM = 255;
const int MOTOR_2_PWM = 255;
const int BUTTON = 7;
const int ROTATIONWHEEL1 = 3;      //rotation sensor 1
const int ROTATIONWHEEL2 = 2;     //rotation sensor 2
bool lastStateRotation2 = LOW;   //this prevents counting the same pulse multiple times
bool lastStateRotation1 = LOW;
bool buttonPressed = false;
int pulseCount2 = 0;       //counts pulses from rotation sensor 2
int pulseCount1 = 0;      //counts pulses from rotation sensor 1
int stage = 1;           //controls the state machine of the robot


void setup() {
    pinMode(SERVO, OUTPUT);
    digitalWrite(SERVO, 0);
    Serial.begin(9600);
}

void loop() {
    pulseCounter();
     
    if(buttonPressed == false){
    gripper(GRIPPER_OPEN);
    }
    if(buttonPressed == true && stage != 1) {
      gripper(GRIPPER_CLOSE);
    }
   
    //Button//
    int buttonState = digitalRead(BUTTON);
    if(buttonState == LOW){
        buttonPressed = true;
    }

    //Switch case//
    switch(stage){

    //Start moving//
    case 1:
    if(buttonPressed == true){
        if(pulseCount1 <= 24 && pulseCount2 <= 24) {
            startMotorsForward();
    }else{
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

    //Move again//
    case 3:
    if(pulseCount1 <= 24 && pulseCount2 <= 24) {
            startMotorsForward();
    }else{
        stopMotors();
    }
    break;
  }
}


void gripper(int newPulse) { 
    static unsigned long timer;   //stores timing between pulses
    static int pulse;            //stores the pulse duration
    if(millis() > timer) {      //checks if it's time to send a new servo pulse
        if(newPulse > 0) {     //updates pulses if a new value is provided
            pulse = newPulse;
        }
        digitalWrite(SERVO, 1);
        delayMicroseconds(pulse);   //keep it high for the needed pulse duration
        digitalWrite(SERVO, 0);
        timer = millis() + 20;     //servo signal repeats every 20ms
    }
}

void startMotorsForward() {
    digitalWrite(MOTOR_1_BACKWARD, LOW);
    analogWrite(MOTOR_1_FORWARD, MOTOR_1_PWM);
    analogWrite(MOTOR_2_FORWARD, MOTOR_2_PWM); 
    digitalWrite(MOTOR_2_BACKWARD, LOW); 
}

void pulseCounter() {

    //Rotation wheel sensor state//
    bool currentStateRotation2 = digitalRead(ROTATIONWHEEL2);                //reads sensor 2
        if(currentStateRotation2 == HIGH && lastStateRotation2 == LOW ){    //pulse detection - by comparing previous state, it prevents multiple pulse counts
            pulseCount2++; 
         }
    lastStateRotation2 = currentStateRotation2; //

    bool currentStateRotation1 = digitalRead(ROTATIONWHEEL1);              //reads sensor 1
        if(currentStateRotation1 == HIGH && lastStateRotation1 == LOW ){  //pulse detection - by comparing previous state, it prevents multiple pulse counts
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
