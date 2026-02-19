const int MOTOR1BACKWARD = 11;
const int MOTOR1FORWARD = 10;
const int MOTOR2FORWARD = 9;
const int MOTOR2BACKWARD = 6;
const int ROTATIONWHEEL1 = 12; //rotation sensor.
const int ROTATIONWHEEL2 = 8;
const int BUTTONFORWARD = 7;
bool lastStateRotation2 = LOW; //this prevents counting the same pulse multiple times and tells what the rotation sensor was reading last time the loop ran
bool lastStateRotation1 = LOW;
bool forwardComplete = false; //movement sequence booleans
bool backwardComplete = false;
bool movingForward = false; 
bool turnLeftComplete = false;
int pulseCount2 = 0; //pulse count for motor 2
int pulseCount1 = 0; //pulse count for motor 1

void setup() {
pinMode(MOTOR1BACKWARD, OUTPUT);
pinMode(MOTOR1FORWARD, OUTPUT);
pinMode(MOTOR2BACKWARD, OUTPUT);
pinMode(MOTOR2FORWARD, OUTPUT);
pinMode(ROTATIONWHEEL1, INPUT);
pinMode(ROTATIONWHEEL2, INPUT);
pinMode(BUTTONFORWARD, INPUT);
Serial.begin(9600);

}

//Drive forward//
void forward(){
if(pulseCount1 <= 98 && pulseCount2 <= 98){
  Serial.print("entered forward");
  movingForward = true;
  startMotorsForward();
  
}else{
  stopMotors();
  movingForward = false;
  forwardComplete = true; //if triggers it and marks the end of the forward cycle
 }
}

//Drive backward//
void backward(){
if(pulseCount1 <= 98 && pulseCount2 <= 98){
  Serial.print("entered backward");
  startMotorsBackward();
  
}else{
  stopMotors();
  forwardComplete = false;
  backwardComplete = true;
 }
}

//Stop motors//
void stopMotors(){
 digitalWrite(MOTOR1BACKWARD, LOW);
 digitalWrite(MOTOR1FORWARD, LOW);
 digitalWrite(MOTOR2FORWARD, LOW);
 digitalWrite(MOTOR2BACKWARD, LOW);
 pulseCount1 = 0;
 pulseCount2 = 0;
}

//Start Motors to move forward//
void startMotorsForward(){
 digitalWrite(MOTOR1BACKWARD, LOW);
 analogWrite(MOTOR1FORWARD, 248); 
 analogWrite(MOTOR2FORWARD, 250); 
 digitalWrite(MOTOR2BACKWARD, LOW); //never put forward and backwards high together because its bad for the hardware - voltage on + and - side.
}

//Start motors go move backward//
void startMotorsBackward(){
 analogWrite(MOTOR1BACKWARD, 242);
 digitalWrite(MOTOR1FORWARD, LOW); 
 digitalWrite(MOTOR2FORWARD, LOW); 
 analogWrite(MOTOR2BACKWARD, 250); 
}

//Turn motor 2 to turn left//
void startMotorLeft(){ //this triggers the motor to go left (what to do with wheels)
 digitalWrite(MOTOR1BACKWARD, LOW);
 digitalWrite(MOTOR1FORWARD, LOW);
 analogWrite(MOTOR2FORWARD, 255); 
 digitalWrite(MOTOR2BACKWARD, LOW);
}

//Turn left//
void turnLeft(){ //here we decide the logic of when the turn will take a place
  if(pulseCount2 <= 18){
    startMotorLeft();
    
  }else{
    stopMotors();
    backwardComplete = false;
    turnLeftComplete = true;
 }
}

//Turn motor 1 to turn right//
void startMotorRight(){ //this triggers the motor to go left (what to do with wheels)
 digitalWrite(MOTOR1BACKWARD, LOW);
 analogWrite(MOTOR1FORWARD, 255);
 digitalWrite(MOTOR2FORWARD, LOW); 
 digitalWrite(MOTOR2BACKWARD, LOW);
}

//Turn right//
void turnRight(){
  if(pulseCount1 <= 18){
    startMotorRight();
    
  }else{
    stopMotors();
    turnLeftComplete = false;
 } 
}

void loop() {
//Count wheel pulses//
bool currentStateRotation2 = digitalRead(ROTATIONWHEEL2); //reads sensor 2
if(currentStateRotation2 == HIGH && lastStateRotation2 == LOW ){ //pulse detection - by comparing previous state, it prevents multiple pulse counts
  pulseCount2++; //new pulse occured - increase it by 1
  }
  lastStateRotation2 = currentStateRotation2; //

bool currentStateRotation1 = digitalRead(ROTATIONWHEEL1);
if(currentStateRotation1 == HIGH && lastStateRotation1 == LOW ){  
  pulseCount1++;
  }
  lastStateRotation1 = currentStateRotation1;
  
 //how many pulses is 1m? diamater of wheel = 6.5cm  circumference of the wheel roughly 20cm (6.5 * pi), 
 //we know 1 full rotation is 20 pulses (measured through sensor). this means that 1 pulse is about 1cm. 1m = 100 pulses (roughly)
 //(wo rounding errors 1m=97.94pulses~~98pulses)

 //Button press//
 int forwardState = digitalRead(BUTTONFORWARD);
 if(forwardState == LOW || movingForward == true){ //if button pressed or already moving - go forward. this prevents holding button
    forward();
 }

 if(forwardComplete == true){
  backward();
 }

 if(backwardComplete == true){
  turnLeft();
 }

 if(turnLeftComplete == true){
  turnRight();
 }
}
