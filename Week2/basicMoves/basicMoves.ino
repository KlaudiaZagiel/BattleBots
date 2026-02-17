const int MOTOR1BACKWARD = 11;
const int MOTOR1FORWARD = 10;
const int MOTOR2FORWARD = 9;
const int MOTOR2BACKWARD = 6;
const int ROTATIONWHEEL1 = 12; //rotation sensor
const int ROTATIONWHEEL2 = 8;
const int BUTTONFORWARD = 7;
bool lastStateRotation2 = LOW; 
bool lastStateRotation1 = LOW;
bool forwardComplete = false; //in logic mostly u want to start in a false state to switch some logic on
bool backwardComplete = false;
bool movingForward = false; //switch to reach forward function without clicking the button bcs otherwise button needs to be pressed again in order to access the forward function
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

 void forward(){
if(pulseCount1 <= 98 && pulseCount2 <= 98){
  Serial.print("entered forward");
  movingForward = true;
  startMotorsForward();
}else{
  stopMotors();
  movingForward = false;
  forwardComplete = true; //if triggers it marks the end of the forward cycle.
 }
}

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

void stopMotors(){
 digitalWrite(MOTOR1BACKWARD, LOW);
 digitalWrite(MOTOR1FORWARD, LOW);
 digitalWrite(MOTOR2FORWARD, LOW);
 digitalWrite(MOTOR2BACKWARD, LOW);
 pulseCount1 = 0;
 pulseCount2 = 0;
}

void startMotorsForward(){
 digitalWrite(MOTOR1BACKWARD, LOW);
 analogWrite(MOTOR1FORWARD, 248); //for PWM we use analogWrite. we are giving a value between 0-255. this allows us to control the speed of the motor(s).
 analogWrite(MOTOR2FORWARD, 250); //this basically controls the direction of the robot though motor speed.
 digitalWrite(MOTOR2BACKWARD, LOW); //never put forward and backwards high together because its bad for the hardware - voltage on + and - side. no turning wheels = both low. (off
}

void startMotorsBackward(){
 analogWrite(MOTOR1BACKWARD, 242);
 digitalWrite(MOTOR1FORWARD, LOW); //for PWM we use analogWrite. we are giving a value between 0-255. this allows us to control the speed of the motor(s).
 digitalWrite(MOTOR2FORWARD, LOW); //this basically controls the direction of the robot though motor speed.
 analogWrite(MOTOR2BACKWARD, 250); //never put forward and backwards high together because its bad for the hardware - voltage on + and - side. no turning wheels = both low. (off
}

void startMotorLeft(){ //this triggers the motor to go left (what to do with wheels)
 digitalWrite(MOTOR1BACKWARD, LOW);
 digitalWrite(MOTOR1FORWARD, LOW);
 analogWrite(MOTOR2FORWARD, 255); 
 digitalWrite(MOTOR2BACKWARD, LOW);
}

void turnLeft(){ //here we decide the logic of when the turn will take a place
  if(pulseCount2 <= 18){
    startMotorLeft();
  }else{
    stopMotors();
    backwardComplete = false;
    turnLeftComplete = true;
 }
}

void startMotorRight(){ //this triggers the motor to go left (what to do with wheels)
 digitalWrite(MOTOR1BACKWARD, LOW);
 analogWrite(MOTOR1FORWARD, 255);
 digitalWrite(MOTOR2FORWARD, LOW); 
 digitalWrite(MOTOR2BACKWARD, LOW);
}

void turnRight(){ //here we decide the logic of when the turn will take a place
  if(pulseCount1 <= 18){
    startMotorRight();
  }else{
    stopMotors();
    turnLeftComplete = false;
 } 
}



void loop() {
bool currentStateRotation2 = digitalRead(ROTATIONWHEEL2); //read the current value of the rotation sensor
if(currentStateRotation2 == HIGH && lastStateRotation2 == LOW ){  //in serial monitor HIGH = 1, LOW = 0
  pulseCount2++;
  }
  lastStateRotation2 = currentStateRotation2; 

bool currentStateRotation1 = digitalRead(ROTATIONWHEEL1); //read the current value of the rotation sensor
if(currentStateRotation1 == HIGH && lastStateRotation1 == LOW ){  //in serial monitor HIGH = 1, LOW = 0
  pulseCount1++;
  }
  lastStateRotation1 = currentStateRotation1;
  
 //how many pulses is 1m? diamater of wheel = 6.5cm  circumference of the wheel roughly 20cm (6.5 * pi), we know 1 full rotation is 20 pulses (measured through sensor). this means that 1 pulse is about 1cm. 1m = 100 pulses (roughly)
 //(wo rounding errors 1m=97.94pulses~~98pulses)
 
 int forwardState = digitalRead(BUTTONFORWARD);
 if(forwardState == LOW || movingForward == true){
    forward(); //call up the function
 }
 Serial.println(pulseCount1);

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
