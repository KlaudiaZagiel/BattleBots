const int TRIGGERPIN = 5;
const int ECHOPIN = 13;

long duration; // in microseconds
float distance; // will store distance in cm

void setup()
{
  Serial.begin(9600);
  pinMode(TRIGGERPIN, OUTPUT);
  pinMode(ECHOPIN, INPUT);
}

void loop() 
{
  // Send trigger pulse
  digitalWrite(TRIGGERPIN, HIGH);
  delayMicroseconds(2);

  digitalWrite(TRIGGERPIN, LOW);
  delayMicroseconds(10);

  digitalWrite(TRIGGERPIN, HIGH);

  duration = pulseIn(ECHOPIN, LOW);

  distance = duration * 0.034 / 2;

  Serial.print("Distance: ");
  Serial.println(distance);

  delay(500);
}
