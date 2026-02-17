const int numSensors = 8;
int sensorPins[numSensors] = {A0, A1, A2, A3, A4, A5, A6, A7};

void setup() {
  Serial.begin(9600);
}

void loop() {

  for(int i = 0; i < numSensors; i++) {
    int value = analogRead(sensorPins[i]);
    Serial.print(value);
    Serial.print("\t");   // spacing between values
  }

  Serial.println();  // new line
  delay(200);
}
