int sensors[8] = {A0, A1, A2, A3, A4, A5, A6, A7};

void setup() {
  Serial.begin(9600);
}

void loop() {
  for(int i = 0; i < 8; i++) {
    Serial.print(analogRead(sensors[i]));
    Serial.print("   ");
  }
  
  Serial.println();
  
  delay(200);
}
