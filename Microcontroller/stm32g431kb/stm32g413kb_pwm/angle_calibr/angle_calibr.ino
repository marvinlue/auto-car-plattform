int anglePin = D3;
int servoAngle = 250;

void setup() {
pinMode(anglePin,OUTPUT);
analogWriteFrequency(50);
analogWrite(anglePin, servoAngle);
}

void loop() {
  
  /*
   * Serial.println('Counter is at:');  
  for (step = 0; step <= 15; step += 1) {
    Serial.println(step);
    analogWrite(anglePin, step);
    delay(500);                      
  } 
  */                   
}
