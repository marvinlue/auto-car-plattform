#include <Servo.h>
int anglePin = D3;
int servoAngle = 5;
int step;

Servo servo;

void setup() {
servo.attach(D3);
servo.writeMicroseconds(2000);
}

void loop() {
  /*
  for (step = 0; step <= 180; step += 10) {
    Serial.println(step);
    servo.write(step);
    delay(500);                      
  }        
  */            
}
