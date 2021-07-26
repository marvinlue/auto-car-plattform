#include <Servo.h>
int servoPin = 3;
int step;

Servo servo;

void setup() {
servo.attach(servoPin,50,2000);
servo.writeMicroseconds(200);
delay(5000);
}

void loop() {
  
  for (step = 150; step <= 260; step += 10) {
    servo.writeMicroseconds(step);
    delay(500);                      
  }   
  for (step = 260; step >= 150; step -= 10) {
    servo.writeMicroseconds(step);
    delay(500);                      
  }     
  
}
