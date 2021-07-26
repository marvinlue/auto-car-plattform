#include <Servo.h>
int motorPin = D3;
int steerPin = D6;
int servoNeutral = 5;
int step;

Servo servo;


void setup() {
pinMode(steerPin,OUTPUT);
analogWriteFrequency(50);
analogWrite(steerPin, servoNeutral);

servo.attach(motorPin);
servo.writeMicroseconds(1500); // send "stop" signal to ESC. Also necessary to arm the ESC.
delay(2000); // delay to allow the ESC to recognize the stopped signal.
}

void loop() {
  // put your main code here, to run repeatedly:
  for (step = 0; step <= 5; step += 1) {
    servo.writeMicroseconds(1500+step*10);
    analogWrite(steerPin, servoNeutral+step);
    delay(50);                       
  } 
for (step = 5; step >= -5; step -= 1) {
    servo.writeMicroseconds(1500+step*10);
    analogWrite(steerPin, servoNeutral+step);
    delay(50);                       
  } 
}
