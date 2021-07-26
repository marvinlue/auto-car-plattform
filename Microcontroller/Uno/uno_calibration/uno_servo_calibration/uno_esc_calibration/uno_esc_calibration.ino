#include <Servo.h>
int motorPin = 3;
int neutral = 1500;
int step;

Servo servo;


void setup() {
Serial.begin(9600);

servo.attach(motorPin);
servo.writeMicroseconds(1500); // send "stop" signal to ESC. Also necessary to arm the ESC.
delay(2000); // delay to allow the ESC to recognize the stopped signal.

servo.writeMicroseconds(1500);
}

void loop() {

for (step = 0; step <=50; step += 10) {
  Serial.println("Current PWM Signal: ");
  Serial.println(neutral + step);
  servo.writeMicroseconds(neutral + step);
  delay(3000);
}
for (step = 50; step >= 0; step -= 10) {
  Serial.println("Current PWM Signal: ");
  Serial.println(neutral + step);
  servo.writeMicroseconds(neutral + step);
  delay(3000);
}
}
