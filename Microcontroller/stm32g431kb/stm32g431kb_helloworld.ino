
/*
 * First 'Hello World' function 
 *  post upcounting string from STM32 to PC serial output 
 *  time interval: 1s
 * 
 */

// Global variable definitions
int counter = 0;

void setup() {
  // put your setup code here, to run once:

  // Serial.begin(long baudrate, [config]) : Setup data transmission, config define data, parity, stop bits if more serial ports available reach them with Serial1, ...
  Serial.begin(9600);

  // Serial.println(val ,  [format]) : prints a new line
  Serial.println('Counter starts now');
}

void loop() {
  // put your main code here, to run repeatedly:
  counter += 1;
  Serial.println('Counter is at:');
  Serial.println(counter);
  
  // delay (ulong intervall): pause program for intervall ms  
  delay(1000);
}
