#include <ESP32Servo.h>

int servoPin = 5;

Servo Servo1;
void setup(){
Servo1.attach(servoPin);
// myservo.attach(servoPin, 500, 2500);
}

void loop(){
  // Make servo go to 100 degrees
  Servo1.write(100);
  delay(2000);
  // Make servo go to 121 degrees
  Servo1.write(121);
  delay(2000);
  // Make servo go back to 100 degrees
  Servo1.write(100);
  delay(2000);
  //Make servo go to 77 degrees
  Servo1.write(77);
  delay(2000);
}