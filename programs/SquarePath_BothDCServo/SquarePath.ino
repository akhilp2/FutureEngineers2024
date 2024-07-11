#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

int servoPin = 5;
Servo Servo1;

#define mlp 15
#define mld 16  // Motor port 1
#define mlf 17

int left_tick = 0;
int right_tick = 0;
bool left_flag = HIGH;
bool right_flag = HIGH;
int power = 200;
int powr = 200;
unsigned long time1 = 0;
int val = 270;

void setup() {
Servo1.attach(servoPin);
Serial.begin(115200);
pinMode(mlp, OUTPUT); 
pinMode(mld, OUTPUT);
attachInterrupt(digitalPinToInterrupt(mlf), left_wheel_pulse, RISING);
display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
display.clearDisplay();
display.setTextSize(1);
display.setTextColor(WHITE);

// Set initial Steering Position to 90 degrees
move_degree_left(true, 255, 00);
Servo1.write(100);
delay(1000);

// Move forward 10 Rotations
move_degree_left(true, 100, 3600);
delay(1000);
// Turn Right
Servo1.write(77);
delay(1000);
// Move forward until car is facing straight
move_degree_left(true, 100, 360*5);
delay(1000);
// Turn Wheel Straight (to 90 degrees)
Servo1.write(100);
delay(1000);
// Go Straight for 10 Rotations
move_degree_left(true, 100, 3600);
delay(1000);
}

void loop() {
}

void move_degree_left( bool dir, int pace, float dis){
    left_flag = dir;
    analogWrite(mlp, pace ); 
    digitalWrite(mld,left_flag);
  while( abs(left_tick)<dis*0.75)
  {
    
//    continue;
//    Serial.println("moving");
delayMicroseconds(1);
//    Serial.println(i);
    
  }
  analogWrite(mlp, 255); 
    digitalWrite(mld, !left_flag);
    delay(10);
  display.clearDisplay();
    display.setCursor(0,0);
    display.print(left_tick*1.333);
    display.display();
  Serial.println("done moving");
    left_tick=0;
  
}

// void move_degree_right( bool dir, int pace, float dis){
//     right_flag = dir;
//     analogWrite(mrp, pace ); 
//     digitalWrite(mrd,right_flag);
//   while( abs(right_tick)<dis*0.75)
//   {
    
// //    continue;
// //    Serial.println("moving");
// delayMicroseconds(1);
// //    Serial.println(i);
    
//   }
//   analogWrite(mrp, 255); 
//     digitalWrite(mrd, !right_flag);
//     delay(10);
//   display.clearDisplay();
//     display.setCursor(0,0);
//     display.print(right_tick*1.333);
//     display.display();
//   Serial.println("done moving");
//     right_tick=0;
  
// }

void left_wheel_pulse() {
   
  // Read the value for the encoder for the right wheel

 if (left_flag){
  left_tick++;
}
else{
   left_tick--;
}
}

void right_wheel_pulse() {
   
  // Read the value for the encoder for the right wheel

 if (right_flag){
  right_tick++;
}
else{
   right_tick--;
}

}
