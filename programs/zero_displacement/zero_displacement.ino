#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);


#define mlp 15
#define mld 16  // Motor port 1
#define mlf 17

// #define mlp 25
// #define mld 26   // motor port 3
// #define mlf 27

int left_tick = 0;
int right_tick = 0;
bool left_flag = HIGH;
bool right_flag = HIGH;
int power = 200;
int powr = 200;
int val = 270;

void setup() {
  dcinitial();
  delay(1000);
  servoinitial();
  delay(1000);
  action1();
  delay(1000);
  action2();
  delay(1000);
  action3();
}

void dcinitial() {
Serial.begin(115200);
pinMode(mlp, OUTPUT); 
pinMode(mld, OUTPUT); 
attachInterrupt(digitalPinToInterrupt(mlf), left_wheel_pulse, RISING);
display.begin(SSD1306_SWITCHCAPVCC, 0x3C);                                //Setting the initial power and rotation of the dc motor to null under the function "dcinitial"
display.clearDisplay();
display.setTextSize(1);
display.setTextColor(WHITE);

move_degree_left(true, 255, 0);
  // delay(1000);
}

int servoPin = 5;

Servo Servo1;
void servoinitial() {
Servo1.attach(servoPin);
Servo1.write(100);          //Setting the initial position of the servo straight (90 degrees) under the function "servoinitial"
  // delay(1000);
}

void action1() {
move_degree_left(true, 100, 3600);
  // delay(1000);
}

void action2() {
Servo1.write(77);
  // delay(1000);
}

void action3() {
move_degree_left(true, 100, 1800);
  // delay(1000);
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

void left_wheel_pulse() {
 if (left_flag){
  left_tick++;
}
else{
   left_tick--;
}
}