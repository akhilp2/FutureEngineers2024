#include <Wire.h>
#include "Adafruit_TCS34725.h"

#include <MPU6050_light.h>
#include <SPI.h>
#include <Servo.h>

Servo myservo;
const int servoPin = 9;

#define mlp 3
#define mld 4  // Motor port 1
#define mlf 2

int left_tick = 0;
bool left_flag = HIGH;
int power = 200;
unsigned long time1 = 0;

MPU6050 mpu(Wire);
unsigned long timer = 0;

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_4X);

void setup() {
  Serial.begin(9600);

  myservo.attach(servoPin, 500, 2500);
  myservo.write(113);


  Serial.begin(115200);
  pinMode(mlp, OUTPUT); 
  pinMode(mld, OUTPUT); 
  attachInterrupt(digitalPinToInterrupt(mlf), left_wheel_pulse, RISING);
  digitalWrite(mlp, HIGH);

  Wire.begin();
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status != 0) {}

  myservo.write(113);

  move_degree_left(true, 150, 2880,0);
 
  move_degree_left(true, 150, 3240,-90);

  move_degree_left(false, 150, 720,-90);
  
  move_degree_left(true, 150, 6120,-180);

}

void loop(void) {
    uint16_t r, g, b, c, colorTemp, lux;

    tcs.getRawData(&r, &g, &b, &c);
    // colorTemp = tcs.calculateColorTemperature(r, g, b);
    colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
    lux = tcs.calculateLux(r, g, b);

  Serial.print("Color Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
  Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" - ");
  Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
  Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
  Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
  Serial.println(" ");

  int col = color(lux,r,b,c,g);

}

int color( int l, int r, int b, int c, int g){
int color_value = 0;
if (l<100){
Serial.println("black");
color_value = 0; 
}
else if (l>4000){
Serial.println("red");
color_value = 1;
move_degree_left(true, 255, 0,0);
delay(1000);
}
else if(l<1000 && b>1000 && c>4800){
Serial.println("white");
color_value = 2;
}
else if (c<2000 && l>100 && l<300){
Serial.println("blue");
color_value = 3;
}
else if(g> 600 && g<900 &&c>2000 && c<3000){
Serial.println("green");
color_value = 4;
move_degree_left(true, 10, 360, 0);
delay(1000);
}
else{
Serial.println("yellow");
color_value = 5;
move_degree_left(true, 200, 180, 0);
}

}

void move_degree_left(bool dir, int pace, float dis, int angle) {
  left_flag = dir;
  digitalWrite(mlp, HIGH);
  analogWrite(mlp, pace); 
  digitalWrite(mld, left_flag);

  while(abs(left_tick) < dis * 0.75) {
    delayMicroseconds(1);
     mpu.update();
  int servoAngle = map((mpu.getAngleZ()-angle), 30, -30, 90, 132);  // Adjust the mapping as necessary
  myservo.write(servoAngle);
    // Serial.println(left_tick);
  }
  
  analogWrite(mlp, 255); 
  digitalWrite(mld, !left_flag);
  delay(10);
  Serial.println("done moving");
  left_tick = 0;
}

void left_wheel_pulse() {
  if (left_flag) {
    left_tick++;
  } else {
    left_tick--;
  }
}

