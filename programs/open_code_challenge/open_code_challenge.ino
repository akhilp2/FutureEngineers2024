
#include "Adafruit_TCS34725.h"
#include <Servo.h>
#include <SPI.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <SharpIR.h>
#define irfront A0
#define modelfront 20150

#define irright A2
#define modelright 20150

SharpIR SharpIRleft(irfront, modelfront);
SharpIR SharpIRright(irright, modelright);

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
int angle = 0;
int angle1 = 0;
int counter = 0;

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_4X);

void setup() {
  Serial.begin(9600);
  myservo.attach(servoPin, 500, 2500);
  myservo.write(113);
  delay(100);


  Serial.begin(9600);
  pinMode(mlp, OUTPUT);
  pinMode(mld, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(mlf), left_wheel_pulse, RISING);
  digitalWrite(mlp, HIGH);


  Wire.begin();
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0)
    ;
}

void loop() {
  uint16_t r, g, b, c, colorTemp, lux;
  tcs.getRawData(&r, &g, &b, &c);
  colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
  lux = tcs.calculateLux(r, g, b);

  Serial.print("Color Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
  Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" - ");
  Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
  Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
  Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
  Serial.println(" ");

  if (lux > 1000) {
    lux = 1000;
  }

  if (lux < 100){
    digitalWrite(13, HIGH);
  }

  // if (lux > 1000){
  //   digitalWrite(13, HIGH);
  // }
  mpu.update();
  int disfrmleft = SharpIRleft.distance();
  int disfrmright = SharpIRright.distance();
  // myservo.write(113);
  int servoAngle = map((mpu.getAngleZ() - angle1), 50, -50, 90, 132);  // Adjust the mapping as necessary
  myservo.write(servoAngle);
  analogWrite(mlp, 100);
  digitalWrite(mld, true);
  
  if (lux > 800) {
    digitalWrite(13, LOW);
    angle1 -= 90;
    myservo.write(93);

    while (mpu.getAngleZ() > angle1 + 2) {
      mpu.update();
      analogWrite(mlp, 100);
      digitalWrite(mld, true);
    }
    counter += 1;
    move_degree_left(true, 0, 2160, angle1);
  }
  else if (lux < 50) {
    // digitalWrite(13, LOW);
    angle1 += 90;
    myservo.write(128);
    while (mpu.getAngleZ() < angle1 - 2) {
      mpu.update();
      analogWrite(mlp, 100);
      digitalWrite(mld, true);
    }
    counter += 1;
    move_degree_left(true, 0, 2160, angle1);
  }

  if (counter == 12) {
    move_degree(false, 0, 0);
    analogWrite(mlp, 255);
    delay(10000);
    counter += 1;
  }

  if (counter == 13) {
    move_degree(true, 255, 0);
  }
}


//gyro straighting untill degree of the motor and remaing there
void move_degree_left(bool dir, int pace, float dis, int angle) {
  left_tick = 0;
  left_flag = dir;
  digitalWrite(mlp, HIGH);
  analogWrite(mlp, pace);
  digitalWrite(mld, left_flag);
  int disfrmright = SharpIRright.distance();
  int disfrmleft = SharpIRleft.distance();

  while (abs(left_tick) < dis * 0.75) {

    mpu.update();
    int servoAngle = map((mpu.getAngleZ() - angle1), 50, -50, 90, 132);  // Adjust the mapping as necessary
    myservo.write(servoAngle);
    // int disfrmleft = SharpIRfront.distance();
    int disfrmright = SharpIRright.distance();
    if (disfrmright < 50) {
      myservo.write(132);
    } else if (disfrmright > 60) {
      myservo.write(90);
    } else if (disfrmleft < 50) {
      myservo.write(90);
    } else if (disfrmleft > 60) {
      myservo.write(132);
    } else {
      mpu.update();
      int servoAngle = map((mpu.getAngleZ() - angle1), 50, -50, 90, 132);  // Adjust the mapping as necessary
      myservo.write(servoAngle);
    }
    //   while (mpu.getAngleZ() < angle1) {
    //   mpu.update();
    //   myservo.write(113 + ((30 - disfrmright) / 10 ));
    //   analogWrite(mlp, 100);
    //   digitalWrite(mld, true);
    // }
    delayMicroseconds(1);
  }

  // analogWrite(mlp, 255);
  // digitalWrite(mld, !left_flag);
  // delay(10);
  // Serial.println("done moving");
  left_tick = 0;
}




//gyro straighting untill the angle
void move_degree_angle(bool dir, int pace, float dis, int angle) {
  left_tick = 0;
  left_flag = dir;
  // digitalWrite(mlp, HIGH);
  analogWrite(mlp, pace);
  digitalWrite(mld, left_flag);

  while (mpu.getAngleZ() < angle) {
    delayMicroseconds(1);
    mpu.update();
    int servoAngle = map((mpu.getAngleZ() - angle - 5), 50, -50, 90, 132);  // Adjust the mapping as necessary
    myservo.write(servoAngle);
    // Serial.println(left_tick);
  }
  analogWrite(mlp, 255);
  digitalWrite(mld, !left_flag);
  delay(10);
  Serial.println("done moving");
  left_tick = 0;
}

// just motor
void move_degree(bool dir, int pace, float dis) {
  left_tick = 0;
  left_flag = dir;
  digitalWrite(mlp, HIGH);
  analogWrite(mlp, pace);
  digitalWrite(mld, left_flag);

  while (abs(left_tick) < dis * 0.75) {
    delayMicroseconds(1);
    Serial.println(left_tick);
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