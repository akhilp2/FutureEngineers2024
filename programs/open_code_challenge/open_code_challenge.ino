#include "Adafruit_TCS34725.h"
#include <Servo.h>
#include <SPI.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <SharpIR.h>
#define irleft A0
#define modelleft 20150

#define irright A2
#define modelright 20150

SharpIR SharpIRleft(irleft, modelleft);
SharpIR SharpIRright(irright, modelright);

// Pin of Servo (front wheels)
Servo myservo;
const int servoPin = 9;

// Pin of DC Motor (back wheels)
#define mlp 3
#define mld 4  // Motor port 1
#define mlf 2

int left_tick = 0;
bool left_flag = HIGH;
int power = 200;
unsigned long time1 = 0;

// Gyro Setup
MPU6050 mpu(Wire);
unsigned long timer = 0;
int angle = 0;
int angle1 = 0;
int counter = 0;

// Button Setup
int buttonState = 1;
const int buttonPin = 8;

// Colour Sensor Setup
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);
bool dir = false;

void setup() {
  Serial.begin(9600);
  myservo.attach(servoPin, 500, 2500);
  myservo.write(112);
  delay(100);

  Serial.begin(9600);
  pinMode(mlp, OUTPUT);
  pinMode(mld, OUTPUT);
  pinMode(buttonPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(mlf), left_wheel_pulse, RISING);
  digitalWrite(mlp, HIGH);
  digitalWrite(buttonPin, HIGH);

  Wire.begin();
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0)
    ;

  analogWrite(mlp, 255);
  digitalWrite(mld, true);

  while (buttonState == HIGH) {
    buttonState = digitalRead(buttonPin);
  }
  delay(500);
  mpu.update();
  int disfrmright = SharpIRright.distance();

  if (disfrmright < 70) {
    myservo.write(130);
    while (mpu.getAngleZ() < 40) {
      mpu.update();
      analogWrite(mlp, 100);
      digitalWrite(mld, true);
    }
    myservo.write(94);
    while (mpu.getAngleZ() > 5) {
      mpu.update();
      analogWrite(mlp, 100);
      digitalWrite(mld, true);
    }
  } else if (disfrmright > 100) {
    myservo.write(94);
    while (mpu.getAngleZ() > -40) {
      mpu.update();
      analogWrite(mlp, 100);
      digitalWrite(mld, true);
    }
    myservo.write(130);
    while (mpu.getAngleZ() < -5) {
      mpu.update();
      analogWrite(mlp, 100);
      digitalWrite(mld, true);
    }
  }
}

void loop() {
  uint16_t r, g, b, c, colorTemp, lux;
  tcs.getRawData(&r, &g, &b, &c);
  colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
  lux = tcs.calculateLux(r, g, b);

  // Serial.print("Color Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
  // Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" - ");
  // Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
  // Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
  // Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
  // Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
  // Serial.println(" ");

  mpu.update();
  int servoAngle = map((mpu.getAngleZ() - angle1), 30, -30, 93, 133);  // Adjust the mapping as necessary
  myservo.write(servoAngle);
  analogWrite(mlp, 170);
  digitalWrite(mld, true);

  int disfrmleft = SharpIRleft.distance();
  int disfrmright = SharpIRright.distance();

  Serial.print("Distance from Left: ");
  Serial.print(disfrmleft);
  Serial.print(" ----- Distance from Right: ");
  Serial.println(disfrmright);

  if (counter == 0) {
    servoAngle = map((mpu.getAngleZ() - angle1), 30, -30, 93, 133);
    myservo.write(servoAngle);
  } else {
    if (dir) {  //true //orange
      if (disfrmleft < 25) {
        myservo.write(101);
      } else if (disfrmleft > 35 && disfrmleft < 100) {
        myservo.write(125);
      }
    } else {  //blue
      if (disfrmright < 45) {
        myservo.write(125);
      } else if (disfrmright > 55 && disfrmright < 100) {
        myservo.write(101);
      }
    }
  }
  if (lux < 10 && r > 20) {  // orange
    angle1 -= 90;
    myservo.write(95);
    dir = true;
    while (mpu.getAngleZ() > angle1 + 2) {
      mpu.update();
      analogWrite(mlp, 100);
      digitalWrite(mld, true);
    }
    counter += 1;
    move_degree_for_orange(true, 0, 1800, angle1);
  }
  if (lux < 6 && r < 15) {  // blue
    // digitalWrite(13, LOW);
    angle1 += 90;
    dir = false;
    myservo.write(130);
    while (mpu.getAngleZ() < angle1 - 2) {
      mpu.update();
      analogWrite(mlp, 100);
      digitalWrite(mld, true);
    }
    counter += 1;
    move_degree_for_blue(true, 0, 1800, angle1);
  }

  if (counter == 12) {
    move_degree(false, 0, 0);
    analogWrite(mlp, 255);
    delay(10000);
    counter += 1;
  }

  if (counter == 13) {
    // move_degree(true, 2160, 0);
    move_degree(false, 0, 0);
    analogWrite(mlp, 255);
    delay(15000);
  }
}


//---------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------


//gyro straighting untill degree of the motor and remaing there
void move_degree_for_blue(bool dir, int pace, float dis, int angle) {
  left_tick = 0;
  left_flag = dir;
  digitalWrite(mlp, HIGH);
  analogWrite(mlp, pace);
  digitalWrite(mld, left_flag);
  int disfrmright = SharpIRright.distance();
  int disfrmleft = SharpIRleft.distance();

  while (abs(left_tick) < dis * 0.75) {
    mpu.update();
    int servoAngle = map((mpu.getAngleZ() - angle1), 30, -30, 93, 133);  // Adjust the mapping as necessary
    myservo.write(servoAngle);
    // int disfrmleft = SharpIRfront.distance();
    int disfrmright = SharpIRright.distance();
    int disfrmleft = SharpIRleft.distance();

    //option1--------------------------------------------------------------------------------------------------
    if (disfrmright < 35) {
      myservo.write(125);
      delay(5);
    } else if (disfrmright > 55 && disfrmright < 100) {
      myservo.write(101);
      delay(5);
    } else {
      mpu.update();
      int servoAngle = map((mpu.getAngleZ() - angle1), 30, -30, 93, 133);  // Adjust the mapping as necessary
      myservo.write(servoAngle);
    }
    delayMicroseconds(1);
  }

  left_tick = 0;
}

//---------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------

void move_degree_for_orange(bool dir, int pace, float dis, int angle) {
  left_tick = 0;
  left_flag = dir;
  digitalWrite(mlp, HIGH);
  analogWrite(mlp, pace);
  digitalWrite(mld, left_flag);
  int disfrmright = SharpIRright.distance();
  int disfrmleft = SharpIRleft.distance();

  while (abs(left_tick) < dis * 0.75) {

    mpu.update();
    int servoAngle = map((mpu.getAngleZ() - angle1), 30, -30, 95, 135);  // Adjust the mapping as necessary
    myservo.write(servoAngle);
    int disfrmleft = SharpIRleft.distance();
    int disfrmright = SharpIRright.distance();

    //option1--------------------------------------------------------------------------------------------------
    if (disfrmleft < 25) {
      myservo.write(101);
      delay(5);
    } else if (disfrmleft > 35 && disfrmleft < 150) {
      myservo.write(125);
      delay(5);
    } else {
      mpu.update();
      int servoAngle = map((mpu.getAngleZ() - angle1), 50, -50, 95, 135);  // Adjust the mapping as necessary
      myservo.write(servoAngle);
    }
    delayMicroseconds(1);
  }

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
    int servoAngle = map((mpu.getAngleZ() - angle - 5), 50, -50, 94, 134);  // Adjust the mapping as necessary
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