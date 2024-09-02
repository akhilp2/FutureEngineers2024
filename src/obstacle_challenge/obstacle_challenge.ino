#include "Adafruit_TCS34725.h"
#include <Servo.h>
#include <SPI.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <Pixy2.h>
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
bool blue = true;
bool orange = true;
// Pin of DC Motor (back wheels)
#define mlp 3
#define mld 4  // Motor port 1
#define mlf 2

int sig;

int left_tick = 0;
bool left_flag = HIGH;
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

// Pixy Setup
Pixy2 pixy;

bool parking = false;

void setup() {
  Serial.begin(115200);

  pixy.init();

  myservo.attach(servoPin, 500, 2500);
  myservo.write(112);
  delay(100);

  pinMode(mlp, OUTPUT);
  pinMode(mld, OUTPUT);
  pinMode(buttonPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(mlf), left_wheel_pulse, RISING);
  digitalWrite(mlp, HIGH);
  digitalWrite(buttonPin, HIGH);

  Wire.begin();
  byte status = mpu.begin();

  analogWrite(mlp, 255);
  digitalWrite(mld, true);

  while (buttonState == HIGH) {
    buttonState = digitalRead(buttonPin);
  }
  delay(500);

  mpu.update();
}

void loop() {
  uint16_t r, g, b, c, colorTemp, lux;
  tcs.getRawData(&r, &g, &b, &c);
  colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
  lux = tcs.calculateLux(r, g, b);

  mpu.update();
  int servoAngle = map((mpu.getAngleZ() - angle1 - 2), 80, -80, 74, 154);  // Adjust the mapping as necessary
  myservo.write(servoAngle);
  analogWrite(mlp, 150);
  digitalWrite(mld, true);

  int disfrmleft = SharpIRleft.distance();
  int disfrmright = SharpIRright.distance();

  // Serial.print("Left: ");  // returns it to the serial monitor
  // Serial.print(disfrmleft);
  // Serial.print(" || Right: ");  // returns it to the serial monitor
  // Serial.println(disfrmright);

  // // Serial.print("X : ");
  // // Serial.print(mpu.getAngleX());
  // // Serial.print("\tY : ");
  // // Serial.print(mpu.getAngleY());
  // Serial.print("Z:  ");
  // Serial.println(mpu.getAngleZ());

  int i;
  pixy.ccc.getBlocks();

  if (lux < 6 && r < 15 && blue) {  // if Blue Line is detected
    blueLineTurning();
    angle1 += 90;
    counter += 1;
  }

  if (lux < 10 && r > 20 && orange) {  // if Orange Line is Detected
    orangeLineTurning();
    angle1 -= 90;
    counter += 1;
  }

  for (i = 0; i < pixy.ccc.numBlocks; i++) {
    // IF RED IS DETECTED
    redDetection(i);
    // IF GREEN IS DETECTED
    greenDetection(i);
    // IF MAGENTA IS DETECTED
    magDetectionBlue(i);
  }

  // // Option 1 WITHOUT Parking
  // if (counter == 12) {
  //   move_degree(true, 200, 2000);
  //   move_degree(false, 0, 0);
  //   analogWrite(mlp, 255);
  //   delay(10000);
  //   counter += 1;
  // }

  // Option 2 WITH Parking
  if (counter == 12) {
    // add code for car to continue in laps until parking is detected, and park in the parking
    while (!parking) {

      uint16_t r, g, b, c, colorTemp, lux;
      tcs.getRawData(&r, &g, &b, &c);
      colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
      lux = tcs.calculateLux(r, g, b);

      mpu.update();
      int servoAngle = map((mpu.getAngleZ() - angle1 - 2), 80, -80, 74, 154);  // Adjust the mapping as necessary
      myservo.write(servoAngle);
      analogWrite(mlp, 150);
      digitalWrite(mld, true);

      int disfrmleft = SharpIRleft.distance();
      int disfrmright = SharpIRright.distance();

      if (lux < 6 && r < 15 && blue) {  // if Blue Line is detected
        blueLineTurning();
        angle1 += 90;
        counter += 1;
      }

      // if (lux < 10 && r > 20 && orange) {  // if Orange Line is Detected
      //   orangeLineTurning();
      //   angle1 -= 90;
      //   counter += 1;
      // }

      int i;
      pixy.ccc.getBlocks();

      for (i = 0; i < pixy.ccc.numBlocks; i++) {
        // IF RED IS DETECTED
        redDetection(i);
        // IF GREEN IS DETECTED
        greenDetection(i);
        // IF MAGENTA IS DETECTED
        magDetectionParking(i);
      }
    }

    move_degree_gyro(true, 200, 2100);
    myservo.write(80);
    move_degree(false, 200, 1000);
    myservo.write(114);
    move_degree(false, 200, 500); //450
    delay(10000);
    counter += 1;
  }

  if (counter == 13) {
    // move_degree(true, 2160, 0);
    move_degree(false, 0, 0);
    analogWrite(mlp, 255);
    delay(15000);
  }

  // Changing from Blue to Orange
  if (blue) {
    if (counter == 8 && sig == 1) {  // if red block is the last signature that is detected at the end of lap 2
      move_degree(true, 150, 900);
      myservo.write(80);
      move_degree(false, 150, 900);
      myservo.write(114);
      angle1 += 90;

      blue = false;
      orange = true;
    } else if (counter == 8 && sig == 2) {  // if green is the last signature detected at the end of lap 2
      // do nothing keep going forward with the code
    }
  }

  // Changing from Blue to Orange
  if (orange) {
    // if (counter == 8 && sig == 1) {  // if red block is the last signature that is detected at the end of lap 2
    //   myservo.write(114);
    //   move_degree(true, 200, 360);
    //   myservo.write(154);
    //   move_degree(true, 200, 900);
    //   angle1 += 90;

    //   blue = false;
    //   orange = true;
    // } else if (counter == 8 && sig == 2) {  // if green is the last signature detected at the end of lap 2
    //   // do nothing keep going forward with the code
    // }
  }

  if (blue || orange) {
    mpu.update();
    disfrmleft = SharpIRleft.distance();
    disfrmright = SharpIRright.distance();

    if (disfrmright < 50 && disfrmleft > 120) {  //120
      myservo.write(135);                        // turn left
      delay(7);
    } else if (disfrmleft < 40 && disfrmright > 120) {  //120
      myservo.write(95);
      delay(7);
    } else {
      delay(1);
    }
  }
}

//---------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------

void turnRight(int right, int gyroAngle) {
  myservo.write(right);
  while (mpu.getAngleZ() > angle1 + gyroAngle) {
    mpu.update();
    analogWrite(mlp, 200);
    digitalWrite(mld, true);
  }
}

void turnLeft(int left, int gyroAngle) {
  myservo.write(left);
  while (mpu.getAngleZ() < angle1 + gyroAngle) {
    mpu.update();
    analogWrite(mlp, 200);
    digitalWrite(mld, true);
  }
}

void redDetection(int i) {
  if (pixy.ccc.blocks[i].m_signature == 1) {
    int x_red = pixy.ccc.blocks[i].m_x;
    int y_red = pixy.ccc.blocks[i].m_y;

    int disfrmleft = SharpIRleft.distance();
    int disfrmright = SharpIRright.distance();

    if (mpu.getAngleZ() < angle1 + 30 && mpu.getAngleZ() > angle1 - 30) {
      if (x_red > 250 && y_red > 40) {  //y_red > 35
        // Car is EXTREMELY to the LEFT of the block, turn RIGHT ABRUPTLY then straighten
        turnRight(85, -40);
        myservo.write(114);
        move_degree(true, 200, 900);  //900
        sig = 1;
      }
      if ((x_red < 250 && x_red > 170) && y_red > 40) {
        // Car is to the LEFT of the block, turn RIGHT then straighten
        turnRight(85, -35);
        myservo.write(114);
        move_degree(true, 200, 540);  //700
        sig = 1;
      }
      if ((x_red < 170 && x_red > 140) && y_red > 50) {  //35
        // Block is right INFRONT of the Car, turn RIGHT slightly then straighten
        turnRight(90, -35);
        myservo.write(114);
        move_degree(true, 200, 360);
        sig = 1;
      }
      if ((x_red < 140 && x_red > 80) && y_red > 50) {
        // Car is to the RIGHT of the block, turn RIGHT very slightly (because sensor values are not going below 100 for x) or go straight, test both
        turnRight(94, -25);  //-15
        myservo.write(114);
        move_degree(true, 200, 180);
        sig = 1;
      }
    }
  }
}

void greenDetection(int i) {
  if (pixy.ccc.blocks[i].m_signature == 2) {
    int x_green = pixy.ccc.blocks[i].m_x;
    int y_green = pixy.ccc.blocks[i].m_y;

    int disfrmleft = SharpIRleft.distance();
    int disfrmright = SharpIRright.distance();

    if (mpu.getAngleZ() < angle1 + 30 && mpu.getAngleZ() > angle1 - 30) {
      if (x_green < 50 && y_green > 40 && disfrmleft > 70) {  // y_green > 35
        // Car is EXTREMELY to the RIGHT of the block, turn LEFT ABRUPTLY then straighten
        turnLeft(154, 40);
        myservo.write(114);
        move_degree(true, 200, 1080);
        sig = 2;
      }
      if ((x_green < 145 && x_green > 55) && y_green > 50 && disfrmleft > 70) {
        // Car is to the RIGHT of the block, turn LEFT ABRUPTLY then straighten
        turnLeft(145, 30);
        myservo.write(114);
        move_degree(true, 200, 720);
        sig = 2;
      }
      if ((x_green < 180 && x_green > 150) && y_green > 50 && disfrmleft > 70) {
        // Block is right INFRONT of the Car, turn LEFT slightly then straighten
        turnLeft(140, 35);
        myservo.write(114);
        move_degree(true, 200, 630);
        sig = 2;
      }
      if ((x_green < 250 && x_green > 185) && y_green > 50) {
        // Car is to the LEFT of the block, turn left very slightly or go straight, test both
        turnLeft(135, 25);
        myservo.write(114);
        move_degree(true, 200, 180);
        sig = 2;
      }
    }
  }
}

void blueLineTurning() {
  blue = true;
  orange = false;

  int disfrmleft = SharpIRleft.distance();
  int disfrmright = SharpIRright.distance();

  if (disfrmleft < 35 && disfrmright > 120) {  // if car is too close to the inner (left) wall
    move_degree_gyro(true, 150, 2160);
    myservo.write(80);
    move_degree(false, 150, 1300);
    myservo.write(114);
    move_degree(false, 200, 360);

    // might not be needed

  } else if (disfrmleft > 40 && disfrmright > 120) {  // if car is still near the inner (left) wall but is still far from the outer wall
    move_degree_gyro(true, 150, 1620);                //1800
    myservo.write(80);
    move_degree(false, 150, 1260);
    myservo.write(114);
    move_degree(true, 200, 90);  // might not be needed

  } else if (disfrmright < 120 && disfrmright > 100) {  // if car is in between the inner (left) and outer wall
    move_degree_gyro(true, 150, 1300);                  // 1300
    myservo.write(80);
    move_degree(false, 150, 1080);
    myservo.write(114);
    move_degree(true, 200, 180);

  } else if (disfrmright < 100) {  // if car is very close to the outer wall
    move_degree_gyro(true, 150, 900);
    myservo.write(80);
    move_degree(false, 150, 1080);
    myservo.write(114);
    move_degree(true, 200, 360);

  } else {
    move_degree_gyro(true, 150, 2160);
    myservo.write(80);
    move_degree(false, 150, 1440);
    myservo.write(114);
    move_degree(true, 200, 90);  // might not be needed
  }
}

void orangeLineTurning() {
  blue = false;
  orange = true;

  int disfrmleft = SharpIRleft.distance();
  int disfrmright = SharpIRright.distance();

  if (disfrmright < 35 && disfrmleft > 120) {  // if car is too close to the inner (right) wall
    move_degree_gyro(true, 150, 2160);
    myservo.write(148);
    move_degree(false, 150, 1440);
    myservo.write(114);
    move_degree(true, 200, 90);  // might not be needed

  } else if (disfrmright > 40 && disfrmleft > 120) {  // if car is still near the inner (right) wall but is still far from the outer (left) wall
    move_degree_gyro(true, 150, 1620);
    myservo.write(148);
    move_degree(false, 150, 1260);
    myservo.write(114);
    move_degree(true, 200, 90);  // might not be needed

  } else if (disfrmleft < 120 && disfrmleft > 100) {  // if car is in between the inner (right) and outer (left) wall
    move_degree_gyro(true, 150, 1500);
    myservo.write(148);
    move_degree(false, 150, 1170);
    myservo.write(114);
    move_degree(true, 200, 180);

  } else if (disfrmleft < 100) {  // if car is very close to the outer (left) wall
    move_degree_gyro(true, 150, 700);
    myservo.write(148);
    move_degree(false, 150, 900);
    myservo.write(114);
    move_degree(true, 200, 360);

  } else {  // if none of the conditions are met
    move_degree_gyro(true, 150, 2160);
    myservo.write(148);
    move_degree(false, 150, 1440);
    myservo.write(114);
    move_degree(true, 200, 90);  // might not be needed
  }
}

void magDetectionBlue(int i) {
  if (pixy.ccc.blocks[i].m_signature == 3) {
    int x_mag = pixy.ccc.blocks[i].m_x;
    int y_mag = pixy.ccc.blocks[i].m_y;

    int disfrmleft = SharpIRleft.distance();
    int disfrmright = SharpIRright.distance();

    if (mpu.getAngleZ() < angle1 + 30 && mpu.getAngleZ() > angle1 - 30) {
      if ((x_mag < 270) && y_mag > 40) {
        turnLeft(140, 15);
        move_degree(true, 200, 90);
      }
    }
  }
}

void magDetectionParking(int i) {
  if (pixy.ccc.blocks[i].m_signature == 3) {
    int x_mag = pixy.ccc.blocks[i].m_x;
    int y_mag = pixy.ccc.blocks[i].m_y;

    int disfrmleft = SharpIRleft.distance();
    int disfrmright = SharpIRright.distance();

    if (mpu.getAngleZ() < angle1 + 30 && mpu.getAngleZ() > angle1 - 30) {
      if ((x_mag < 270) && y_mag > 50) {
        turnLeft(140, 15);
        move_degree(true, 200, 180);
        parking = true;
      } else if (x_mag > 290 && y_mag > 50) {
        turnRight(80, -15);
        move_degree(true, 200, 180);
        parking = true;
      }
    }
  }
}

//---------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------

void move_degree_gyro(bool dir, int pace, float dis) {
  left_tick = 0;
  left_flag = dir;
  digitalWrite(mlp, HIGH);
  analogWrite(mlp, pace);
  digitalWrite(mld, left_flag);
  int disfrmright = SharpIRright.distance();
  int disfrmleft = SharpIRleft.distance();

  while (abs(left_tick) < dis * 0.75) {
    mpu.update();
    int servoAngle = map((mpu.getAngleZ() - angle1), 80, -80, 74, 154);  // Adjust the mapping as necessary
    myservo.write(servoAngle);
    int disfrmleft = SharpIRleft.distance();
    int disfrmright = SharpIRright.distance();
  }
  left_tick = 0;
}

void move_degree(bool dir, int pace, float dis) {
  left_tick = 0;
  left_flag = dir;
  digitalWrite(mlp, HIGH);
  analogWrite(mlp, pace);
  digitalWrite(mld, left_flag);

  while (abs(left_tick) < dis * 0.75) {
    mpu.update();
    delayMicroseconds(1);
  }

  analogWrite(mlp, 255);
  digitalWrite(mld, !left_flag);
  delay(10);
  left_tick = 0;
}

void left_wheel_pulse() {
  if (left_flag) {
    left_tick++;
  } else {
    left_tick--;
  }
}
