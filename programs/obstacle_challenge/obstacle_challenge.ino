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

// Pixy Setup
Pixy2 pixy;

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
  int servoAngle = map((mpu.getAngleZ() - angle1), 80, -80, 74, 154);  // Adjust the mapping as necessary
  myservo.write(servoAngle);
  analogWrite(mlp, 170);
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

  if (lux < 6 && r < 15 && blue) {  // if Blue Line is detected
    blue = true;
    orange = false;

    if (disfrmleft < 35 && disfrmright > 120) {  // if car is too close to the inner (left) wall, then turn and go toward the outer wall for more rotations
      // myservo.write(89);
      // move_degree(true, 150, 1260);
      // turnLeft(139, -2);

      move_degree_gyro(true, 200, 2160, angle1);
      myservo.write(80);
      move_degree(false, 200, 1440);
      myservo.write(114);
      delay(100);

    } else if (disfrmright > 120) {  // if car isn't right next to the inner (left) wall but is still far from the outer wall
      // myservo.write(89);
      // move_degree(true, 150, 900);
      // turnLeft(139, -2);

      move_degree_gyro(true, 200, 1800, angle1);
      myservo.write(80);
      move_degree(false, 200, 1260);
      myservo.write(114);
      delay(100);

    } else if (disfrmright < 100) {  // if car is very close to the outer wall, it will just turn
      // turnLeft(139, -2);

      move_degree_gyro(true, 200, 900, angle1);
      myservo.write(80);
      move_degree(false, 200, 1080);
      myservo.write(114);
      move_degree(true, 200, 360);

    } else {  // if car is not too close to the outer wall, but it needs to be farther away from the outer wall

      myservo.write(89);
      // move_degree(true, 150, 180);
      turnLeft(139, -2);
    }
    angle1 += 90;
    counter += 1;
  }

  int i;
  pixy.ccc.getBlocks();

  for (i = 0; i < pixy.ccc.numBlocks; i++) {
    // IF RED IS DETECTED
    redDetection(i);
    // IF GREEN IS DETECTED
    greenDetection(i);
  }

  // if (lux < 10 && r > 20 && orange) {  // if Orange Line is Detected
  //   blue = false;
  //   orange = true;
  // }

  if (blue) {
    mpu.update();
    // if (mpu.getAngleZ() > (angle1 + 30) || mpu.getAngleZ() < (angle1 - 30)) {
    disfrmleft = SharpIRleft.distance();
    disfrmright = SharpIRright.distance();
    if (disfrmright < 40 && disfrmleft > 120) {
      myservo.write(130);  // turn left
      delay(5);
    } else if (disfrmleft < 30 && disfrmright > 120) {
      myservo.write(98);
      delay(5);
    } else {
      delay(1);
    }
    // }
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
    analogWrite(mlp, 100);
    digitalWrite(mld, true);
  }
}

void turnLeft(int left, int gyroAngle) {
  myservo.write(left);
  while (mpu.getAngleZ() < angle1 + gyroAngle) {
    mpu.update();
    analogWrite(mlp, 100);
    digitalWrite(mld, true);
  }
}

void redDetection(int i) {
  if (pixy.ccc.blocks[i].m_signature == 1) {
    int x_red = pixy.ccc.blocks[i].m_x;
    int y_red = pixy.ccc.blocks[i].m_y;

    if (x_red > 250 && y_red > 35) {
      // Car is EXTREMELY to the LEFT of the block, turn RIGHT ABRUPTLY then straighten
      turnRight(74, -40);
      myservo.write(114);
      move_degree(true, 150, 750);
      // turnLeft(144, -10);
    }

    if ((x_red < 245 && x_red > 175) && y_red > 35) {
      // Car is to the LEFT of the block, turn RIGHT then straighten
      turnRight(74, -30);
      myservo.write(114);
      move_degree(true, 150, 400);
      // turnLeft(134, -15);
    }

    if ((x_red < 170 && x_red > 140) && y_red > 35) {
      // Block is right INFRONT of the Car, turn RIGHT slightly then straighten
      turnRight(90, -35);
      myservo.write(114);
      move_degree(true, 150, 360);
      // turnLeft(134, -15);
    }

    if ((x_red < 140 && x_red > 80) && y_red > 30) {
      // Car is to the RIGHT of the block, turn RIGHT very slightly (because sensor values are not going below 100 for x) or go straight, test both
      turnRight(94, -15);
      // turnLeft(124, -10);
    }
  }
}

void greenDetection(int i) {
  if (pixy.ccc.blocks[i].m_signature == 2) {
    int x_green = pixy.ccc.blocks[i].m_x;
    int y_green = pixy.ccc.blocks[i].m_y;

    int disfrmleft = SharpIRleft.distance();
    int disfrmright = SharpIRright.distance();

    if (x_green < 50 && y_green > 35 && disfrmleft > 70) {
      // Car is EXTREMELY to the RIGHT of the block, turn LEFT ABRUPTLY then straighten
      turnLeft(154, 30);
      myservo.write(114);
      move_degree(true, 150, 750);
      // turnRight(64, 10);
    }

    if ((x_green < 145 && x_green > 55) && y_green > 35 && disfrmleft > 70) {
      // Car is to the RIGHT of the block, turn LEFT ABRUPTLY then straighten
      turnLeft(154, 25);
      myservo.write(114);
      move_degree(true, 150, 400);
      // turnRight(94, 5);
    }

    if ((x_green < 180 && x_green > 150) && y_green > 35 && disfrmleft > 70) {
      // Block is right INFRONT of the Car, turn LEFT slightly then straighten
      turnLeft(140, 25);
      myservo.write(114);
      move_degree(true, 150, 250);
      // turnRight(94, 5);
    }

    if ((x_green > 180 && x_green < 240) && y_green > 30) {  //&& (y_green < 85 && y_green > 75
      // Car is to the LEFT of the block, turn left very slightly or go straight, test both
      turnLeft(135, 15);
      // turnRight(104, 10);
    }
  }
}

//---------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------

void move_degree_gyro(bool dir, int pace, float dis, int angle) {
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
  }
  analogWrite(mlp, 255);
  digitalWrite(mld, !left_flag);
  delay(10);
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
