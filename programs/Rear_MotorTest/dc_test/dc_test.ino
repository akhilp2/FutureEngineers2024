#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

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
  // put your setup code here, to run once:
Serial.begin(115200);
pinMode(mlp, OUTPUT); 
pinMode(mld, OUTPUT); 
attachInterrupt(digitalPinToInterrupt(mlf), left_wheel_pulse, RISING);
display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
display.clearDisplay();
display.setTextSize(1);
display.setTextColor(WHITE);
}

void loop() {
move_degree_left(false, 100, 355);
delay(1000);
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