#include <SharpIR.h>

#define ir0 A0
#define ir1 A1
#define ir2 A2

#define model0 20150
#define model1 20150
#define model2 20150


// ir: the pin where your sensor is attached
// model: an int that determines your sensor:  1080 for GP2Y0A21Y
//                                            20150 for GP2Y0A02Y
//                                            (working distance range according to the datasheets)

SharpIR SharpIR0(ir0, model0);
SharpIR SharpIR1(ir1, model1);
SharpIR SharpIR2(ir2, model2);



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
delay (500);

  unsigned long pepe1=millis();  // takes the time before the loop on the library begins

  int dis0 = SharpIR0.distance();  // this returns the distance to the object you're measuring
  int dis1 = SharpIR1.distance();  // this returns the distance to the object you're measuring
  int dis2 = SharpIR2.distance();  // this returns the distance to the object you're measuring


  Serial.print("A0: ");  // returns it to the serial monitor
  Serial.print(dis0);
  
  Serial.print("     A1: ");  // returns it to the serial monitor
  Serial.print(dis1);

  Serial.print("          A2: ");  // returns it to the serial monitor
  Serial.println(dis2);
    

}