#include "MQ135.h"

MQ135 gasSensor = MQ135(4);
void setup(){
  Serial.begin(9600);
}
void loop(){
  float rzero = gasSensor.getRZero();
  Serial.println(rzero);
  float ppm = gasSensor.getPPM();
  Serial.println(ppm);
  delay(3000);
}

