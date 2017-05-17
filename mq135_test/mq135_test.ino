#include "MQ135.h"

MQ135 gasSensor = MQ135(4);
void setup(){
  Serial.begin(9600);
}
void loop(){
  float rzero = gasSensor.getRZero();
  Serial.print("gas: ");
  Serial.print(rzero);
  Serial.println("");
  float ppm = gasSensor.getPPM();
  Serial.print("pressure: ");
  Serial.print(ppm);
  Serial.println(" ppm");
  delay(1000);
}

