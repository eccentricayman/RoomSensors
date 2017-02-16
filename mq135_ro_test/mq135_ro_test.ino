#include <math.h>
#include <SPI.h>
#include <MySensor.h>
#include <Wire.h>
#include"AirQuality.h"
#include"Arduino.h"
#include "MQ135.h"
#include <OneWire.h>
MQ135 gasSensor = MQ135(4); 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  

}

void loop() {
  // put your main code here, to run repeatedly:
  float rzeroMQ135 = gasSensor.getRZero();
  Serial.println(rzeroMQ135);
  delay(1);
}
