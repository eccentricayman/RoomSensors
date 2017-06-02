/*Credits:
   MQ135: https://hackaday.io/project/3475-sniffing-trinket/log/12363-mq135-arduino-library
   Temperature sensors: https://github.com/barnybug
   Grove Air quality: http://www.seeedstudio.com/wiki/Grove_-_Air_Quality_Sensor
   Grove O2: http://www.seeedstudio.com/wiki/Grove_-_Gas_Sensor(O%E2%82%82)
   MQ7: https://github.com/iot-playground
*/
#include <math.h>
#include <SPI.h>
#include <MySensor.h>
#include <Wire.h>
#include"AirQuality.h"
#include"Arduino.h"
#include "MQ135.h"
#include <OneWire.h>
OneWire  ds(2);  //Black Tape Air Sensor
OneWire  ad(7);   //Yellow Tape Water Sensor
MQ135 gasSensor = MQ135(4);

/************************Hardware Related Macros************************************/
#define         MQ_PIN                       (2)     //define which analog input channel you are going to use
#define         RL_VALUE                     (5)     //define the load resistance on the board, in kilo ohms
#define         RO_CLEAN_AIR_FACTOR          (9.83)  //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,
//which is derived from the chart in datasheet

/***********************Software Related Macros************************************/
#define         CALIBARAION_SAMPLE_TIMES     (50)    //define how many samples you are going to take in the calibration phase
#define         CALIBRATION_SAMPLE_INTERVAL  (500)   //define the time interal(in milisecond) between each samples in the
//cablibration phase
#define         READ_SAMPLE_INTERVAL         (50)    //define how many samples you are going to take in normal operation
#define         READ_SAMPLE_TIMES            (5)     //define the time interal(in milisecond) between each samples in 
//normal operation

/**********************Application Related Macros**********************************/
#define         GAS_LPG                      (0)
#define         GAS_CO                       (1)
#define         GAS_SMOKE                    (2)

/*****************************Globals***********************************************/
float           LPGCurve[3]  =  {2.3, 0.21, -0.47}; //two points are taken from the curve.
//with these two points, a line is formed which is "approximately equivalent"
//to the original curve.
//data format:{ x, y, slope}; point1: (lg200, 0.21), point2: (lg10000, -0.59)
float           COCurve[3]  =  {2.3, 0.72, -0.34};  //two points are taken from the curve.
//with these two points, a line is formed which is "approximately equivalent"
//to the original curve.
//data format:{ x, y, slope}; point1: (lg200, 0.72), point2: (lg10000,  0.15)
float           SmokeCurve[3] = {2.3, 0.53, -0.44}; //two points are taken from the curve.
//with these two points, a line is formed which is "approximately equivalent"
//to the original curve.
//data format:{ x, y, slope}; point1: (lg200, 0.53), point2: (lg10000,  -0.22)
float           Ro           =  10;                 //Ro is initialized to 10 kilo ohms


#define Version     10          // version, 1.0 or 1.1, which depands on your board you use as it is
const int pinO2 =   A2;       // Connect Grove - Gas Sensor(O2) to A0
const int airQ =    A1;
const int MQ7   =   A3;
const int light =   A5;

#if Version==11
const int AMP   = 121;
#elif Version==10
const int AMP   = 201;
#endif

const float K_O2    = 7.43;
AirQuality airqualitysensor;
int current_quality = -1;
/*======================================================================================
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 *================================================================================*/
void setup()
{
  Serial.begin(9600);         //Start the Serial connection
  //Serial.println("CLEARDATA");
  //Serial.println("LABEL, Time, Time Since Start, Date, Temperature of Air in Celsius, Temperature of Air in Fahrenheit, Temperature of Water in Celsius, Temperature of Water in Fahrenheit, Percent O2 Concentration, CO2 ppm, Illuminance, LPG, CO, Smoke");
  //Serial.println("RESETTIMER");
  airqualitysensor.init(14);
  //Serial.print("Calibrating...\n");
  Ro = MQCalibration(MQ_PIN);                       //Calibrating the sensor. Please make sure the sensor is in clean air
  //when you perform the calibration
  //Serial.print("Calibration is done...\n");
  //Serial.print("Ro=");
  //Serial.print(Ro);
  //Serial.print("kohm");
  //Serial.print("\n");
}

void loop()
{
  float sensorValue;
  float sensorVoltage;
  float Value_O2;

  sensorValue   = analogRead(A0);
  sensorVoltage = (sensorValue / 1024.0) * 5.0;
  sensorVoltage = sensorVoltage / (float)AMP * 10000.0;
  Value_O2 = sensorVoltage / K_O2;
  current_quality = airqualitysensor.slope();
  //MQ135
  //float rzeroMQ135 = gasSensor.getRZero();
  float ppmMQ135 = gasSensor.getPPM();
  //Temperature Sensors
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
  float celsiusW, fahrenheitW;

  if ( !ad.search(addr)) {
    //Serial.println("No more addresses.");
    //Serial.println();
    ad.reset_search();
    delay(250);
    return;
  }

  //Serial.print("ROM =");
  for ( i = 0; i < 8; i++) {
    //Serial.write(' ');
    //Serial.print(addr[i], HEX);
  }

  if (OneWire::crc8(addr, 7) != addr[7]) {
    //Serial.println("CRC is not valid!");
    return;
  }
  //Serial.println();

  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      //Serial.println("  Chip = ad18S20");  // or old ad1820
      type_s = 1;
      break;
    case 0x28:
      //Serial.println("  Chip = ad18B20");
      type_s = 0;
      break;
    case 0x22:
      //Serial.println("  Chip = ad1822");
      type_s = 0;
      break;
    default:
      //Serial.println("Device is not a ad18x20 family device.");
      return;
  }

  ad.reset();
  ad.select(addr);
  ad.write(0x44, 1);        // start conversion, with parasite power on at the end

  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ad.depower() here, but the reset will take care of it.

  present = ad.reset();
  ad.select(addr);
  ad.write(0xBE);         // Read Scratchpad

  //Serial.print("  Data = ");
  //Serial.print(present, HEX);
  //Serial.print(" ");
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ad.read();
    //Serial.print(data[i], HEX);
    //Serial.print(" ");
  }
  //Serial.print(" CRC=");
  //Serial.print(OneWire::crc8(data, 8), HEX);
  //Serial.println();

  // convert the data to actual temperature

  unsigned int raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // count remain gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    if (cfg == 0x00) raw = raw << 3;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw << 2; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw << 1; // 11 bit res, 375 ms
    // default is 12 bit resolution, 750 ms conversion time
  }
  celsiusW = (float)raw / 16.0;
  fahrenheitW = celsiusW * 1.8 + 32.0;
  //Serial.print("  Temperature = ");
  //Serial.print(celsiusW);
  //Serial.print(" Celsius, ");
  //Serial.print(fahrenheitW);
  //Serial.println(" Fahrenheit");
  //==============================================
  float celsiusA, fahrenheitA;

  if ( !ds.search(addr)) {
    //Serial.println("No more addresses.");
    //Serial.println();
    ds.reset_search();
    delay(250);
    return;
  }
  //check serial.write
  //Serial.print("ROM =");
  for ( i = 0; i < 8; i++) {
    //Serial.write(' ');
    //Serial.print(addr[i], HEX);
  }

  if (OneWire::crc8(addr, 7) != addr[7]) {
    //Serial.println("CRC is not valid!");
    return;
  }
  //Serial.println();

  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      //Serial.println("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
      //Serial.println("  Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
      //Serial.println("  Chip = DS1822");
      type_s = 0;
      break;
    default:
      //Serial.println("Device is not a DS18x20 family device.");
      return;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end

  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.

  present = ds.reset();
  ds.select(addr);
  ds.write(0xBE);         // Read Scratchpad

  //Serial.print("  Data = ");
  //Serial.print(present,HEX);
  //Serial.print(" ");
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
    //Serial.print(data[i], HEX);
    //Serial.print(" ");
  }
  //Serial.print(" CRC=");
  //Serial.print(OneWire::crc8(data, 8), HEX);
  //Serial.println();

  // convert the data to actual temperature

  unsigned int raw2 = (data [1] << 8) | data[0];
  if (type_s) {
    raw2 = raw2 << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // count remain gives full 12 bit resolution
      raw2 = (raw2 & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    if (cfg == 0x00) raw2 = raw2 << 3;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw2 = raw2 << 2; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw2 = raw2 << 1; // 11 bit res, 375 ms
    // default is 12 bit resolution, 750 ms conversion time
  }
  celsiusA = (float)raw2 / 16.0;
  fahrenheitA = celsiusA * 1.8 + 32.0;
  //Serial.print("  Temperature = ");
  //Serial.print(celsiusA);
  //Serial.print(" Celsius, ");
  //Serial.print(fahrenheitA);
  //Serial.println(" Fahrenheit");
 
/*======================================================================================
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 *================================================================================*/

  //Serial.print("DATA,TIME,TIMER,");
  //Serial.print("DATE,");
  delay(500);
  //sensor 1
  Serial.print(celsiusA);
  Serial.print(",");
  Serial.print(fahrenheitA);
  Serial.print(",");
  //sensor 2
  Serial.print(celsiusW);
  Serial.print(",");
  Serial.print(fahrenheitW);
  Serial.print(",");
  //oxygen
  Serial.print(Value_O2, 1);
  Serial.print(",");
  //ppm
  Serial.print(ppmMQ135);
  Serial.print(",");
  //light sensor
  Serial.print(analogRead(light));
  Serial.print(",");
  //
  Serial.print(MQGetGasPercentage(MQRead(MQ_PIN) / Ro, GAS_LPG) );
  Serial.print(",");
  //carbon diox
  Serial.print(MQGetGasPercentage(MQRead(MQ_PIN) / Ro, GAS_CO) );
  Serial.print(",");
  //smoke
  Serial.print(MQGetGasPercentage(MQRead(MQ_PIN) / Ro, GAS_SMOKE) );
  //Serial.println();
  //Serial.println();
  Serial.println("\n");
  Serial.flush();
  delay(5000);
}

ISR(TIMER1_OVF_vect)
{
  if (airqualitysensor.counter == 61) //set 2 seconds as a detected duty
  {

    airqualitysensor.last_vol = airqualitysensor.first_vol;
    airqualitysensor.first_vol = analogRead(A0);
    airqualitysensor.counter = 0;
    airqualitysensor.timer_index = 1;
    PORTB = PORTB ^ 0x20;
  }
  else
  {
    airqualitysensor.counter++;
  }
}

/****************** MQResistanceCalculation ****************************************
  Input:   raw_adc - raw value read from adc, which represents the voltage
  Output:  the calculated sensor resistance
  Remarks: The sensor and the load resistor forms a voltage divider. Given the voltage
         across the load resistor and its resistance, the resistance of the sensor
         could be derived.
************************************************************************************/
float MQResistanceCalculation(int raw_adc)
{
  return ( ((float)RL_VALUE * (1023 - raw_adc) / raw_adc));
}

/***************************** MQCalibration ****************************************
  Input:   mq_pin - analog channel
  Output:  Ro of the sensor
  Remarks: This function assumes that the sensor is in clean air. It use
         MQResistanceCalculation to calculates the sensor resistance in clean air
         and then divides it with RO_CLEAN_AIR_FACTOR. RO_CLEAN_AIR_FACTOR is about
         10, which differs slightly between different sensors.
************************************************************************************/
float MQCalibration(int mq_pin)
{
  int i;
  float val = 0;

  for (i = 0; i < CALIBARAION_SAMPLE_TIMES; i++) {      //take multiple samples
    val += MQResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val / CALIBARAION_SAMPLE_TIMES;                 //calculate the average value

  val = val / RO_CLEAN_AIR_FACTOR;                      //divided by RO_CLEAN_AIR_FACTOR yields the Ro
  //according to the chart in the datasheet

  return val;
}
/*****************************  MQRead *********************************************
  Input:   mq_pin - analog channel
  Output:  Rs of the sensor
  Remarks: This function use MQResistanceCalculation to caculate the sensor resistenc (Rs).
         The Rs changes as the sensor is in the different consentration of the target
         gas. The sample times and the time interval between samples could be configured
         by changing the definition of the macros.
************************************************************************************/
float MQRead(int mq_pin)
{
  int i;
  float rs = 0;

  for (i = 0; i < READ_SAMPLE_TIMES; i++) {
    rs += MQResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }

  rs = rs / READ_SAMPLE_TIMES;

  return rs;
}

/*****************************  MQGetGasPercentage **********************************
  Input:   rs_ro_ratio - Rs divided by Ro
         gas_id      - target gas type
  Output:  ppm of the target gas
  Remarks: This function passes different curves to the MQGetPercentage function which
         calculates the ppm (parts per million) of the target gas.
************************************************************************************/
int MQGetGasPercentage(float rs_ro_ratio, int gas_id)
{
  if ( gas_id == GAS_LPG ) {
    return MQGetPercentage(rs_ro_ratio, LPGCurve);
  } else if ( gas_id == GAS_CO ) {
    return MQGetPercentage(rs_ro_ratio, COCurve);
  } else if ( gas_id == GAS_SMOKE ) {
    return MQGetPercentage(rs_ro_ratio, SmokeCurve);
  }

  return 0;
}

/*****************************  MQGetPercentage **********************************
  Input:   rs_ro_ratio - Rs divided by Ro
         pcurve      - pointer to the curve of the target gas
  Output:  ppm of the target gas
  Remarks: By using the slope and a point of the line. The x(logarithmic value of ppm)
         of the line could be derived if y(rs_ro_ratio) is provided. As it is a
         logarithmic coordinate, power of 10 is used to convert the result to non-logarithmic
         value.
************************************************************************************/
int  MQGetPercentage(float rs_ro_ratio, float *pcurve)
{
  return (pow(10, ( ((log(rs_ro_ratio) - pcurve[1]) / pcurve[2]) + pcurve[0])));
}

int  MQGetPercentage(float rs_ro_ratio, float ro, float *pcurve)
{
  return (double)(pcurve[0] * pow(((double)rs_ro_ratio / ro), pcurve[1]));
}
