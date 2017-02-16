#include <math.h>
#include <SPI.h>
#include <MySensor.h>  
#include <Wire.h> 
#include"AirQuality.h"
#include"Arduino.h"

#define CHILD_ID_AIQ 0
#define AIQ_SENSOR_ANALOG_PIN A4

#define MQ135_DEFAULTPPM 399 //default ppm of CO2 for calibration
#define MQ135_DEFAULTRO 68550 //default Ro for MQ135_DEFAULTPPM ppm of CO2
#define MQ135_SCALINGFACTOR 116.6020682 //CO2 gas value
#define MQ135_EXPONENT -2.769034857 //CO2 gas value
#define MQ135_MAXRSRO 2.428 //for CO2
#define MQ135_MINRSRO 0.358 //for CO2

unsigned long SLEEP_TIME = 3; // Sleep time between reads (in seconds)
//VARIABLES
float mq135_ro = 10000.0;    // this has to be tuned 10K Ohm
int val = 0;                 // variable to store the value coming from the sensor
float valAIQ =0.0;
float lastAIQ =0.0;

MyMessage msg(CHILD_ID_AIQ, V_LEVEL);

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
float           LPGCurve[3]  =  {2.3,0.21,-0.47};   //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent"
                                                    //to the original curve. 
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.21), point2: (lg10000, -0.59) 
float           COCurve[3]  =  {2.3,0.72,-0.34};    //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent" 
                                                    //to the original curve.
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.72), point2: (lg10000,  0.15) 
float           SmokeCurve[3] ={2.3,0.53,-0.44};    //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent" 
                                                    //to the original curve.
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.53), point2: (lg10000,  -0.22)                                                     
float           Ro           =  10;                 //Ro is initialized to 10 kilo ohms
 

#define Version     10          // version, 1.0 or 1.1, which depands on your board you use as it is
const int pinO2 =   A0;       // Connect Grove - Gas Sensor(O2) to A0
const int airQ =    A1;
const int MQ7   =   A3; 
const int MQ135 =   A4;
const int light =   A5;

#if Version==11
const int AMP   = 121;
#elif Version==10
const int AMP   = 201;
#endif
 
const float K_O2    = 7.43;
AirQuality airqualitysensor;
int current_quality =-1;
 
void setup()
{
    Serial.begin(9600);         //Start the Serial connection
    Serial.println("CLEARDATA");
    Serial.println("LABEL, Time, O2 Concentration, Air Quality, CO2 ppm, Illuminance, LPG ppm, CO ppm, Smoke ppm");
    Serial.println("RESETTIMER");
    airqualitysensor.init(14);
    Serial.print("Calibrating...\n");                
    Ro = MQCalibration(MQ_PIN);                       //Calibrating the sensor. Please make sure the sensor is in clean air 
                                                    //when you perform the calibration    
    Serial.print("Calibration is done...\n"); 
    Serial.print("Ro=");
    Serial.print(Ro);
    Serial.print("kohm");
    Serial.print("\n");
}

    long mq135_getro(long resvalue, double ppm) {
      return (long)(resvalue * exp( log(MQ135_SCALINGFACTOR/ppm) / MQ135_EXPONENT ));
    }
    double mq135_getppm(long resvalue, long ro) {
      double ret = 0;
      double validinterval = 0;
      validinterval = resvalue/(double)ro;
      ret = (double)MQ135_SCALINGFACTOR * pow( ((double)resvalue/ro), MQ135_EXPONENT);
      return ret;
    }
void loop()
{
    float sensorValue;
    float sensorVoltage;
    float Value_O2;

    uint16_t valr = analogRead(AIQ_SENSOR_ANALOG_PIN);// Get AIQ value
    uint16_t val =  ((float)22000*(1023-valr)/valr); 
    //during clean air calibration, read the Ro value and replace MQ135_DEFAULTRO value with it, you can even deactivate following function call.
    mq135_ro = mq135_getro(val, MQ135_DEFAULTPPM);
    //convert to ppm (using default ro)
    valAIQ = mq135_getppm(val, MQ135_DEFAULTRO);
    sensorValue   = analogRead(A0);
    sensorVoltage =(sensorValue/1024.0)*5.0;
    sensorVoltage = sensorVoltage/(float)AMP*10000.0;
    Value_O2 = sensorVoltage/K_O2;

    Serial.print("DATA,TIME");
    Serial.print(Value_O2,1);
    Serial.print(valAIQ/14);
    Serial.print(analogRead(light));
    Serial.print(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_LPG) );
    Serial.print(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_CO) );
    Serial.print(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_SMOKE) );
    delay(3000);
    current_quality=airqualitysensor.slope();
}
ISR(TIMER1_OVF_vect)
{
  if(airqualitysensor.counter==61)//set 2 seconds as a detected duty
  {

      airqualitysensor.last_vol=airqualitysensor.first_vol;
      airqualitysensor.first_vol=analogRead(A0);
      airqualitysensor.counter=0;
      airqualitysensor.timer_index=1;
      PORTB=PORTB^0x20;
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
  return ( ((float)RL_VALUE*(1023-raw_adc)/raw_adc));
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
  float val=0;
 
  for (i=0;i<CALIBARAION_SAMPLE_TIMES;i++) {            //take multiple samples
    val += MQResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val/CALIBARAION_SAMPLE_TIMES;                   //calculate the average value
 
  val = val/RO_CLEAN_AIR_FACTOR;                        //divided by RO_CLEAN_AIR_FACTOR yields the Ro 
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
  float rs=0;
 
  for (i=0;i<READ_SAMPLE_TIMES;i++) {
    rs += MQResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }
 
  rs = rs/READ_SAMPLE_TIMES;
 
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
     return MQGetPercentage(rs_ro_ratio,LPGCurve);
  } else if ( gas_id == GAS_CO ) {
     return MQGetPercentage(rs_ro_ratio,COCurve);
  } else if ( gas_id == GAS_SMOKE ) {
     return MQGetPercentage(rs_ro_ratio,SmokeCurve);
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
  return (pow(10,( ((log(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}

int  MQGetPercentage(float rs_ro_ratio, float ro, float *pcurve)
{
  return (double)(pcurve[0] * pow(((double)rs_ro_ratio/ro), pcurve[1]));
} 
