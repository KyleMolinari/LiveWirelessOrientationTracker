#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "BluetoothSerial.h"

Adafruit_BNO055 bno1 = Adafruit_BNO055(55, 0x28); //ADR pin to ground

BluetoothSerial SerialBT;

int freq = 10;
int deltaT = 1000/freq;
int zerobutton = 37;
int reset;

void setup(void) 
{
  //Serial.begin(9600);
  SerialBT.begin("ESP32 position"); 
  /* Initialise the sensor */
  if(!bno1.begin()) 
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    SerialBT.print("Ooops, no BNO055s detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(10);
  
  pinMode(zerobutton, INPUT);
  
  bno1.setExtCrystalUse(true);
}

void loop(void) 
{ 
  reset = 0;
  if(digitalRead(zerobutton) == HIGH)
  {
    reset = 1;
  }
  imu::Vector<3> accel = bno1.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Quaternion quat = bno1.getQuat();
  
  SerialBT.print(quat.w());
  SerialBT.print(",");
  SerialBT.print(quat.x());
  SerialBT.print(",");
  SerialBT.print(quat.y());
  SerialBT.print(",");
  SerialBT.print(quat.z());
  SerialBT.print(",");
  SerialBT.print(accel.x());
  SerialBT.print(",");
  SerialBT.print(accel.y());
  SerialBT.print(",");
  SerialBT.print(accel.z());
  SerialBT.print(",");
  SerialBT.print(reset);
  SerialBT.println("");
     
  delay(deltaT);
}
