#ifndef irSensor_h
#define irSensor_h

#include "Arduino.h"
#include "math.h"

class irSensor
{
public:
  irSensor(int sensorPin, float sensorHeight, float sensorAngle);
  float senseRawData();
  float senseVoltage();
  float senseDistance();
  float senseSlope();
  boolean senseDanger();
  void printData();
 
private:
  int   _sensorPin;
  float _theta;
  float _thetaPrime;
  float _distance;
  float _sensorPing;
  float _sensorRawData;
  float _voltage;
  float _oldDistance;
  float _height;
  bool  _danger;
};

#endif

