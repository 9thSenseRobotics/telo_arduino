#include "Arduino.h"
#include "irSensor.h"
#include "math.h"

irSensor::irSensor(int sensorPin, float sensorHeight, float sensorAngle)
{
  _sensorPin = sensorPin;
  _height    = sensorHeight;
  _theta     = sensorAngle;
}

float irSensor::senseRawData()
{
  for (int i = 0; i<100; i++)
    {
      _sensorPing    = analogRead(_sensorPin);
      _sensorRawData = (_sensorPing + _sensorRawData)/2;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  
    }
  
  return _sensorRawData; 
}

float irSensor::senseVoltage()
{
    _voltage = senseRawData()*5/1024;
  
    return _voltage;
}

float irSensor::senseDistance()
{
	_oldDistance=_distance;
  _distance = pow(senseVoltage(), -2.0202)/.0079566725761;
  
  return _distance;
}

float irSensor::senseSlope()
{
  _thetaPrime=asin((_height*sin(_theta)/senseDistance()));
  
  return _thetaPrime;
}


boolean irSensor:: senseDanger()
{
	_danger        = false;
	senseSlope();
	senseDistance();
   if((_oldDistance - _distance)> 5)
   {
     _danger = true;
   }
   else if (_distance < 15)
   {
     _danger = true;
   }
   else if (_thetaPrime>(3.14/2)||_thetaPrime<(3.14/20))
   {
	 _danger = true;
   }

    return _danger;
}

void irSensor:: printData()
{
	senseDanger();
	Serial.print("Raw data: ");
	Serial.println(_sensorRawData);
	Serial.print("Voltage: ");
	Serial.println(_voltage);
	Serial.print("Distance: ");
	Serial.println(_distance);
	Serial.print("Ground Angle: ");
	Serial.println(_thetaPrime);
	Serial.print("Are we in danger?: ");
	Serial.println(_danger);
	Serial.println("----------------------");
}