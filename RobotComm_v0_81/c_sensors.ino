// sensors
#include <Wire.h> // for I2C


int battery_monitor_pin_default;
int encoder_ticks_per_cm_default;

//battery

double zero_percent_battery_voltage_default,full_battery_voltage_default,voltage_divider_ratio_default;

int checkBattery()
{
  double voltage =  (double) ((analogRead(battery_monitor_pin_default) / 1023.) * 5.0 ) * voltage_divider_ratio_default;
  double batteryRange = full_battery_voltage_default - zero_percent_battery_voltage_default;
  int batteryPercent =  (int) ( 100. * ( ( voltage - zero_percent_battery_voltage_default) / batteryRange)); // returns percentage
  if (batteryPercent > 99) batteryPercent = 100;
  if (batteryPercent < 0) batteryPercent = 0;
  //batteryPercent = analogRead(BATTERY_MONITOR_PIN);  // for testing 
  return batteryPercent;
} 


