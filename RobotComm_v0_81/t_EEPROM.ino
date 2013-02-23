//writes and reads to/from EEPROM
//to hold parameters and default values

#include <EEPROM.h>

//long EEPROMvalue, EEPROMaddress;

// standard defines

#define BATTERY_MONITOR_PIN 4  // note that this refers to arduino pin A4, since it is an analog read
#define MESSAGE_BATTERY_PERCENT mb
#define MESSAGE_EEPROM_VALUE mE
#define EEPROM_TEST_VALUE_10 2
#define EEPROM_TEST_VALUE_11 4
#define EEPROM_TEST_VALUE_12 8

// EEPROM written defines
#define TIMED_OUT 30  // this gets multiplied by 100 in actual use.  have to keep parameters < 256
#define DEFAULT_SPEED 220
#define BW_REDUCTION 50
#define DEFAULT_TILT_UP_SPEED 180
#define DEFAULT_TILT_DOWN_SPEED 135
#define DEFAULT_DEGREES 5
#define TICKS_PER_DEGREE_OF_TILT 30
#define DEFAULT_TURN_FOREVER_SPEED 150
#define TURN_TIME 500
#define MOVE_TIME 1000
#define TILT_TIME 500
#define NUDGE_TURN_TIME 200
#define NUDGE_MOVE_TIME 300
#define NUDGE_TILT_TIME 200
#define MIN_ACCEL_SPEED 120
#define MIN_DECEL_SPEED 60
#define DELTA_SPEED 60
#define ACCEL_DELAY 200
#define LEFT_MOTOR_BIAS 0
#define LEFT_MOTOR_BW_BIAS 23
#define RIGHT_MOTOR_BIAS 0
#define RIGHT_MOTOR_BW_BIAS 0
#define LEFT_MOTOR_STOP_DELAY 0
#define RIGHT_MOTOR_STOP_DELAY 0
#define MODIFY_MOTOR_BIASES 1  // this is a boolean


// next two are modified so that they stay under 255
#define CURRENT_LIMIT_TOP_MOTOR 20  // this gets multiplied by 100 in actual use
#define CURRENT_LIMIT_DRIVE_MOTORS 40 // this gets multiplied by 100 in actual use
#define CURRENT_LIMIT_ENABLED 1

#define ZERO_PERCENT_BATTERY_VOLTAGE 10.5
#define FULL_BATTERY_VOLTAGE 13.0
#define VOLTAGE_DIVIDER_RATIO 3.2

void setDefaults()
{
 if (EEPROM.read(10) == EEPROM_TEST_VALUE_10 &&
    EEPROM.read(11) == EEPROM_TEST_VALUE_11 &&
    EEPROM.read(12) == EEPROM_TEST_VALUE_12)  // test to see if we have written to the EEPROM
  {
    timed_out_default = EEPROM.read(101)* 100;  // needs to be mult by 100 to keep the original parameter < 256
    speed_default = EEPROM.read(102);
    bw_reduction_default = EEPROM.read(103);
    tilt_up_speed_default = EEPROM.read(104);
    tilt_down_speed_default = EEPROM.read(105);
    degrees_default = EEPROM.read(106);
    ticks_per_degree_of_tilt_default = EEPROM.read(107);
    turn_forever_speed_default = EEPROM.read(108);
    turn_time_default = EEPROM.read(109) * 10;  // needs to be mult by 10 to keep the original parameter < 256
    move_time_default = EEPROM.read(110) * 10;
    tilt_time_default = EEPROM.read(111) * 10;
    nudge_turn_time_default = EEPROM.read(201) * 10;
    nudge_move_time_default = EEPROM.read(202) * 10;
    nudge_tilt_time_default = EEPROM.read(203);
    min_accel_speed_default = EEPROM.read(112);
    min_decel_speed_default = EEPROM.read(113);
    delta_speed_default = EEPROM.read(114);
    accel_delay_default = EEPROM.read(115) * 10;  // needs to be mult by 10 to keep the original parameter < 256
    left_motor_bias_default = EEPROM.read(116);
    if (left_motor_bias_default > 128) left_motor_bias_default -= 256;  // negative numbers encoded as counting back from 256
    left_motor_bw_bias_default = EEPROM.read(117);
    left_motor_stop_delay_default = EEPROM.read(118) * 10; // needs to be mult by 10 to keep the original parameter < 256
    right_motor_bias_default = EEPROM.read(210);
    if (right_motor_bias_default > 128) right_motor_bias_default -= 256;
    right_motor_bw_bias_default = EEPROM.read(211);
    right_motor_stop_delay_default = EEPROM.read(212) * 10;  // needs to be mult by 10 to keep the original parameter < 256
    current_limit_top_motor_default = EEPROM.read(119)*100;     // multiply to keep the eeprom parameter < 255
    current_limit_drive_motors_default = EEPROM.read(120)*100; // multiply to keep the eeprom parameter < 255
    current_limit_enabled_default = EEPROM.read(204);
    encoder_ticks_per_cm_default = EEPROM.read(121);
    zero_percent_battery_voltage_default = ((double) EEPROM.read(122)) + (( (double) EEPROM.read(123)) / 10.);
    full_battery_voltage_default = ((double) EEPROM.read(124)) + ( ((double) EEPROM.read(125)) / 10.);
    voltage_divider_ratio_default = ((double) EEPROM.read(126)) + ( ((double) EEPROM.read(127)) / 10.);  
    battery_monitor_pin_default = EEPROM.read(128);  
    modify_motor_biases_default = EEPROM.read(129);
  }
  else
  {
    timed_out_default =  TIMED_OUT * 100;
    speed_default = DEFAULT_SPEED;
    bw_reduction_default = BW_REDUCTION;
    tilt_up_speed_default = DEFAULT_TILT_UP_SPEED;
    tilt_down_speed_default = DEFAULT_TILT_DOWN_SPEED;
    degrees_default = DEFAULT_DEGREES;
    ticks_per_degree_of_tilt_default = TICKS_PER_DEGREE_OF_TILT;
    turn_forever_speed_default = DEFAULT_TURN_FOREVER_SPEED;
    turn_time_default = TURN_TIME;
    move_time_default = MOVE_TIME;
    tilt_time_default = TILT_TIME;
    nudge_turn_time_default = NUDGE_TURN_TIME;
    nudge_move_time_default = NUDGE_MOVE_TIME;
    nudge_tilt_time_default = NUDGE_TILT_TIME;
    min_accel_speed_default = MIN_ACCEL_SPEED;
    min_decel_speed_default = MIN_DECEL_SPEED;
    delta_speed_default = DELTA_SPEED;
    accel_delay_default = ACCEL_DELAY;
    left_motor_bias_default = LEFT_MOTOR_BIAS;
    left_motor_bw_bias_default = LEFT_MOTOR_BW_BIAS;
    left_motor_stop_delay_default = LEFT_MOTOR_STOP_DELAY;
    right_motor_bias_default = RIGHT_MOTOR_BIAS;
    right_motor_bw_bias_default = RIGHT_MOTOR_BW_BIAS;
    right_motor_stop_delay_default = RIGHT_MOTOR_STOP_DELAY;    
    current_limit_top_motor_default = CURRENT_LIMIT_TOP_MOTOR * 100; // multiply to keep the eeprom parameter < 255
    current_limit_drive_motors_default = CURRENT_LIMIT_DRIVE_MOTORS * 100; // multiply to keep the eeprom parameter < 255
    current_limit_enabled_default = CURRENT_LIMIT_ENABLED;
    battery_monitor_pin_default = BATTERY_MONITOR_PIN;
    zero_percent_battery_voltage_default = ZERO_PERCENT_BATTERY_VOLTAGE;
    full_battery_voltage_default = FULL_BATTERY_VOLTAGE;
    voltage_divider_ratio_default = VOLTAGE_DIVIDER_RATIO;
    battery_monitor_pin_default = BATTERY_MONITOR_PIN; 
    modify_motor_biases_default = MODIFY_MOTOR_BIASES;
  }
}

void writeToEEPROM(int address, byte value)
{
  if (address > 4095 || address < 0) return;
  EEPROM.write(address, value);
}

int readFromEEPROM(int address)
{
  if (address > 4095 || address < 0) return -1;
  return EEPROM.read(address);
}

void readBTaddress()
{
  char buffer[18];
  for (int i= 0; i < 17; i++)
  {
    buffer[i] = EEPROM.read(300 + i);
  }
  //SERIAL_PORT.print("Bluetooth address is: ");
  //for (int i=0; i < 17; i++) SERIAL_PORT.print(buffer[i]);
  //SERIAL_PORT.println();
}

/*
long getEEPROMaddress()
{
  return EEPROMaddress;
}

void setEEPROMaddress(long value)
{
  EEPROMaddress = value;
}
*/
