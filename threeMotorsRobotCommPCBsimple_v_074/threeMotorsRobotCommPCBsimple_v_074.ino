
// accepts serial port inputs and responds with moves and tilts
// requires a Mega to work, not an Uno or Nano, due to the need for two serial inputs
// version 0.74:
// EEPROM reading and writing work nicely
// added some default parameters
// version 0.73:
// Uses coast instead of Stop when commanded move or turn speed = 0
// Sends correct lead character ('e') when echoing command
// version 0.72:
// revised current limits and also added boolean for whether or not to use current limits, default is true
// version 0.71:
// even simpler routines now, just turn and move
// fixed but in turn that had us turning only to the right
//
// version 0.70 changes:
// EEPROM writes and reads are now used for all dynamic parameters
// move, turn, and tilt routines simplified

// version 0.63 changes:
// Bias values to compensate for turning are now written to eeprom, so they can be changed remotely
// commands added to do that.
// Bug fixed in MoveForward to get sign of mySpeed correct


// wiring:
// motor A is the left motor, motor B the right.
// hook them both up so that the outside wire from the motor goes into the outside terminal
// that corresponds to M1A for the right motor, M2B for the left motor
// see the comments in threeMotorsDriverPCB.cpp for details.

// for the battery monitor, hook it up with the + side soldered to Vout on the motor driver (+12)
// the negative side to ground and the middle to A4.  With the battery attached, 
// measure the voltage on Vout and divide it by the voltage on A4
// and enter that as the parameter VOLTAGE_DIVIDER_RATIO (assuming 22K and 10K resistors, the value should be 3.2)
// Also measure the fully charged battery value and enter that with the discharged value into the parameters
// FULL_BATTERY_VOLTAGE and ZERO_PERCENT_BATTERY_VOLTAGE

// command form is a letter for direction:
// f,b,r,d = move forward, move backward, right turn, left turn.
// upper case means move forever
// t,g move forward or back a little
// followed by the speed to move, 0 to 255
// followed by a character to indicate that the input is complete, in our case that character is #
// for example, move forward with a speed of 200 is
// f200#
// to turn right at full speed is
// r255#
// if you wish to use the default speed (a defined value), then just the letters is sufficient:
// r#
// 
// for tilting
// u, n = move up, down
// followed by the number of steps to take, for example, to move 2 steps up:
// U2#
// if there is no number, then the servo will just move by one step (TILT_DELTA)
// U#

// for the arduino mega, pin 47 corresponds to pin T5 on the ATMEL 2560
// ATMEL T0 = arduino mega 38
// ATMEL T1 = arduino mega NOT MAPPED uno pin 5
// ATMEL T3 = arduino mega NOT MAPPED
// ATMEL T4 = arduino mega NOT MAPPED
// ATMEL T5 = arduino mega 47

//The UNO has 3 Timers and 6 PWM output pins. The relation between timers and PWM outputs is:
//Pins 5 and 6: controlled by timer0
//Pins 9 and 10: controlled by timer1 and the servo library uses timer1, which is why 9 and 10 are not avail for PWM when you are using the servo library
//Pins 11 and 3: controlled by timer2

//On the Arduino Mega we have 6 timers and 15 PWM outputs:
//Pins 4 and 13: controlled by timer0
//Pins 11 and 12: controlled by timer1
//Pins 9 and10: controlled by timer2
//Pin 2, 3 and 5: controlled by timer 3
//Pin 6, 7 and 8: controlled by timer 4
//Pin 46, 45 and 44:: controlled by timer 5

#include <threeMotorsDriverPCB.h>
#include <EEPROM.h>

// standard defines
#define BATTERY_MONITOR_PIN A4
#define SERIAL_PORT Serial
#define SERIAL_PORT_BLUETOOTH Serial2
#define SERIAL_SPEED 115200
#define BLUETOOTH_SPEED 115200
#define INPUT_BUFFER_SIZE 256
#define COMMAND_END_CHARACTER '#'
#define COMM_CHECK_CHARACTER 'c'
#define COMMAND_ECHO_CHARACTER 'e'
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
#define DEFAULT_DEGREES 10
#define TICKS_PER_DEGREE_OF_TILT 30
#define DEFAULT_TURN_FOREVER_SPEED 220
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
#define LEFT_MOTOR_BIAS 10
#define LEFT_MOTOR_BW_BIAS 23
#define RIGHT_MOTOR_BIAS 0
#define RIGHT_MOTOR_BW_BIAS 0
#define LEFT_MOTOR_STOP_DELAY 0
#define RIGHT_MOTOR_STOP_DELAY 0

// next two are modified so that they stay under 255
#define CURRENT_LIMIT_TOP_MOTOR 20  // this gets multiplied by 100 in actual use
#define CURRENT_LIMIT_DRIVE_MOTORS 40 // this gets multiplied by 100 in actual use
#define CURRENT_LIMIT_ENABLED 1

#define ZERO_PERCENT_BATTERY_VOLTAGE 10.5
#define FULL_BATTERY_VOLTAGE 13.0
#define VOLTAGE_DIVIDER_RATIO 3.2

threeMotorsDriverPCB motorDriver;

char inputBuffer[INPUT_BUFFER_SIZE], charIn;
int inputLength, mySpeed;
bool Moving, brakesOn, exceededCurrentLimitC = false, enableEEPROMwrite = false;
long timeOutCheck;
float batteryRange;
int currentTopMotor, currentRightMotor, currentLeftMotor, current_limit_enabled_default;
int program_version;
int timed_out_default, speed_default, bw_reduction_default, tilt_up_speed_default;
int tilt_down_speed_default, degrees_default, ticks_per_degree_of_tilt_default;
int turn_forever_speed_default, move_time_default, turn_time_default, tilt_time_default;
int nudge_turn_time_default, nudge_move_time_default, nudge_tilt_time_default;
int min_accel_speed_default, min_decel_speed_default, delta_speed_default, accel_delay_default;
int left_motor_bias_default, left_motor_bw_bias_default, left_motor_stop_delay_default;
int right_motor_bias_default, right_motor_bw_bias_default, right_motor_stop_delay_default;
int current_limit_top_motor_default, current_limit_drive_motors_default;
int encoder_ticks_per_cm_default;
long EEPROMvalue, EEPROMaddress;
double zero_percent_battery_voltage_default,full_battery_voltage_default,voltage_divider_ratio_default;

void setDefaults()
{
 if (EEPROM.read(10) == EEPROM_TEST_VALUE_10 &&
    EEPROM.read(11) == EEPROM_TEST_VALUE_11 &&
    EEPROM.read(12) == EEPROM_TEST_VALUE_12)  // test to see if we have written to the EEPROM
  {
    timed_out_default = EEPROM.read(101)* 100;
    speed_default = EEPROM.read(102);
    bw_reduction_default = EEPROM.read(103);
    tilt_up_speed_default = EEPROM.read(104);
    tilt_down_speed_default = EEPROM.read(105);
    degrees_default = EEPROM.read(106);
    ticks_per_degree_of_tilt_default = EEPROM.read(107);
    turn_forever_speed_default = EEPROM.read(108);
    turn_time_default = EEPROM.read(109);
    move_time_default = EEPROM.read(110);
    tilt_time_default = EEPROM.read(111);
    nudge_turn_time_default = EEPROM.read(201);
    nudge_move_time_default = EEPROM.read(202);
    nudge_tilt_time_default = EEPROM.read(203);
    min_accel_speed_default = EEPROM.read(112);
    min_decel_speed_default = EEPROM.read(113);
    delta_speed_default = EEPROM.read(114);
    accel_delay_default = EEPROM.read(115);
    left_motor_bias_default = EEPROM.read(116);
    left_motor_bw_bias_default = EEPROM.read(117);
    left_motor_stop_delay_default = EEPROM.read(118);
    right_motor_bias_default = EEPROM.read(210);
    right_motor_bw_bias_default = EEPROM.read(211);
    right_motor_stop_delay_default = EEPROM.read(212);
    current_limit_top_motor_default = EEPROM.read(119)*100;     // multiply to keep the eeprom parameter < 255
    current_limit_drive_motors_default = EEPROM.read(120)*100; // multiply to keep the eeprom parameter < 255
    current_limit_enabled_default = EEPROM.read(204);
    encoder_ticks_per_cm_default = EEPROM.read(121);
    zero_percent_battery_voltage_default = ((double) EEPROM.read(122)) + (( (double) EEPROM.read(123)) / 10.);
    full_battery_voltage_default = ((double) EEPROM.read(124)) + ( ((double) EEPROM.read(125)) / 10.);
    voltage_divider_ratio_default = ((double) EEPROM.read(126)) + ( ((double) EEPROM.read(127)) / 10.);    
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
    //battery_monitor_pin_default = BATTERY_MONITOR_PIN;
    zero_percent_battery_voltage_default = ZERO_PERCENT_BATTERY_VOLTAGE;
    full_battery_voltage_default = FULL_BATTERY_VOLTAGE;
    voltage_divider_ratio_default = VOLTAGE_DIVIDER_RATIO;
  }
}

int checkBattery()
{
  float voltage =  (float) ((analogRead(BATTERY_MONITOR_PIN) / 1023.) * 5.0 ) * voltage_divider_ratio_default;
  int batteryPercent =  (int) ( 100. * ( ( voltage - zero_percent_battery_voltage_default) / batteryRange)); // returns percentage
  if (batteryPercent > 99) batteryPercent = 100;
  if (batteryPercent < 0) batteryPercent = 0;
  //batteryPercent = analogRead(BATTERY_MONITOR_PIN);  // for testing 
  return batteryPercent;
} 

bool checkForFault()
{
  bool result = false;
  if (!motorDriver.getStatusA())
  {
      motorDriver.setBrakesAB();
      SERIAL_PORT.println("Fault detected in left motor ");
      result = true;
  }
  if (!motorDriver.getStatusB())
  {
      motorDriver.setBrakesAB();
      SERIAL_PORT.println("Fault detected in right motor ");
      result = true;
  }
  
  if (!motorDriver.getStatusC())
  {
      motorDriver.setBrakesC();
      SERIAL_PORT.println("Fault detected in top motor ");
      result = true;
  }
  return result;
}

void coast()
{
  //SERIAL_PORT.println("coasting");
  motorDriver.setCoastAB();
  brakesOn = false;
}

void brakes()
{
  //SERIAL_PORT.println("braking");
  motorDriver.setBrakesAB();
  brakesOn = true;
}

void Stop()
{
  //SERIAL_PORT.println("stopping");
  motorDriver.setBrakesAB();
  brakesOn = true;
  // Moving = false;  // don't set this since tilt motor might still be moving and so we want to check TIME_OUT
}

void accelerate(int targetSpeed)
{
  int goSpeed = min_accel_speed_default;
  if (targetSpeed > 0)
  {
    while (goSpeed < targetSpeed - delta_speed_default)
    {
      motorDriver.setSpeedAB(goSpeed + left_motor_bias_default, goSpeed + right_motor_bias_default);
      //SERIAL_PORT.println("moving, speed = ");
      //SERIAL_PORT.println(goSpeed);
      goSpeed += delta_speed_default;
      delay(accel_delay_default);
    }
  }
  else
  {
    targetSpeed = -targetSpeed;
    while (goSpeed < targetSpeed - delta_speed_default)
    {
      motorDriver.setSpeedAB(-goSpeed - left_motor_bias_default,-goSpeed - right_motor_bias_default);
      //SERIAL_PORT.println("moving, speed = ");
      //SERIAL_PORT.println(-goSpeed);
      goSpeed += delta_speed_default;
      delay(accel_delay_default);
    }
  }
}

void decelerate(int initialSpeed)
{
  int goSpeed;
  if (initialSpeed > 0)
  {
    goSpeed = initialSpeed - delta_speed_default;
    while (goSpeed > min_decel_speed_default + delta_speed_default)
    {
      motorDriver.setSpeedAB(goSpeed + left_motor_bias_default, goSpeed + right_motor_bias_default);
      //SERIAL_PORT.println("moving, speed = ");
      //SERIAL_PORT.println(goSpeed);
      goSpeed -= delta_speed_default;
      delay(accel_delay_default);
    }
  }
  else
  {
    initialSpeed = -initialSpeed;
    goSpeed = initialSpeed - delta_speed_default;
    while (goSpeed > min_decel_speed_default + delta_speed_default)
    {
      motorDriver.setSpeedAB(-goSpeed - left_motor_bias_default, -goSpeed - right_motor_bias_default);
      //SERIAL_PORT.println("moving, speed = ");
      //SERIAL_PORT.println(-goSpeed);
      goSpeed -= delta_speed_default;
      delay(accel_delay_default);
    }
  }    
}
  
// calling moveForward with speed 0 causes motors to coast to stop
// while calling Stop() causes motors to stop and brake.

void move(int mySpeed, int moveTime)
{
  SERIAL_PORT.print("moving, speed = ");
  SERIAL_PORT.println(mySpeed);
  accelerate(mySpeed);
  if (mySpeed != 0 && (!checkForFault()))
  {
    if (mySpeed > 0) motorDriver.setSpeedAB(mySpeed + left_motor_bias_default, mySpeed);
    else motorDriver.setSpeedAB(mySpeed - left_motor_bias_default, mySpeed - right_motor_bias_default);
    Moving = true;
  }
  else coast();
  
  timeOutCheck = millis();
  if (moveTime == 0) delay(move_time_default); // move a normal amount
  else if (moveTime > 0) delay(moveTime);  // move a specified amount. 
  // after the delay, we coast to a stop, unless if moveTime < 0, we move forever until told to stop or current limit exceeded
  if (moveTime >= 0)
  {
    //  decelerate(mySpeed);
    coast();
  }
}

void turn(int mySpeed, int turnTime)
{
  SERIAL_PORT.print("turning, speed = ");
  SERIAL_PORT.println(mySpeed);
  SERIAL_PORT.print(" time = ");
  SERIAL_PORT.println(turnTime);
  //accelerate(mySpeed);
  if (mySpeed != 0 && (!checkForFault()))
  {
    if (mySpeed > 0) motorDriver.setSpeedAB(mySpeed + left_motor_bias_default, -mySpeed - right_motor_bias_default);
    else motorDriver.setSpeedAB(mySpeed - left_motor_bias_default, -mySpeed + right_motor_bias_default);
    Moving = true;
  }
  else Stop();
  timeOutCheck = millis();
  if (turnTime == 0) delay(turn_time_default); // turn a normal amount
  else if (turnTime > 0) delay(turnTime);  // turn a specified amount. 
  // after delay, coast to a stop or, if turnTime < 0, we turn forever until told to stop
  if (turnTime >= 0)
  {
    //  decelerate(mySpeed);
    coast();
  }

}

void tilt(int mySpeed, int tiltTime)
{
  SERIAL_PORT.print("tilting, speed = ");
  SERIAL_PORT.println(mySpeed);
  if (mySpeed != 0 && (!checkForFault()))
  {
    motorDriver.setSpeedC(mySpeed);
    Moving = true;  // set this so that we check for TIME_OUT in the loop
  }
  else motorDriver.setCoastC(); 
  timeOutCheck = millis();
  
  if (tiltTime == 0) delay(tilt_time_default); // tilt a normal amount
  else if (tiltTime > 0) delay(tiltTime);  // tilt a specified amount. 
  // after delay, coast to a stop or, if tiltTime < 0, we tilt forever until told to stop
  if (tiltTime >= 0) motorDriver.setCoastC();  // don't set Moving = false, since base might be moving
}

void getMotorCurrents()
{
  currentLeftMotor = motorDriver.getCurrentA();
  currentRightMotor = motorDriver.getCurrentB();
  currentTopMotor = motorDriver.getCurrentC();
  if ((currentLeftMotor > current_limit_drive_motors_default
    || currentRightMotor > current_limit_drive_motors_default) && current_limit_enabled_default)
  {
    coast();
    SERIAL_PORT.print("Excessive motor current detected = ");
    SERIAL_PORT.print(currentLeftMotor);
    SERIAL_PORT.print(", ");
    SERIAL_PORT.println(currentRightMotor);
  }
  if (currentTopMotor > current_limit_top_motor_default && current_limit_enabled_default) 
  {
    motorDriver.setCoastC();  
    SERIAL_PORT.print("Excessive tilt motor current detected = ");
    SERIAL_PORT.println(currentTopMotor);
  }
}

void monitorMotorCurrents()
{
  currentLeftMotor = motorDriver.getCurrentA();
  currentRightMotor = motorDriver.getCurrentB();
  currentTopMotor = motorDriver.getCurrentC();
  if (currentLeftMotor > 50 || currentRightMotor > 50 || currentTopMotor > 50)
  {
    SERIAL_PORT.print("Motor current Left, Right, Top = ");
    SERIAL_PORT.print(currentLeftMotor);
    SERIAL_PORT.print(", ");
    SERIAL_PORT.print(currentRightMotor);
    SERIAL_PORT.print(", ");
    SERIAL_PORT.println(currentTopMotor);
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
  SERIAL_PORT.print("Bluetooth address is: ");
  for (int i=0; i < 17; i++) SERIAL_PORT.print(buffer[i]);
  SERIAL_PORT.println();
}
   

// process a command string
void HandleCommand(char* input, int length)
{
  int speedToGo = speed_default;
  long value = 0;
  // calculate number following command
  long powerOf10 = 1;
  for (int j=0; j < length - 2; j++) powerOf10 *= 10;
  //SERIAL_PORT.print("length = ");
  //SERIAL_PORT.println(length);
  if (length > 1)
  {
     for (int i = 1; i < length; i++) 
     {
       char singlechar[2];
       singlechar[0] = input[i];
       singlechar[1] = 0;
       value += atoi(&singlechar[0]) * powerOf10;
       //SERIAL_PORT.print("powerof10, input[i], i, value = ");
       //SERIAL_PORT.print(powerOf10);
       //SERIAL_PORT.print(", ");
       //SERIAL_PORT.print(input[i]);
       //SERIAL_PORT.print(", ");
       //SERIAL_PORT.print(i);
       //SERIAL_PORT.print(", ");
       //SERIAL_PORT.println(value);
       powerOf10 /= 10;
     }
       
    //  {value = atoi(&input[1]);  // doesn't work for large numbers, as we have for EEPROM writes.
    //SERIAL_PORT.print("atoi value = ");
    //SERIAL_PORT.println(value);
    if (value < 256)
    {
      speedToGo = value;                  // either speed specified or eeprom address to read
      EEPROMaddress = value;
    }
    else  // means we are writing to the EEPROM
    {
      SERIAL_PORT.print("EEPROM command string parameter is ");
      SERIAL_PORT.println(value);
      EEPROMvalue=  value%1000;  // the lower three digits are the value
      SERIAL_PORT.print("EEPROM command value is ");
      SERIAL_PORT.println(EEPROMvalue);
      EEPROMaddress = (value - EEPROMvalue) / 1000;  // the upper three digits are the address
      SERIAL_PORT.print("EEPROM command address is ");
      SERIAL_PORT.println(EEPROMaddress);
    }
  }

  // check commands
  // ************note that if you use more than one character here
  // the bytes are swapped, ie 'FM' means command MF *****************
  // you can use this stmt to get the command:
  // int* command = (int*)input;
  // but not needed when we just have a single character command format
  switch(input[0]) {
    
    // move forward
    case 't':
      move(speedToGo, nudge_move_time_default);  // forward a little
      break;
    case 'f':    
      move(speedToGo, 0);  // forward for the default time
      break;
    case 'F':
      move(speedToGo, -1);  // forward forever
      break;
      
    // move backward
    case 'g':
      move(-speedToGo, nudge_move_time_default);  // backward a little
      break;
    case 'b':
      move(-speedToGo, 0);  // backward for the default time
      break;
    case 'B':
      move(-speedToGo, -1);  // backward forever
      break;
      
    // turn right
    case 'y':
      turn(speedToGo, nudge_turn_time_default);  // turn right a little
    case 'r':    
      turn(speedToGo, 0);  // turn right for the default time
      break;
    case 'R':    
      turn(speedToGo, -1);    // turn right forever
      break;
      
    // turn left
    case 'h':
      turn(-speedToGo, nudge_turn_time_default);  // turn left a little
      break;
    case 'l':   // turn left
      turn(-speedToGo, 0);  // turn left for the default time
      break;
    case 'L':
      turn(-speedToGo, -1);  // turn left forever
      break;    
      
    // tilt up
    case 'i':
      tilt(tilt_up_speed_default, nudge_tilt_time_default);  // nudge tilt up
      break;
    case 'u':    // tilt up
      tilt(tilt_up_speed_default, 0);  // tilt up for the default time
      break;
    case 'U':    // tilt up forever
      tilt(tilt_up_speed_default, -1);  // tilt up forever
      break;
      
    // tilt down
    case 'k':
      tilt(-tilt_down_speed_default, nudge_tilt_time_default);  // nudge tilt down
      break;
    case 'n':    // tilt down
      tilt(-tilt_down_speed_default, 0);  // tilt down for the default time
      break;
    case 'N':    // tilt down forever
      tilt(-tilt_down_speed_default, -1);  // tilt down forever
      break;
      
      
    case 'x':    // stop drive wheels
    case 'X':
      Stop();
      break;
      
    case 'p':
      SERIAL_PORT_BLUETOOTH.print("MESSAGE_BATTERY_PERCENT");  // lead with "mb"  
                                      // 'm' indicates that this is a message for the server
                                      // 'b' indicates that it is a battery percent messsage
      SERIAL_PORT_BLUETOOTH.println(checkBattery()); 
      break;
      
    // EEPROM commands
    case 'E':
      EEPROMvalue = readFromEEPROM(EEPROMaddress);
      SERIAL_PORT_BLUETOOTH.print("MESSAGE_EEPROM_VALUE");  // lead with "mE"  
                                      // 'm' indicates that this is a message for the server
                                      // 'E' indicates that it is an EEPROM value
      SERIAL_PORT_BLUETOOTH.println(EEPROMvalue); 
      SERIAL_PORT.print("EEPROM requested:  address, value: ");
      SERIAL_PORT.print(EEPROMaddress);
      SERIAL_PORT.print(", ");
      SERIAL_PORT.println(EEPROMvalue); 
      break;
    case 'Z':
      enableEEPROMwrite = true;
      SERIAL_PORT.println("EEPROM writing enabled.");
      SERIAL_PORT_BLUETOOTH.println("EEPROM writing enabled.");
      break;
    case 'z':
      enableEEPROMwrite = false;
      SERIAL_PORT.println("EEPROM writing disabled.");
      SERIAL_PORT_BLUETOOTH.println("EEPROM writing disabled.");
      break;
    case 'v':
      if (enableEEPROMwrite)
      {
        writeToEEPROM(EEPROMaddress, EEPROMvalue);
        SERIAL_PORT.print("Value = ");
        SERIAL_PORT.print(EEPROMvalue);
        SERIAL_PORT.print(" was written to EEPROM address = ");
        SERIAL_PORT.print(EEPROMaddress);
      }
      else SERIAL_PORT.print("EEPROM write requested when writing not enabled");
      break;
    case 'a':
    case 'A':
      readBTaddress();
      break;
        
    default:
      SERIAL_PORT.print("did not recognize command: ");
       if (input[0] == 13) SERIAL_PORT.print(" 13 ");
       else if (input[0] == 10) SERIAL_PORT.print(" 10 ");
       else if (input[0] == 32) SERIAL_PORT.print(" 32 ");
       else if (input[0] == 0) SERIAL_PORT.print(" 0 ");
       else SERIAL_PORT.print(input[0]);
      Stop();
      break;
  }
} 

void setup()  
{
  setDefaults();
  // get currents at the start, since the first value seems to often be a large number
  currentLeftMotor = motorDriver.getCurrentA();
  currentRightMotor = motorDriver.getCurrentB();
  currentTopMotor = motorDriver.getCurrentC();
  
  SERIAL_PORT.begin(SERIAL_SPEED);
  SERIAL_PORT.println("ready for commands");
  SERIAL_PORT_BLUETOOTH.begin(BLUETOOTH_SPEED);   // usually connect to bluetooth on serial2
  motorDriver.setCoastAB();
  motorDriver.setCoastC();
  
  getMotorCurrents();
  
  batteryRange = full_battery_voltage_default - zero_percent_battery_voltage_default;
  
  for (int i=0; i< INPUT_BUFFER_SIZE; i++) inputBuffer[i] = 0;
  
  
}

void loop()
{
  inputLength = 0;
  charIn = 0;
  while (charIn != COMMAND_END_CHARACTER && inputLength < INPUT_BUFFER_SIZE)
  {
    while (!SERIAL_PORT_BLUETOOTH.available()) // wait for input
    {
      if ((millis() - timeOutCheck > timed_out_default) && Moving)
      {
        //SERIAL_PORT.println("timed out");
        Stop();  //if we are moving and haven't heard anything in a long time, stop moving
        motorDriver.setCoastC();  // stop tilting too        
        Moving = false;
      }
       getMotorCurrents();
       //monitorMotorCurrents();
       delay(10);
    }
    // got a character on the bluetooth port
    charIn = SERIAL_PORT_BLUETOOTH.read(); // read it in
    if (charIn != 13 && charIn != 10 && charIn != 32)  // ignore carriage returns, line feeds, and spaces
    {
       inputBuffer[inputLength] = charIn;  // building the command string
       inputLength++;
       //SERIAL_PORT.print(charIn);
    }
  } // this loop keeps going until the COMMAND_END_CHARACTER is received or the string gets too long
  
  //now flush anything that came after the COMMAND_END_CHARACTER
  if (SERIAL_PORT_BLUETOOTH.available())
  {
    SERIAL_PORT.print("Extra character(s) received have ASCII values = ");  // this is usually the null char end string (0)
    while (SERIAL_PORT_BLUETOOTH.available())
    {
       charIn = SERIAL_PORT_BLUETOOTH.read(); // read it in so we can see what is really getting sent
       if (charIn == 13) SERIAL_PORT.print(" 13 ");
       else if (charIn == 10) SERIAL_PORT.print(" 10 ");
       else if (charIn == 32) SERIAL_PORT.print(" 32 ");
       else if (charIn == 0) SERIAL_PORT.print(" 0 ");
       else SERIAL_PORT.print(charIn);
       SERIAL_PORT.print(" ");
    }
    SERIAL_PORT.println();
  }
  
  // throw away the command end character
  inputLength -= 1;  // -1 because it is incremented after the last character
  inputBuffer[inputLength] = 0;  // change COMMAND_END_CHARACTER to a 0 (remember, index is one behind inputLength)
  
 SERIAL_PORT.print("commanded: ");
 SERIAL_PORT.println(inputBuffer);

  // if the command == COMM_CHECK_CHARACTER it is just a local bluetooth comm check
  if (inputBuffer[0] != COMM_CHECK_CHARACTER && inputLength > 0)
  {
      // echo the command, so that the android app knows we are alive
    SERIAL_PORT_BLUETOOTH.print(COMMAND_ECHO_CHARACTER); // lead with this character to indicate just a command echo
    SERIAL_PORT_BLUETOOTH.println(inputBuffer); // need to println because android uses the CR as a delimiter
    HandleCommand(inputBuffer, inputLength);  // this goes after the echo, so that responses will show up on the details screen
  }
  else  // just a comm check
  {
    SERIAL_PORT_BLUETOOTH.print("c");  // this indicates it is just a response to a comm check
    SERIAL_PORT_BLUETOOTH.println(checkBattery()); // might as well send along the battery state
  } 
    
  // clear the buffer for the next command
  for (int i = 0; i < inputLength; i++) inputBuffer[i] = 0;
}


