
// accepts serial port inputs and responds with moves and tilts
// requires a Mega to work, not an Uno or Nano, due to the need for two serial inputs

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
// J to center and M for max down
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

// one turn of the motor = 968 steps in the encoder.
// with large vex wheel, one rotation = 42 cm
// so we have 23 steps per cm


// macros to set and clear individual register bits
#define clearRegisterBit(register, bit) (_SFR_BYTE(register) &= ~_BV(bit))
#define setRegisterBit(register, bit) (_SFR_BYTE(register) |= _BV(bit))

// Macro that clears all Timer/Counter5 interrupt flags by writing a 1 to the flag bit
#define CLEAR_ALL_TIMER5_INT_FLAGS    (TIFR5 = TIFR5)

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
#define MESSAGE_BATTERY_PERCENT mb
#define MESSAGE_EEPROM_VALUE mE
#define EEPROM_TEST_VALUE_10 2
#define EEPROM_TEST_VALUE_11 4
#define EEPROM_TEST_VALUE_12 8

// EEPROM written defines
#define TIMED_OUT 3000
#define DEFAULT_SPEED 220
#define BW_REDUCTION 50
#define DEFAULT_TILT_UP_SPEED 180
#define DEFAULT_TILT_DOWN_SPEED 135
#define DEFAULT_DEGREES 10
#define TICKS_PER_DEGREE_OF_TILT 30
//#define TD_REDUCTION 50
#define DEFAULT_TURN_FOREVER_SPEED 220
#define MOVE_TIME 100
#define TILT_TIME 500
#define MIN_ACCEL_SPEED 100
#define MIN_DECEL_SPEED 60
#define DELTA_SPEED 10
#define ACCEL_DELAY 200
#define LEFT_MOTOR_BIAS 10
#define LEFT_MOTOR_BW_BIAS 23
#define LEFT_MOTOR_STOP_DELAY 0
#define CURRENT_LIMIT_TOP_MOTOR 2000
#define CURRENT_LIMIT_DRIVE_MOTORS 4500
// one turn of the motor = 960 steps in the encoder.
// with large vex wheel, one rotation = 42 cm
// so we have 23 steps per cm
#define ENCODER_TICKS_PER_CM 23

//#define TILT_CENTER 85
//#define TILT_LOOK_DOWN 135
//#define TILT_MIN 55
//#define TILT_MAX 165
//#define TILT_DELTA 10

#define ZERO_PERCENT_BATTERY_VOLTAGE 10.5
#define FULL_BATTERY_VOLTAGE 13.0
#define VOLTAGE_DIVIDER_RATIO 3.2

threeMotorsDriverPCB motorDriver;

char inputBuffer[INPUT_BUFFER_SIZE], charIn;
int tiltPos, inputLength, mySpeed;
bool Moving, brakesOn, exceededCurrentLimitC = false, enableEEPROMwrite = false;
long timeOutCheck;
float batteryRange;
int currentTopMotor, currentRightMotor, currentLeftMotor;
volatile unsigned long encoderCounterLeft = 0; //range from 0 to 4,294,967,295 (2^32 - 1)
volatile unsigned long encoderCounterRight = 0;
unsigned int lastReportedCounterLeft = 1, lastReportedCounterRight = 1;
int program_version;
int timed_out_default, speed_default, bw_reduction_default, tilt_up_speed_default;
int tilt_down_speed_default, degrees_default, ticks_per_degree_of_tilt_default;
//int td_reduction_default;
int turn_forever_speed_default, move_time_default, tilt_time_default;
int min_accel_speed_default, min_decel_speed_default, delta_speed_default, accel_delay_default;
int left_motor_bias_default, left_motor_bw_bias_default, left_motor_stop_delay_default;
int current_limit_top_motor_default, current_limit_drive_motors_default;
int encoder_ticks_per_cm_default;
//int battery_monitor_pin_default;
int EEPROMvalue, EEPROMaddress;
double zero_percent_battery_voltage_default,full_battery_voltage_default,voltage_divider_ratio_default;


// have to redo some of these, as they do not fit into one byte
void setDefaults()
{
 if (EEPROM.read(10) == EEPROM_TEST_VALUE_10 &&
    EEPROM.read(11) == EEPROM_TEST_VALUE_11 &&
    EEPROM.read(12) == EEPROM_TEST_VALUE_12)  // test to see if we have written to the EEPROM
  {
    timed_out_default = EEPROM.read(101);
    speed_default = EEPROM.read(102);
    bw_reduction_default = EEPROM.read(103);
    tilt_up_speed_default = EEPROM.read(104);
    tilt_down_speed_default = EEPROM.read(105);
    degrees_default = EEPROM.read(106);
    ticks_per_degree_of_tilt_default = EEPROM.read(107);
    //td_reduction_default = EEPROM.read(108);
    turn_forever_speed_default = EEPROM.read(109);
    move_time_default = EEPROM.read(110);
    tilt_time_default = EEPROM.read(111);
    min_accel_speed_default = EEPROM.read(112);
    min_decel_speed_default = EEPROM.read(113);
    delta_speed_default = EEPROM.read(114);
    accel_delay_default = EEPROM.read(115);
    left_motor_bias_default = EEPROM.read(116);
    left_motor_bw_bias_default = EEPROM.read(117);
    left_motor_stop_delay_default = EEPROM.read(118);
    current_limit_top_motor_default = EEPROM.read(119)*10;     // multiply to keep the eeprom parameter < 255
    current_limit_drive_motors_default = EEPROM.read(120)*100; // multiply to keep the eeprom parameter < 255
    encoder_ticks_per_cm_default = EEPROM.read(121);
    zero_percent_battery_voltage_default = ((double) EEPROM.read(122)) + (( (double) EEPROM.read(123)) / 10.);
    full_battery_voltage_default = ((double) EEPROM.read(124)) + ( ((double) EEPROM.read(125)) / 10.);
    voltage_divider_ratio_default = ((double) EEPROM.read(125)) + ( ((double) EEPROM.read(126)) / 10.);    
  }
  else
  {
    timed_out_default =  TIMED_OUT;
    speed_default = DEFAULT_SPEED;
    bw_reduction_default = BW_REDUCTION;
    tilt_up_speed_default = DEFAULT_TILT_UP_SPEED;
    tilt_down_speed_default = DEFAULT_TILT_DOWN_SPEED;
    degrees_default = DEFAULT_DEGREES;
    ticks_per_degree_of_tilt_default = TICKS_PER_DEGREE_OF_TILT;
    //td_reduction_default = TD_REDUCTION;
    turn_forever_speed_default = DEFAULT_TURN_FOREVER_SPEED;
    move_time_default = MOVE_TIME;
    tilt_time_default = TILT_TIME;
    min_accel_speed_default = MIN_ACCEL_SPEED;
    min_decel_speed_default = MIN_DECEL_SPEED;
    delta_speed_default = DELTA_SPEED;
    accel_delay_default = ACCEL_DELAY;
    left_motor_bias_default = LEFT_MOTOR_BIAS;
    left_motor_bw_bias_default = LEFT_MOTOR_BW_BIAS;
    left_motor_stop_delay_default = LEFT_MOTOR_STOP_DELAY;
    current_limit_top_motor_default = CURRENT_LIMIT_TOP_MOTOR;
    current_limit_drive_motors_default = CURRENT_LIMIT_DRIVE_MOTORS;
    encoder_ticks_per_cm_default = ENCODER_TICKS_PER_CM;
    //battery_monitor_pin_default = BATTERY_MONITOR_PIN;
    zero_percent_battery_voltage_default = ZERO_PERCENT_BATTERY_VOLTAGE;
    full_battery_voltage_default = FULL_BATTERY_VOLTAGE;
    voltage_divider_ratio_default = VOLTAGE_DIVIDER_RATIO;
  }
}

void checkBattery()
{
  float voltage =  (float) ((analogRead(BATTERY_MONITOR_PIN) / 1023.) * 5.0 ) * voltage_divider_ratio_default;
  int batteryPercent =  (int) ( 100. * ( ( voltage - zero_percent_battery_voltage_default) / batteryRange)); // returns percentage
  if (batteryPercent > 99) batteryPercent = 100;
  if (batteryPercent < 0) batteryPercent = 0;
  //batteryPercent = analogRead(batteryMonitorPin);  // for testing
  SERIAL_PORT_BLUETOOTH.print("MESSAGE_BATTERY_PERCENT");  // lead with "mb"  
                                      // 'm' indicates that this is a message for the server
                                      // 'b' indicates that it is a battery percent messsage
  SERIAL_PORT_BLUETOOTH.println(batteryPercent); 
} 


// use timer5 for tilt encoder
void setupTiltEncoder()
{ 
  noInterrupts();  // disable all interrupts while we mess with timers
  TCCR5A=0;  // reset timer/counter5 control register A  
  
  // register B: bit 7 = 1  to use the filter "Noise canceller" see p.160
  // of the ATMEL 2560 chip specification document
  // bit 6 = 1 to capture on rising edge
  // bits 0,1,2 all = 1 to use external clock (counter)
  // 128 + 64 + 7 = 199, so we could just   TCCR5B = 199;
  // but this is clearer:
  TCCR5B = 0;
  setRegisterBit(TCCR5B, ICNC5);
  setRegisterBit(TCCR5B, ICES5);
  setRegisterBit(TCCR5B, CS52);
  setRegisterBit(TCCR5B, CS51);
  setRegisterBit(TCCR5B, CS50);
  
  setRegisterBit(TIMSK5, TOIE5);
  // Set bit 0 (TOIE5) to use the overflow interrupt to count cycles of the counter 
  // timer5 overflow is interrupt vector 51 (program address $0064) (p.106)
  // and the flag is in TIFR5, bit 0
  
  TCNT5=0;   // counter value = 0
  interrupts();  // re-enable interrupts
  
}

bool stopIfFault()
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
  motorDriver.setCoastAB();
  brakesOn = false;
}

void brakes()
{
  motorDriver.setBrakesAB();
  brakesOn = true;
}

void Stop()
{
  motorDriver.setBrakesAB();
  brakesOn = true;
  Moving = false;
}

void accelerate(int targetSpeed)
{
  int goSpeed = min_accel_speed_default;
  if (targetSpeed > 0)
  {
    while (goSpeed < targetSpeed - delta_speed_default)
    {
      motorDriver.setSpeedAB(goSpeed + left_motor_bias_default,goSpeed);
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
      motorDriver.setSpeedAB(-goSpeed - left_motor_bias_default,-goSpeed);
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
      motorDriver.setSpeedAB(goSpeed + left_motor_bias_default, goSpeed);
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
      motorDriver.setSpeedAB(-goSpeed - left_motor_bias_default, -goSpeed);
      //SERIAL_PORT.println("moving, speed = ");
      //SERIAL_PORT.println(-goSpeed);
      goSpeed -= delta_speed_default;
      delay(accel_delay_default);
    }
  }    
}
  
// calling moveForward with speed 0 causes motors to coast to stop
// while calling Stop() causes motors to stop and brake.

void moveForwardaLittle(int mySpeed)
{
  //mySpeed = speed_default;  // speed goes from 0 to 255
  SERIAL_PORT.print("moving, speed = ");
  SERIAL_PORT.println(mySpeed);
  if (brakesOn) coast();
  long startingEncoderLeft = encoderCounterLeft;
  long startingEncoderRight = encoderCounterRight;
  motorDriver.setSpeedAB(mySpeed + left_motor_bias_default, mySpeed);
  if (mySpeed == 0 || stopIfFault()) Moving = false;
  else Moving = true;
  timeOutCheck = millis();
  delay(300);
  Stop();
  long cmMoved = (encoderCounterLeft - startingEncoderLeft) + (encoderCounterRight - startingEncoderRight);
  cmMoved = (long) ( cmMoved / (2.0 * (float) encoder_ticks_per_cm_default));
  SERIAL_PORT.print("move distance in cm = ");
  SERIAL_PORT.println(cmMoved);
}

void moveForward(int mySpeed)
{
  //mySpeed = speed_default;  // speed goes from 0 to 255
  SERIAL_PORT.print("moving, speed = ");
  SERIAL_PORT.println(mySpeed);
  if (brakesOn) coast();
//  accelerate(mySpeed);
  motorDriver.setSpeedAB(mySpeed + left_motor_bias_default, mySpeed);
  if (mySpeed == 0 || stopIfFault()) Moving = false;
  else Moving = true;
  timeOutCheck = millis();
  delay(move_time_default);
//  decelerate(mySpeed);
  Stop();
}

void moveForwardForever(int mySpeed)
{
  //mySpeed = speed_default;  // speed goes from 0 to 255
  SERIAL_PORT.print("moving, speed = ");
  SERIAL_PORT.println(mySpeed);
  if (brakesOn) coast();
//  accelerate(mySpeed);
  motorDriver.setSpeedAB(mySpeed + left_motor_bias_default, mySpeed);
  if (mySpeed == 0 || stopIfFault()) Moving = false;
  else Moving = true;
  timeOutCheck = millis(); 
}

void moveBackwardaLittle(int mySpeed)
{
  mySpeed = -speed_default +  bw_reduction_default;  // speed goes from 0 to 255
  SERIAL_PORT.print("moving, speed = ");
  SERIAL_PORT.println(mySpeed);
  if (brakesOn) coast();
  //accelerate(mySpeed);
  motorDriver.setSpeedAB(mySpeed - left_motor_bw_bias_default, mySpeed);
  if (mySpeed == 0 || stopIfFault()) Moving = false;
  else Moving = true;
  timeOutCheck = millis();
  delay(300);
  //decelerate(mySpeed);
  Stop();
}

void moveBackward(int mySpeed)
{
  mySpeed = -speed_default +  bw_reduction_default;  // speed goes from 0 to 255// backward should be slower than forward
  SERIAL_PORT.print("moving, speed = ");
  SERIAL_PORT.println(mySpeed);
  if (brakesOn) coast();
  //accelerate(mySpeed);
  motorDriver.setSpeedAB(mySpeed - left_motor_bw_bias_default, mySpeed);
  if (mySpeed == 0 || stopIfFault()) Moving = false;
  else Moving = true;
  timeOutCheck = millis();
  delay(move_time_default);
  //decelerate(mySpeed);
  Stop();
}

void moveBackwardForever(int mySpeed)
{
  mySpeed = -speed_default +  bw_reduction_default;  // speed goes from 0 to 255 // backward should be slower than forward
  SERIAL_PORT.print("moving, speed = ");
  SERIAL_PORT.println(mySpeed);
  if (brakesOn) coast();
  //accelerate(mySpeed);
  motorDriver.setSpeedAB(mySpeed - left_motor_bw_bias_default, mySpeed);
  if (mySpeed == 0 || stopIfFault()) Moving = false;
  else Moving = true;
  timeOutCheck = millis();
}

void turnRightForever(int mySpeed) // speed goes from 0 to 255
{
  mySpeed = turn_forever_speed_default; 
  SERIAL_PORT.print("turning, speed = ");
  SERIAL_PORT.println(mySpeed);
  if (brakesOn) coast();
  motorDriver.setSpeedAB(mySpeed, -mySpeed);
  if (mySpeed == 0 || stopIfFault()) Moving = false;
  else Moving = true;
  timeOutCheck = millis(); 
}

void turnRight(int mySpeed) // speed goes from 0 to 255
{
  mySpeed = speed_default;  
  SERIAL_PORT.print("turning, speed = ");
  SERIAL_PORT.println(mySpeed);
  if (brakesOn) coast();
  motorDriver.setSpeedAB(mySpeed, -mySpeed);
  if (mySpeed == 0 || stopIfFault()) Moving = false;
  else Moving = true;
  timeOutCheck = millis();
  delay(200);
  Stop();  
}

void turnLeftForever(int mySpeed) // speed goes from 0 to 255
{
  mySpeed = turn_forever_speed_default; 
  SERIAL_PORT.print("turning, speed = ");
  SERIAL_PORT.println(mySpeed);
  if (brakesOn) coast();
  motorDriver.setSpeedAB(-mySpeed, mySpeed);
  if (mySpeed == 0 || stopIfFault()) Moving = false;
  else Moving = true;
  timeOutCheck = millis();
}


void turnLeft(int mySpeed) // speed goes from 0 to 255
{
  mySpeed = speed_default; 
  SERIAL_PORT.print("turning, speed = ");
  SERIAL_PORT.println(mySpeed);
  if (brakesOn) coast();
  motorDriver.setSpeedAB(-mySpeed, mySpeed);
  if (mySpeed == 0 || stopIfFault()) Moving = false;
  else Moving = true;
  timeOutCheck = millis();
  delay(200);
  Stop();    
}


void tiltUp(int degreesToMove) // distance goes from 0 to 255
{
  TCNT5=0;   // counter value = 0
  mySpeed = tilt_up_speed_default; 
  SERIAL_PORT.print("tilting up, degrees = ");
  SERIAL_PORT.println(degreesToMove);
  motorDriver.setSpeedC(mySpeed);
  //if (stopIfFault()) Moving = false;
  //else Moving = true;
  Moving = true;
  timeOutCheck = millis();
  /*
  if (ticks_per_degree_of_tilt_default > 1  && degreesToMove > 0 )
  {
    SERIAL_PORT.println("number of counts recorded while tilting");
    long cycles = 0;
    long count = 0;
    long targetCount = degreesToMove * ticks_per_degree_of_tilt_default;
    int movementStopped = 0;
    int oldCount = 0;
    while (count < targetCount && movementStopped < 3 && currentTopMotor < current_limit_top_motor_default)
    {
       if (TIFR5 & 0x01) // check to see if tilt encoder overflowed
       {
           cycles++;
           CLEAR_ALL_TIMER5_INT_FLAGS;
       }
       count = (cycles * 65636) + TCNT5;
       if (count == oldCount) movementStopped++;
       oldCount = count;
       SERIAL_PORT.print("Tilting up, encoder: ");
       SERIAL_PORT.println(count);
       getMotorCurrents();
       SERIAL_PORT.print("Tilt motor current = ");
       SERIAL_PORT.println(currentTopMotor);
       delay(100);
    }
    if (currentTopMotor >= current_limit_top_motor_default)
    {
      motorDriver.setSpeedC(0);
      delay(200);
      SERIAL_PORT.println("Tilt motor current exceeded limit");
      currentTopMotor = 0;
      if (exceededCurrentLimitC)
      {
        SERIAL_PORT.println("Tilt motor current exceeded limit twice in a row");
        return;
      }
      exceededCurrentLimitC = true;
      tiltDown(5);
      return;   
    }
  }
  else 
  */
   for (int i = 0; i < 10; i++)
 {
    getMotorCurrents();
    SERIAL_PORT.print("Tilt motor current = ");
    SERIAL_PORT.println(currentTopMotor);
    if (currentTopMotor >= current_limit_top_motor_default)
    {
      motorDriver.setSpeedC(0);
      delay(200);
      SERIAL_PORT.println("Tilt motor current exceeded limit");
      currentTopMotor = 0;
      if (exceededCurrentLimitC && i > 2) // the first couple of current values are high, so we ignore them
      {
        SERIAL_PORT.println("Tilt motor current exceeded limit twice in a row");
        return;
      }
      exceededCurrentLimitC = true;
      //tiltDown(8);
      //delay(100);
      //tiltDown(5);
      return; 
     
    }
    
    delay(tilt_time_default/10);
 }
  
  motorDriver.setBrakesC();   
  Moving = false; 
  exceededCurrentLimitC = false;
  timeOutCheck = millis(); 
}


void tiltDown(int degreesToMove)
{
  TCNT5=0;   // counter value = 0
  mySpeed = -tilt_down_speed_default; 
  SERIAL_PORT.print("tilting down, degrees = ");
  SERIAL_PORT.println(degreesToMove);
  motorDriver.setSpeedC(mySpeed);
  //if (stopIfFault()) Moving = false;
  //else Moving = true;
  Moving = true;
  timeOutCheck = millis();
  
  /*
  if (ticks_per_degree_of_tilt_default > 1 && degreesToMove > 0 )
  {
    long cycles = 0;
    long count = 0;
    long targetCount = degreesToMove * ticks_per_degree_of_tilt_default;
    int movementStopped = 0;
    int oldCount = 0;
    while (count < targetCount && movementStopped < 3 && currentTopMotor < current_limit_top_motor_default)
    {
       if (TIFR5 & 0x01) // check to see if tilt encoder overflowed
       {
           cycles++;
           CLEAR_ALL_TIMER5_INT_FLAGS;
       }
       count = (cycles * 65636) + TCNT5;
       if (count == oldCount) movementStopped++;
       oldCount = count;
       SERIAL_PORT.print("Tilting down, encoder: ");
       SERIAL_PORT.println(count);
       getMotorCurrents();
       SERIAL_PORT.print("Tilt motor current = ");
       SERIAL_PORT.println(currentTopMotor);
       delay(100);
    }
    if (currentTopMotor >= current_limit_top_motor_default)
    {
      motorDriver.setSpeedC(0);
      delay(200);
      SERIAL_PORT.println("Tilt motor current exceeded limit");
      currentTopMotor = 0;
      if (exceededCurrentLimitC)
      {
        SERIAL_PORT.println("Tilt motor current exceeded limit twice in a row");
        return;
      }
      exceededCurrentLimitC = true;
      tiltUp(5);
      return; 
    }
  }
  else
 */
 for (int i = 0; i < 10; i++)
 {
    getMotorCurrents();
    SERIAL_PORT.print("Tilt motor current = ");
    SERIAL_PORT.println(currentTopMotor);
    if (currentTopMotor >= current_limit_top_motor_default)
    {
      motorDriver.setSpeedC(0);
      delay(200);
      SERIAL_PORT.println("Tilt motor current exceeded limit");
      currentTopMotor = 0;
      if (exceededCurrentLimitC)
      {
        SERIAL_PORT.println("Tilt motor current exceeded limit twice in a row");
        return;
      }
      exceededCurrentLimitC = true;
      //tiltUp(8);
      //delay(100);
      //tiltUp(5);
      return; 
    }
    delay(tilt_time_default/10);
 }

  
  motorDriver.setBrakesC();   
  Moving = false; 
  exceededCurrentLimitC = false;
  timeOutCheck = millis(); 
}

void getMotorCurrents()
{
  currentLeftMotor = motorDriver.getCurrentA();
  currentRightMotor = motorDriver.getCurrentB();
  currentTopMotor = motorDriver.getCurrentC();
}

// process a command string
void HandleCommand(char* input, int length)
{
  int speedToGo = speed_default;
  int degreesToGo = degrees_default;  // degrees of tilt
  long value = 0;
  // calculate number following command
  if (length > 1)
  {
    value = atoi(&input[1]);
    if (value < 256)
    {
      speedToGo = value;                  // either servo steps or speed specified or eeprom address to read
      degreesToGo = value;
      EEPROMaddress = value;
    }
    else  // means we are writing to the EEPROM
    {
      EEPROMvalue =  value % 1000;  // the lower three digits are the value
      EEPROMaddress = value - EEPROMvalue;  // the upper three digits are the address
    }
  }

  // check commands
  // ************note that if you use more than one character here
  // the bytes are swapped, ie 'FM' means command MF *****************
  // you can use this stmt to get the command:
  // int* command = (int*)input;
  // but not needed when we just have a single character command format
  switch(input[0]) {
    case 't':
      moveForwardaLittle(speedToGo);
      break;
    case 'f':    // move forward
      moveForward(-speedToGo);
      break;
    case 'F':
      moveForwardForever(speedToGo);
      break;
    case 'g':
      moveBackwardaLittle(speedToGo);
      break;
    case 'b':
      moveBackward(speedToGo);
      break;
      case 'B':
      moveBackwardForever(speedToGo);
      break;
    case 'r':    // turn right
      turnRight(speedToGo);
      break;
    case 'R':    // turn right
      turnRightForever(speedToGo);
      break;
    case 'l':    // turn left
      turnLeft(speedToGo);
      break;
    case 'L':
      turnLeftForever(speedToGo);
      break;    
    case 'x':    // stop
      Stop();
      break;
    case 'u':    // tilt up
      tiltUp(degreesToGo);
      break;
    case 'n':    // tilt down
      tiltDown(degreesToGo);
      break;
    case 'p':
      checkBattery(); // note that this writes a single char, so value should be in range 0-255
      break;    
    case 'E':
      EEPROMvalue = readFromEEPROM(EEPROMaddress);
      SERIAL_PORT_BLUETOOTH.print("MESSAGE_EEPROM_VALUE");  // lead with "mE"  
                                      // 'm' indicates that this is a message for the server
                                      // 'E' indicates that it is an EEPROM value
      SERIAL_PORT_BLUETOOTH.println(EEPROMvalue); 
      break;
    case 'Z':
      enableEEPROMwrite = true;
      SERIAL_PORT.print("EEPROM writing enabled.");
      break;
    case 'z':
      enableEEPROMwrite = false;
      SERIAL_PORT.print("EEPROM writing disabled.");
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

void doEncoderLeft()
{
  encoderCounterLeft++;
}

void doEncoderRight()
{
  encoderCounterRight++;
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


void setup()  
{
  setDefaults();
  SERIAL_PORT.begin(SERIAL_SPEED);
  SERIAL_PORT.println("ready for commands");
  SERIAL_PORT_BLUETOOTH.begin(BLUETOOTH_SPEED);   // usually connect to bluetooth on serial2
  motorDriver.setCoastAB();
  motorDriver.setBrakesC();
  
  // attach interrupts
  // encoder pin for left motor is on interrupt 5 (pin 18)
  // encoder pin for right motor is on interrupt 4 (pin 19)
  attachInterrupt(motorDriver.INTERRUPTA, doEncoderLeft, CHANGE);
  attachInterrupt(motorDriver.INTERRUPTB, doEncoderRight, CHANGE);
  
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
       if (millis() - timeOutCheck > timed_out_default && Moving) Stop();  //if we are moving and haven't heard anything in a long time, stop moving 
       getMotorCurrents();
       delay(10);
    }
    charIn = SERIAL_PORT_BLUETOOTH.read(); // read it in
    if (charIn != 13 && charIn != 10 && charIn != 32)  // ignore carriage returns, line feeds, and spaces
    {
       inputBuffer[inputLength] = charIn;
       inputLength++;
       //SERIAL_PORT.print(charIn);
    }
  } 
  
  //now flush anything that came after the COMMAND_END_CHARACTER
  if (SERIAL_PORT_BLUETOOTH.available())
  {
    SERIAL_PORT.print("Extra characters received are: ");  // this is usually the null char end string (0)
    while (SERIAL_PORT_BLUETOOTH.available())
    {
       charIn = SERIAL_PORT_BLUETOOTH.read(); // read it in
       if (charIn == 13) SERIAL_PORT.print(" 13 ");
       else if (charIn == 10) SERIAL_PORT.print(" 10 ");
       else if (charIn == 32) SERIAL_PORT.print(" 32 ");
       else if (charIn == 0) SERIAL_PORT.print(" 0 ");
       else SERIAL_PORT.print(charIn);
    }
    SERIAL_PORT.println();
  }
  
  // throw away the line end character
  inputLength -= 1;  // -1 because it is incremented after the last character
  inputBuffer[inputLength] = 0;  // change COMMAND_END_CHARACTER to a 0 (remember, index is one behind inputLength)
  
 SERIAL_PORT.print("commanded: ");
 SERIAL_PORT.println(inputBuffer);

  // if the command == COMM_CHECK_CHARACTER it is just a local bluetooth comm check
  if (inputBuffer[0] != COMM_CHECK_CHARACTER && inputLength > 0) HandleCommand(inputBuffer, inputLength);
   
  // echo the command
  SERIAL_PORT_BLUETOOTH.println(inputBuffer); // need to println because android uses the CR as a delimiter
  for (int i = 0; i < inputLength; i++) inputBuffer[i] = 0;
}


