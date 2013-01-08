// motor control routines
#include <threeMotorsDriverPCB.h>

threeMotorsDriverPCB motorDriver;

int currentTopMotor, currentRightMotor, currentLeftMotor, current_limit_enabled_default;
int bw_reduction_default, ticks_per_degree_of_tilt_default;
int turn_forever_speed_default, move_time_default, turn_time_default, tilt_time_default;

int min_accel_speed_default, min_decel_speed_default, delta_speed_default, accel_delay_default;
int left_motor_bias_default, left_motor_bw_bias_default, left_motor_stop_delay_default;
int right_motor_bias_default, right_motor_bw_bias_default, right_motor_stop_delay_default;
int current_limit_top_motor_default, current_limit_drive_motors_default;

int commandForeverSpeed;

bool Moving = false, Turning = false, Tilting = false, brakesOn, gyroPresent;

//bool modify_motor_biases_default;
bool modify_motor_biases_default = 0; // ************changed for testing *********************************


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
  Moving = false;
  Turning = false;
}

void coastTilt()
{
  motorDriver.setCoastC();
  Tilting = false;
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
  Moving = false;
  Turning = false;
}


// calling moveForward with speed 0 causes motors to coast to stop
// while calling Stop() causes motors to stop and brake.

void commandMove(int moveSpeed)
{
  if (moveSpeed != 0 && (!checkForFault()))
  {
    if (moveSpeed > 0) motorDriver.setSpeedAB(255,255); // + left_motor_bias_default, moveSpeed + right_motor_bias_default);
    else motorDriver.setSpeedAB(moveSpeed - left_motor_bias_default, moveSpeed - right_motor_bias_default);
    Moving = true;
  }
  else coast();
}


void move(int mySpeed, int moveTime)
{
  SERIAL_PORT.print("moving, speed = ");
  SERIAL_PORT.println(mySpeed);
  if (Moving || Turning) coast(); // protect from reversing a motor abruptly, although this state should never occur
  long delayTime; // default is move forever;
  if (moveTime == 0) delayTime = move_time_default; // move a normal amount
  else delayTime = moveTime;  // move a specified amount, negative 1 means move forever
  SERIAL_PORT.print("move time = ");
  SERIAL_PORT.println(delayTime);  
  timeOutCheck = millis();
  if (delayTime < 0)  // go until told to stop
  {
    commandMove(mySpeed);
    commandForeverSpeed = mySpeed;
    return;
  }
  int goSpeed = min_accel_speed_default;
  double initialYaw;
  if ((!gyroPresent))
  {
    while ( (long)(millis() - timeOutCheck) < delayTime) // won't loop if delayTime = -1 (move forever)
    {
      //accelerate(mySpeed);
      goSpeed += delta_speed_default; // accelerate every 100 msec
      if (goSpeed > abs(mySpeed)) goSpeed = abs(mySpeed);
      if (mySpeed > 0) commandMove(goSpeed);
      else commandMove(-goSpeed);
      delay(100);
    }
  }
  else
  {
    long moveCount = 1;
    unsigned long timePrevious = millis();
    initialYaw = totalYaw;
    while ( (long)(millis() - timeOutCheck) < delayTime) // won't loop if delayTime = 0 (move forever)
    // because millis() returns an unsigned long, when delayTime is negative it is greater than millis() - timeOutCheck
    // because it is treating the comparison as if delayTime is an unsigned long, so the negative value is a
    // very large positive number.
    {
      delay(20);
      goStraight(initialYaw, timePrevious);  // change the bias levels to make straighter path
      if ( !(moveCount % 5)) goSpeed += delta_speed_default; // accelerate every 100 msec
      if (goSpeed > abs(mySpeed)) goSpeed = abs(mySpeed);
      if (mySpeed > 0) commandMove(goSpeed);
      else commandMove(-goSpeed);
      timePrevious = millis();
    }
    if (modify_motor_biases_default && (delayTime >= 0) )  // if we are in a learning mode, write the new biases to EEPROM to recall next powerup
    {
      writeToEEPROM(116, left_motor_bias_default);
      writeToEEPROM(210, right_motor_bias_default);
    }
    if (delayTime >=0)
    {
      SERIAL_PORT.print("delta Yaw during move = ");
      SERIAL_PORT.println(totalYaw - initialYaw);
    }    
  } 
  // after the delay, we coast to a stop, unless if moveTime < 0, we move forever until told to stop or current limit exceeded
  if (delayTime > 0)
  {
    //  decelerate(mySpeed);
    coast();
  }
  SERIAL_PORT.println(totalYaw - initialYaw); 
}

void turn(int mySpeed, int turnAmount)  // turnAmount is either time (ms) or degrees, depending on if a gyro is present
{
  double initialYaw;
  SERIAL_PORT.print("turning, speed = ");
  SERIAL_PORT.println(mySpeed);
  SERIAL_PORT.print(" amount = ");
  SERIAL_PORT.print(turnAmount);
  if (gyroPresent) SERIAL_PORT.println(" degrees");
  else SERIAL_PORT.println(" msec");
  if (Moving || Turning) coast();  // protect from reversing a motor abruptly, although this state should never occur
  //accelerate(mySpeed);
  if (mySpeed != 0 && (!checkForFault()))
  {
    timeOutCheck = millis();
    if (mySpeed > 0) motorDriver.setSpeedAB(mySpeed + left_motor_bias_default, -mySpeed - right_motor_bias_default);
    else motorDriver.setSpeedAB(mySpeed - left_motor_bias_default, -mySpeed + right_motor_bias_default);
    Turning = true;
  }
  else Stop();
  if (gyroPresent) gyroTurnDelay(turnAmount); // turn until we reach turnAmount number of degrees or timeout
    // the gyro routine stops the turn after the turn completed
    // so that it can monitor the overshoot due to time needed to stop
  else
  {
     delay(turnAmount); // without a gyro, just turn for a specified time
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
    Tilting = true;
  }
  else motorDriver.setCoastC(); 
  timeOutCheck = millis();
  
  if (tiltTime == 0) delay(tilt_time_default); // tilt a normal amount
  else if (tiltTime > 0) delay(tiltTime);  // tilt a specified amount. 
  // after delay, coast to a stop or, if tiltTime < 0, we tilt forever until told to stop
  if (tiltTime >= 0) motorDriver.setCoastC();  // don't set Moving = false, since base might be moving
}


// monitor the yaw and change motor biases to make moving go straight
// don't allow the changes to get too big, as this routine could get fooled by a stuck wheel
void goStraight(double initialYaw, unsigned long previousTime)
{
  double deltaYaw = updateYaw(millis() - previousTime) - initialYaw;
  if (deltaYaw < 0) // this means we have turn to the left, so make right motor stronger, left motor weaker
  {
    if (left_motor_bias_default < 20) left_motor_bias_default++;
    if (right_motor_bias_default > -20) right_motor_bias_default--;
  }
  if (deltaYaw > 0)  // we have turn to the right, so make left motor stronger, right motor weaker
  {
    if (left_motor_bias_default > -20) left_motor_bias_default--;
    if (right_motor_bias_default < 20)right_motor_bias_default++;    
  } 
  /*SERIAL_PORT.print("deltaYaw, left_motor_bias_default, right_motor_bias_default =  ");
  SERIAL_PORT.print(deltaYaw);
  SERIAL_PORT.print(", ");
  SERIAL_PORT.print(left_motor_bias_default);
  SERIAL_PORT.print(", ");
  SERIAL_PORT.println(right_motor_bias_default);*/
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
  // average over 2 ms to include one full duty cycle; each point takes about 0.1 ms to return
  for (int i=0; i < 20; i++)currentLeftMotor += motorDriver.getCurrentA();
  for (int i=0; i < 20; i++)currentRightMotor += motorDriver.getCurrentB();
  for (int i=0; i < 20; i++)currentTopMotor += motorDriver.getCurrentC();
  currentLeftMotor /= 20.;
  currentRightMotor /= 20.;
  currentTopMotor /= 20.;
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
 

/*
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
*/

