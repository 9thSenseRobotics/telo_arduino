// motor control routines
#include <threeMotorsDriverPCB.h>

threeMotorsDriverPCB motorDriver;
double goStraight(double initialYaw, double previousYaw, unsigned long previousTime, int mySpeed);

int currentTopMotor, currentRightMotor, currentLeftMotor, current_limit_enabled_default;
int bw_reduction_default, ticks_per_degree_of_tilt_default;
int turn_forever_speed_default, move_time_default, turn_time_default, tilt_time_default;

int min_accel_speed_default, min_decel_speed_default, delta_speed_default, accel_delay_default;
int left_motor_bias_default, left_motor_bw_bias_default, left_motor_stop_delay_default;
int right_motor_bias_default, right_motor_bw_bias_default, right_motor_stop_delay_default;
int current_limit_top_motor_default, current_limit_drive_motors_default;
double previousDeltaYaw = 0;

int commandForeverSpeed, dampenChangesCounter = 0;

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
  SERIAL_PORT.println("stopping");
  motorDriver.setBrakesAB();
  brakesOn = true;
  Moving = false;
  Turning = false;
}


// calling moveForward with speed 0 causes motors to coast to stop
// while calling Stop() causes motors to stop and brake.

void commandMove(int moveSpeed)
{
  int leftSpeed, rightSpeed;
  if (moveSpeed != 0 && (!checkForFault()))
  {
    if (moveSpeed > 0) 
    {
      leftSpeed = moveSpeed + left_motor_bias_default;
      rightSpeed = moveSpeed + right_motor_bias_default;
      if (leftSpeed > 255) leftSpeed = 255;
      if (rightSpeed > 255) rightSpeed = 255;
    }
    else 
    {
      leftSpeed = moveSpeed - left_motor_bias_default;
      rightSpeed = moveSpeed - right_motor_bias_default;
      if (leftSpeed < -255) leftSpeed = -255;
      if (rightSpeed < -255) rightSpeed = -255;
    }
    motorDriver.setSpeedAB(leftSpeed, rightSpeed);
    Moving = true;
    SERIAL_PORT.print("moving, L, R speeds = ");
    SERIAL_PORT.print(leftSpeed);
    SERIAL_PORT.print(", ");
    SERIAL_PORT.println(rightSpeed);
  }
  else coast();
}


void move(int mySpeed, int moveTime)
{
  if (Moving || Turning) coast(); // protect from reversing a motor abruptly, although this state should never occur
  SERIAL_PORT.print("moving, speed = ");
  SERIAL_PORT.println(mySpeed);
  int goSpeed = min_accel_speed_default;
  double initialYaw;
  long delayTime; // default is move forever;
  if (moveTime == 0) delayTime = move_time_default; // move a normal time
  else delayTime = moveTime;  // move for a specified time, negative 1 means move forever
  
  SERIAL_PORT.print("move time = ");
  SERIAL_PORT.println(delayTime);  
  timeOutCheck = millis();
  if (delayTime < 0)  // go until told to stop
  {
    commandMove(mySpeed);
    commandForeverSpeed = mySpeed;
    return;
  }

  if ((!gyroPresent))
  {
    while ( (long)(millis() - timeOutCheck) < delayTime) // won't loop if delayTime = -1 (move forever)
    {
      //accelerate(mySpeed);
      goSpeed += delta_speed_default; // accelerate every 100 msec
      if (goSpeed > abs(mySpeed)) goSpeed = abs(mySpeed);
      SERIAL_PORT.print("unbiased command speed = ");
      SERIAL_PORT.println(goSpeed);
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
    double previousYaw = initialYaw;
    while ( (long)(millis() - timeOutCheck) < delayTime) // won't loop if delayTime = 0 (move forever)
    // because millis() returns an unsigned long, when delayTime is negative it is greater than millis() - timeOutCheck
    // because it is treating the comparison as if delayTime is an unsigned long, so the negative value is a
    // very large positive number.
    {
      delay(30);
      previousYaw = goStraight(initialYaw, previousYaw, timePrevious,  mySpeed);  // change the bias levels to make straighter path
      if ( !(moveCount % 5)) goSpeed += delta_speed_default; // accelerate every 100 msec
      if (goSpeed > abs(mySpeed)) goSpeed = abs(mySpeed);
      SERIAL_PORT.print("unbiased, unsigned command speed = ");
      SERIAL_PORT.println(goSpeed);
      if (mySpeed > 0) commandMove(goSpeed);
      else commandMove(-goSpeed);
      timePrevious = millis();
      moveCount++;
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
  if (Moving || Turning) coast();  // protect from reversing a motor abruptly, although this state should never occur
  SERIAL_PORT.print("turning, speed, amount = ");
  SERIAL_PORT.print(mySpeed);
  SERIAL_PORT.print(", ");
  SERIAL_PORT.print(turnAmount);
  if (gyroPresent) SERIAL_PORT.println(" degrees");
  else SERIAL_PORT.println(" msec");
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
double goStraight(double initialYaw, double previousYaw, unsigned long previousTime, int mySpeed)
{
  unsigned long deltaTime = millis() - previousTime;
  double currentYaw = updateYaw(deltaTime);
  double integratedYaw = currentYaw - initialYaw;
  double deltaYaw = currentYaw - previousYaw;
  int MAX_BIAS = 30;
  double deltaLeft = 0., deltaRight = 0.;
  
  SERIAL_PORT.print("initial left_motor_bias_default, right_motor_bias_default = ");
  SERIAL_PORT.print(left_motor_bias_default);
  SERIAL_PORT.print(", ");
  SERIAL_PORT.println(right_motor_bias_default);
  
  if (dampenChangesCounter > 0)
  {
    SERIAL_PORT.print("dampenChangesCounter = ");
    SERIAL_PORT.println(dampenChangesCounter);
    dampenChangesCounter--;
    previousDeltaYaw = deltaYaw;
    return currentYaw;
  }
  
  if (deltaYaw * previousDeltaYaw < 0) // sign change-- we are getting near the right numbers
  {
    dampenChangesCounter = 5;
    if (deltaYaw > 0)  // we want to dampen the last few changes
    {
      deltaLeft = -10;
      deltaRight = 10;
    }
  }
  
  else
  {
    boolean sameSignBias = false;
    if (deltaYaw * integratedYaw > 0) sameSignBias = true;  // if delta and integrated values have the same sign, double the bias change.
    
    //derivative, fast immediate correction
    if (deltaYaw > 0.2 || (sameSignBias && (deltaYaw > 0.)) ) // this means we have turn to the left, so make right motor stronger, left motor weaker
    {
      deltaLeft = -4 - (deltaYaw * 10.); //* sameSignBias;
      deltaRight = 4 + (deltaYaw * 10.); // * sameSignBias;
    }
    if (deltaYaw < -0.2 || (sameSignBias && (deltaYaw < 0.)) )  // we have turn to the right, so make left motor stronger, right motor weaker
    {
      deltaLeft = 4 - (deltaYaw * 10.); // * sameSignBias;
      deltaRight= -4 + (deltaYaw * 10.); // * sameSignBias;   
    } 
    
    //integrative, slowly come back to original heading
    if (integratedYaw > 2 || (sameSignBias && (integratedYaw > 0.)) ) // this means we have turn to the left, so make right motor stronger, left motor weaker
    {
      deltaLeft -= 1 + integratedYaw; // -= sameSignBias;
      deltaRight += 1 + integratedYaw; // += sameSignBias;  
    }
    if (integratedYaw < -2 || (sameSignBias && (integratedYaw < 0.)) )  // we have turn to the right, so make left motor stronger, right motor weaker
    {
      deltaLeft += 1 - integratedYaw; //sameSignBias;
      deltaRight -= 1 - integratedYaw; //sameSignBias;  
    }
  }
  
  if (mySpeed > 0)
  {
    if (deltaLeft > 0 )  // each has to be a separate line in case a bias overshoots the max, so this way it can be brought back down again
    {
      if (left_motor_bias_default < MAX_BIAS) left_motor_bias_default += (int) deltaLeft;
    }
    else
    {
      if (left_motor_bias_default > -MAX_BIAS) left_motor_bias_default += (int) deltaLeft;
    }
    if (deltaRight > 0)
    {
       if (right_motor_bias_default < MAX_BIAS) right_motor_bias_default += (int) deltaRight;
    }
    else
    {
      if (right_motor_bias_default > -MAX_BIAS) right_motor_bias_default += (int) deltaRight;
    } 
  }    
  else  // when mySpeed < 0, we are going backwards, so the sign of the bias change is opposite
  {
    if (deltaLeft > 0 )
    {
      if (left_motor_bias_default > -MAX_BIAS) left_motor_bias_default -= (int) deltaLeft;
    }
    else
    {
      if (left_motor_bias_default < MAX_BIAS) left_motor_bias_default -= (int) deltaLeft;
    } 
    if (deltaRight > 0 )
    {
      if (right_motor_bias_default > -MAX_BIAS) right_motor_bias_default -= (int) deltaRight;
    }
    else
    {
      if (right_motor_bias_default < MAX_BIAS) right_motor_bias_default -= (int) deltaRight;
    }      
  }    
  
  SERIAL_PORT.print("integratedYaw, deltaYaw, deltaTime, left_motor_bias_default, right_motor_bias_default =  ");
  SERIAL_PORT.print(integratedYaw);
  SERIAL_PORT.print(", ");
  SERIAL_PORT.print(deltaYaw);
  SERIAL_PORT.print(", ");
  SERIAL_PORT.print( (unsigned int) deltaTime);
  SERIAL_PORT.print(", ");
  SERIAL_PORT.print(left_motor_bias_default);
  SERIAL_PORT.print(", ");
  SERIAL_PORT.println(right_motor_bias_default);
  SERIAL_PORT.print("speed, deltaLeft, deltaRight =  ");
  SERIAL_PORT.print(mySpeed);
  SERIAL_PORT.print(", ");
  SERIAL_PORT.print(deltaLeft);
  SERIAL_PORT.print(", ");
  SERIAL_PORT.println(deltaRight);
  return currentYaw;
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
  currentLeftMotor = 0;
  currentRightMotor = 0;
  currentTopMotor = 0;
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
 
void accelerateTurn(int targetSpeed)
{
  int goSpeed = min_accel_speed_default;
  if (targetSpeed > 0)
  {
    while (goSpeed < targetSpeed - delta_speed_default)
    {
      motorDriver.setSpeedAB(goSpeed + left_motor_bias_default, -goSpeed - right_motor_bias_default); // turn
      goSpeed += delta_speed_default;
      delay(accel_delay_default);
    }
  }
  else
  {
    targetSpeed = -targetSpeed;
    while (goSpeed < targetSpeed - delta_speed_default)
    {
      motorDriver.setSpeedAB(-goSpeed - left_motor_bias_default, goSpeed + right_motor_bias_default); // turn
      goSpeed += delta_speed_default;
      delay(accel_delay_default);
    }
  }
}

/*
void decelerate(int initialSpeed)
{
  int goSpeed;
  if (initialSpeed > 0)
  {
    goSpeed = initialSpeed - delta_speed_default;
    while (goSpeed > min_decel_speed_default + delta_speed_default)
    {
      commandMove(goSpeed);
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
      commandMove(-goSpeed);
      motorDriver.setSpeedAB(-goSpeed - left_motor_bias_default, -goSpeed - right_motor_bias_default);
      goSpeed -= delta_speed_default;
      delay(accel_delay_default);
    }
  }    
}
*/
