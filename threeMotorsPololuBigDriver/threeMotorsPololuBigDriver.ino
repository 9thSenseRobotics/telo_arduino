// accepts serial port inputs and responds with moves and servos
// requires a Mega to work, not an Uno or Nano, due to conflicts with the pololu motor board
// and the servo library (both want exclusive use of pins 9 and 10).

// wiring:
// motor 1 is the right motor, motor2 the left.
// hook them both up so that the outside wire from the motor goes into the outside terminal
// that corresponds to M1A for the right motor, M2B for the left motor

// for the battery monitor, hook it up with the + side soldered to Vout on the motor driver (+12)
// the negative side to ground and the middle to A5.  With the battery attached, 
// measure the voltage on Vout and divide it by the voltage on A5
// and enter that as the parameter VOLTAGE_DIVIDER_RATIO (assuming 22K and 10K resistors, the value should be 3.2)
// Also measure the fully charged battery value and enter that with the discharged value into the parameters
// FULL_BATTERY_VOLTAGE and ZERO_PERCENT_BATTERY_VOLTAGE

// command form is a letter for direction:
// W, S, A, D, = move forward, move backward, right turn, left turn.
// case does not matter, so w, a, s, d are OK
// followed by the speed to move, 0 to 255
// followed by a character to indicate that the input is complete, in our case that character is #
// for example, move with a speed of 200 is
// W200#
// to turn right at full speed is
// D255#
// if you wish to use the default speed (a defined value), then just the letters is sufficient:
// D#
// 
// for servos
// U, N, K, H = move up, down, right, left
// J to center and M for max down
// followed by the number of steps to take, for example, to move 2 steps up:
// U2#
// if there is no number, then the servo will just move by one step (TILT_DELTA or PAN_DELTA)
// U#

#include <threeMotorsPololuBigDriver.h>
#include <DualVNH5019MotorShield.h>
#include <amarinoMega.h>
// timers on the mega:
  // timer 0 pins A,B are 13,4
  // timer 1 pins A,B are 11,12
  // timer 2 pins A,B are 10,9  
  // timer 3 pins A,B,C are 5,2,3
  // timer 4 pins A,B,C are 6,7,8
  // timer 5 pins A,B,C are 44,45,46
  

#define SERIAL_SPEED 115200
#define BLUETOOTH_SPEED 115200
#define TIMED_OUT 8000
#define DEFAULT_SPEED 220
#define BW_REDUCTION 50
#define DEFAULT_TILT_SPEED 240
#define TD_REDUCTION 50
#define DEFAULT_TURN_FOREVER_SPEED 220
#define MOVE_TIME 100
#define TILT_TIME 100
#define MIN_ACCEL_SPEED 100
#define MIN_DECEL_SPEED 60
#define DELTA_SPEED 10
#define ACCEL_DELAY 200
#define LEFT_MOTOR_BIAS 10
#define LEFT_MOTOR_BW_BIAS 23
#define LEFT_MOTOR_STOP_DELAY 0

//#define TILT_CENTER 85
//#define TILT_LOOK_DOWN 135
//#define TILT_MIN 55
//#define TILT_MAX 165
//#define TILT_DELTA 10

#define BATTERY_MONITOR_PIN A5
#define ZERO_PERCENT_BATTERY_VOLTAGE 10.5
#define FULL_BATTERY_VOLTAGE 13.0
#define VOLTAGE_DIVIDER_RATIO 8.21

#define DUAL_MOTOR_DRIVER_MAX 400
#define DUAL_MOTOR_DRIVER_MIN -400

amarinoMega amarino;
threeMotorsPololuBigDriver tiltMotorDriver;
DualVNH5019MotorShield driveMotorDriver;

bool Moving, brakesOn;
long timeOutCheck;
float batteryRange;
int mySpeed;

void checkBattery(byte flag, byte numOfValues)
{
  float voltage =  (float) ((analogRead(BATTERY_MONITOR_PIN) / 1023.) * 5.0 ) * VOLTAGE_DIVIDER_RATIO;
  int batteryPercent =  (int) ( 100. * ( ( voltage - ZERO_PERCENT_BATTERY_VOLTAGE) / batteryRange)); // returns percentage
  if (batteryPercent > 99) batteryPercent = 100;
  if (batteryPercent < 0) batteryPercent = 0;
  //batteryPercent = analogRead(batteryMonitorPin);  // for testing
  amarino.send(batteryPercent); 
} 

// pololu driver goes from -400 to 400, but our input goes from -255 to 255, so ew have to convert the values
// this is a custom form of the arduino map function
int convert(int x)
{
  //return (x - BYTE_VALUE_MIN) * (MOTOR_DRIVER_MAX - MOTOR_DRIVER_MIN) / (BYTE_VALUE_MAX - BYTE_VALUE_MIN) + MOTOR_DRIVER_MIN;
  //return (x + 255) * 800 / 510) - 400;
  return (int) ( (((float)(x + 255)) * 1.57 ) ) - 400;
}

bool stopIfFault()
{
  bool result = false;
  if (driveMotorDriver.getM1Fault())
  {
      driveMotorDriver.setSpeeds(0,0);
      Serial.println("Fault detected in Motor 1 ");
      result = true;
  }
  if (driveMotorDriver.getM2Fault())
  {
      driveMotorDriver.setSpeeds(0,0);
      Serial.println("Fault detected in Motor 2 ");
      result = true;
  }
  if (tiltMotorDriver.getStatusC())
  {
      tiltMotorDriver.setBrakesC();
      Serial.println("Fault detected in top motor ");
      result = true;
  }
  return result;
}

void coast()
{
  driveMotorDriver.setBrakes(0,0);
  brakesOn = false;
}

void brakes()
{
  driveMotorDriver.setBrakes(DUAL_MOTOR_DRIVER_MAX, DUAL_MOTOR_DRIVER_MAX);
  brakesOn = true;
}

void Stop()
{
  driveMotorDriver.setSpeeds(0,0);
  brakes();
  brakesOn = true;
  Moving = false;
}
void stopCallback(byte flag, byte numOfValues)
{
  Stop();
  char message = 'x';
  amarino.send(message);
}

void accelerate(int targetSpeed)
{
  int goSpeed = MIN_ACCEL_SPEED;
  if (targetSpeed > 0)
  {
    while (goSpeed < targetSpeed - DELTA_SPEED)
    {
      driveMotorDriver.setSpeeds(convert(goSpeed + LEFT_MOTOR_BIAS), convert(goSpeed));
      //Serial.println("moving, speed = ");
      //Serial.println(goSpeed);
      goSpeed += DELTA_SPEED;
      delay(ACCEL_DELAY);
    }
  }
  else
  {
    targetSpeed = -targetSpeed;
    while (goSpeed < targetSpeed - DELTA_SPEED)
    {
      driveMotorDriver.setSpeeds(convert(-goSpeed - LEFT_MOTOR_BIAS), convert(-goSpeed));
      //Serial.println("moving, speed = ");
      //Serial.println(-goSpeed);
      goSpeed += DELTA_SPEED;
      delay(ACCEL_DELAY);
    }
  }
}

void decelerate(int initialSpeed)
{
  int goSpeed;
  if (initialSpeed > 0)
  {
    goSpeed = initialSpeed - DELTA_SPEED;
    while (goSpeed > MIN_DECEL_SPEED + DELTA_SPEED)
    {
      driveMotorDriver.setSpeeds(convert(goSpeed + LEFT_MOTOR_BIAS), convert(goSpeed));
      //Serial.println("moving, speed = ");
      //Serial.println(goSpeed);
      goSpeed -= DELTA_SPEED;
      delay(ACCEL_DELAY);
    }
  }
  else
  {
    initialSpeed = -initialSpeed;
    goSpeed = initialSpeed - DELTA_SPEED;
    while (goSpeed > MIN_DECEL_SPEED + DELTA_SPEED)
    {
      driveMotorDriver.setSpeeds(convert(-goSpeed - LEFT_MOTOR_BIAS), convert(-goSpeed));
      //Serial.println("moving, speed = ");
      //Serial.println(-goSpeed);
      goSpeed -= DELTA_SPEED;
      delay(ACCEL_DELAY);
    }
  }    
}
  
// calling moveForward with speed 0 causes motors to coast to stop
// while calling Stop() causes motors to stop and brake.

// calling moveForward with speed 0 causes motors to coast to stop
// while calling Stop() causes motors to stop and brake.

void moveForwardaLittle(byte flag, byte numOfValues)
{
  mySpeed = DEFAULT_SPEED; //amarino.getInt(); // speed goes from 0 to 255
  Serial.println("moving, speed = ");
  Serial.println(mySpeed);
  if (brakesOn) coast();
  driveMotorDriver.setSpeeds(convert(mySpeed + LEFT_MOTOR_BIAS), convert(mySpeed));
  if (mySpeed == 0 || stopIfFault()) Moving = false;
  else Moving = true;
  char message = 't';
  amarino.send(message);
  timeOutCheck = millis();
  delay(300);
  Stop();
}

void moveForward(byte flag, byte numOfValues)
{
  mySpeed = DEFAULT_SPEED; //amarino.getInt(); // speed goes from 0 to 255
  Serial.println("moving, speed = ");
  Serial.println(mySpeed);
  if (brakesOn) coast();
  accelerate(mySpeed);
  driveMotorDriver.setSpeeds(convert(mySpeed + LEFT_MOTOR_BIAS), convert(mySpeed));
  if (mySpeed == 0 || stopIfFault()) Moving = false;
  else Moving = true;
  char message = 'f';
  amarino.send(message);
  timeOutCheck = millis();
  delay(MOVE_TIME);
  decelerate(mySpeed);
  Stop();
}

void moveForwardForever(byte flag, byte numOfValues)
{
  mySpeed = DEFAULT_SPEED; //amarino.getInt(); // speed goes from 0 to 255
  Serial.println("moving, speed = ");
  Serial.println(mySpeed);
  if (brakesOn) coast();
  accelerate(mySpeed);
  driveMotorDriver.setSpeeds(convert(mySpeed + LEFT_MOTOR_BIAS), convert(mySpeed));
  if (mySpeed == 0 || stopIfFault()) Moving = false;
  else Moving = true;
  char message = 'F';
  amarino.send(message);
  timeOutCheck = millis();
}

void moveBackwardaLittle(byte flag, byte numOfValues)
{
  mySpeed = -DEFAULT_SPEED; //amarino.getInt(); // speed goes from 0 to 255
  Serial.println("moving, speed = ");
  Serial.println(mySpeed);
  if (brakesOn) coast();
  accelerate(mySpeed);
  driveMotorDriver.setSpeeds(convert(mySpeed - LEFT_MOTOR_BIAS), convert(mySpeed));
  if (mySpeed == 0 || stopIfFault()) Moving = false;
  else Moving = true;
  char message = 'g';
  amarino.send(message);
  timeOutCheck = millis();
  delay(300);
  decelerate(mySpeed);
  Stop();
}

void moveBackward(byte flag, byte numOfValues)
{
  mySpeed = -DEFAULT_SPEED; //amarino.getInt(); // speed goes from 0 to 255
  Serial.println("moving, speed = ");
  Serial.println(mySpeed);
  if (brakesOn) coast();
  accelerate(mySpeed);
  driveMotorDriver.setSpeeds(convert(mySpeed - LEFT_MOTOR_BIAS), convert(mySpeed));
  if (mySpeed == 0 || stopIfFault()) Moving = false;
  else Moving = true;
  char message = 'b';
  amarino.send(message);
  timeOutCheck = millis();
  delay(MOVE_TIME);
  decelerate(mySpeed);
  Stop();
}

void moveBackwardForever(byte flag, byte numOfValues)
{
  mySpeed = -DEFAULT_SPEED; //amarino.getInt(); // speed goes from 0 to 255
  Serial.println("moving, speed = ");
  Serial.println(mySpeed);
  if (brakesOn) coast();
  accelerate(mySpeed);
  driveMotorDriver.setSpeeds(convert(mySpeed - LEFT_MOTOR_BIAS), convert(mySpeed));
  if (mySpeed == 0 || stopIfFault()) Moving = false;
  else Moving = true;
  char message = 'B';
  amarino.send(message);
  timeOutCheck = millis();
}

void turnRightForever(byte flag, byte numOfValues) // speed goes from 0 to 255
{
  mySpeed = DEFAULT_TURN_FOREVER_SPEED; //amarino.getInt(); // speed goes from 0 to 255
  Serial.println("turning, speed = ");
  Serial.println(mySpeed);
  if (brakesOn) coast();
  driveMotorDriver.setSpeeds(convert(mySpeed), convert(-mySpeed));
  if (mySpeed == 0 || stopIfFault()) Moving = false;
  else Moving = true;
  char message = 'R';
  amarino.send(message);
  timeOutCheck = millis(); 
}

void turnRight(byte flag, byte numOfValues) // speed goes from 0 to 255
{
  mySpeed = DEFAULT_SPEED; //amarino.getInt(); // speed goes from 0 to 255
  Serial.println("turning, speed = ");
  Serial.println(mySpeed);
  if (brakesOn) coast();
  driveMotorDriver.setSpeeds(convert(mySpeed), convert(-mySpeed));
  if (mySpeed == 0 || stopIfFault()) Moving = false;
  else Moving = true;
  char message = 'r';
  amarino.send(message);
  timeOutCheck = millis();
  delay(200);
  Stop();  
}

void turnLeftForever(byte flag, byte numOfValues) // speed goes from 0 to 255
{
  mySpeed = -DEFAULT_TURN_FOREVER_SPEED; //amarino.getInt(); // speed goes from 0 to 255
  Serial.println("turning, speed = ");
  Serial.println(mySpeed);
  if (brakesOn) coast();
  driveMotorDriver.setSpeeds(convert(mySpeed), convert(-mySpeed));
  if (mySpeed == 0 || stopIfFault()) Moving = false;
  else Moving = true;
  char message = 'L';
  amarino.send(message);
  timeOutCheck = millis();
}


void turnLeft(byte flag, byte numOfValues) // speed goes from 0 to 255
{
  mySpeed = -DEFAULT_SPEED; //amarino.getInt(); // speed goes from 0 to 255
  Serial.println("turning, speed = ");
  Serial.println(mySpeed);
  if (brakesOn) coast();
  driveMotorDriver.setSpeeds(convert(mySpeed), convert(-mySpeed));
  if (mySpeed == 0 || stopIfFault()) Moving = false;
  else Moving = true;
  char message = 'l';
  amarino.send(message);
  timeOutCheck = millis();
  delay(200);
  Stop();    
}

void getMotorCurrents()
{
  int currentM1 = driveMotorDriver.getM1CurrentMilliamps();
  int currentM2 = driveMotorDriver.getM2CurrentMilliamps();
  int currentTopMotor = tiltMotorDriver.getCurrentC();
}

void tiltUp(byte flag, byte numOfValues) // speed goes from 0 to 255
{
  mySpeed = DEFAULT_TILT_SPEED; //amarino.getInt(); // speed goes from 0 to 255
  Serial.println("tilting up, speed = ");
  Serial.println(mySpeed);
  tiltMotorDriver.setSpeedC(mySpeed);
  //if (mySpeed == 0 || stopIfFault()) Moving = false;
  //else Moving = true;
  char message = 'u';
  amarino.send(message);
  timeOutCheck = millis();
  delay(TILT_TIME);
  tiltMotorDriver.setCoastC();    
}


void tiltDown(byte flag, byte numOfValues) // speed goes from 0 to 255
{
  mySpeed = -DEFAULT_TILT_SPEED; //amarino.getInt(); // speed goes from 0 to 255
  Serial.println("tilting dwon, speed = ");
  Serial.println(mySpeed);
  tiltMotorDriver.setSpeedC(mySpeed);
  //if (mySpeed == 0 || stopIfFault()) Moving = false;
  //else Moving = true;
  char message = 'n';
  amarino.send(message);
  timeOutCheck = millis();
  delay(TILT_TIME);
  tiltMotorDriver.setCoastC();   
}

void commCheck(byte flag, byte numOfValues)
{
  //const char message[3] = {'O', 'K', '/0'};
  //char message = 67;
  const char message[] = "comm check good";  // works
  //long value= 110;
  //int base = 2;  
  //amarino.send(n,m); // sends 110 in base 2 = 1101110
  
  // to convert a float to a character array:
  //dtostrf(value, width, precision, output);
  //where value is the float value you wish to convert,
  //width is the minimum number of characters to display before the decimal point
  //(padding with spaces as needed),
  //precision is the number of decimal places,
  //and output is the character array to put the results in.
  
  amarino.send(message);
}

 
void setup()  
{
  Serial.begin(SERIAL_SPEED);
  SERIAL_PORT_BLUETOOTH.begin(BLUETOOTH_SPEED);   // connect to bluetooth on serial1 
  driveMotorDriver.setBrakes(0,0);  // start with coasting
  brakesOn = false;
  tiltMotorDriver.setBrakesC();  // start with brakes on tilt motor
  
  batteryRange = FULL_BATTERY_VOLTAGE - ZERO_PERCENT_BATTERY_VOLTAGE;
  
  // register callback functions, which will be called when an associated event occurs.
  amarino.registerFunction(moveForwardaLittle, 't');
  amarino.registerFunction(moveForward, 'f');
  amarino.registerFunction(moveForwardForever, 'F');
  amarino.registerFunction(moveBackwardaLittle, 'g');
  amarino.registerFunction(moveBackward, 'b');
  amarino.registerFunction(moveBackwardForever, 'B');  
  amarino.registerFunction(turnRightForever, 'R'); 
  amarino.registerFunction(turnRight, 'r');
  amarino.registerFunction(turnLeft, 'l');
  amarino.registerFunction(turnLeftForever, 'L');
  amarino.registerFunction(tiltUp, 'u');
  amarino.registerFunction(tiltDown, 'n');  
  //amarino.registerFunction(servoCenter, 'j');
  //amarino.registerFunction(servoMaxDown, 'm');
  amarino.registerFunction(stopCallback, 'x');
  amarino.registerFunction(checkBattery, 'p');
  amarino.registerFunction(commCheck, 'c');
}

void loop()
{
  amarino.receive(); // you need to keep this in your loop() to receive events
  long currentTime = millis();
 // if (currentTime - timeOutCheck > TIMED_OUT && Moving) Stop();  //if we are moving and haven't heard anything in a long time, stop moving
  delay(50); // some delay keeps the tablet or phone from getting too busy with this
}


