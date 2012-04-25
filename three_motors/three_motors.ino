

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

#include <triMotorDriver.h>
#include <MeetAndroid.h>

// timers on the mega:
  // timer 0 pins A,B are 13,4
  // timer 1 pins A,B are 11,12
  // timer 2 pins A,B are 10,9  
  // timer 3 pins A,B,C are 5,2,3
  // timer 4 pins A,B,C are 6,7,8
  // timer 5 pins A,B,C are 44,45,46
  
#define tiltPin 3
#define SerialSpeed 57600

#define TIMED_OUT 8000
#define DEFAULT_SPEED 140
#define DEFAULT_TURN_FOREVER_SPEED 100
#define MOVE_TIME 100
#define MIN_ACCEL_SPEED 100
#define MIN_DECEL_SPEED 60
#define DELTA_SPEED 10
#define ACCEL_DELAY 200
#define LEFT_MOTOR_BIAS 0
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

#define MOTOR_DRIVER_MAX 400
#define MOTOR_DRIVER_MIN -400

MeetAndroid meetAndroid;
triMotorDriver motorDriver;

//Servo tiltServo;  // create servo objects to control the servos
//int tiltPos;    // variable to store the servo position 
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
  meetAndroid.send(batteryPercent); 
} 

bool stopIfFault()
{
  bool result = false;
  if (motorDriver.getStatusA())
  {
      motorDriver.setBrakesAB();
      Serial.println("Fault detected in left motor ");
      result = true;
  }
  if (motorDriver.getStatusB())
  {
      motorDriver.setBrakesAB();
      Serial.println("Fault detected in right motor ");
      result = true;
  }
  
  if (motorDriver.getStatusC())
  {
      motorDriver.setBrakesC();
      Serial.println("Fault detected in top motor ");
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

void stopCallback(byte flag, byte numOfValues)
{
  Stop();
  char message = 'x';
  meetAndroid.send(message);
}

void accelerate(int targetSpeed)
{
  int goSpeed = MIN_ACCEL_SPEED;
  if (targetSpeed > 0)
  {
    while (goSpeed < targetSpeed - DELTA_SPEED)
    {
      motorDriver.setSpeedAB(goSpeed + LEFT_MOTOR_BIAS,goSpeed);
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
      motorDriver.setSpeedAB(-goSpeed - LEFT_MOTOR_BIAS,-goSpeed);
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
      motorDriver.setSpeedAB(goSpeed + LEFT_MOTOR_BIAS, goSpeed);
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
      motorDriver.setSpeedAB(-goSpeed - LEFT_MOTOR_BIAS, -goSpeed);
      //Serial.println("moving, speed = ");
      //Serial.println(-goSpeed);
      goSpeed -= DELTA_SPEED;
      delay(ACCEL_DELAY);
    }
  }    
}
  
// calling moveForward with speed 0 causes motors to coast to stop
// while calling Stop() causes motors to stop and brake.

void moveForwardaLittle(byte flag, byte numOfValues)
{
  mySpeed = DEFAULT_SPEED; //meetAndroid.getInt(); // speed goes from 0 to 255
  Serial.println("moving, speed = ");
  Serial.println(mySpeed);
  if (brakesOn) coast();
  motorDriver.setSpeedAB(mySpeed + LEFT_MOTOR_BIAS, mySpeed);
  if (mySpeed == 0 || stopIfFault()) Moving = false;
  else Moving = true;
  char message = 't';
  meetAndroid.send(message);
  timeOutCheck = millis();
  delay(300);
  Stop();
}

void moveForward(byte flag, byte numOfValues)
{
  mySpeed = DEFAULT_SPEED; //meetAndroid.getInt(); // speed goes from 0 to 255
  Serial.println("moving, speed = ");
  Serial.println(mySpeed);
  if (brakesOn) coast();
  accelerate(mySpeed);
  motorDriver.setSpeedAB(mySpeed + LEFT_MOTOR_BIAS, mySpeed);
  if (mySpeed == 0 || stopIfFault()) Moving = false;
  else Moving = true;
  char message = 'L';
  meetAndroid.send(message);
  timeOutCheck = millis();
  delay(MOVE_TIME);
  decelerate(mySpeed);
  Stop();
}

void moveForwardForever(byte flag, byte numOfValues)
{
  mySpeed = DEFAULT_SPEED; //meetAndroid.getInt(); // speed goes from 0 to 255
  Serial.println("moving, speed = ");
  Serial.println(mySpeed);
  if (brakesOn) coast();
  accelerate(mySpeed);
  motorDriver.setSpeedAB(mySpeed + LEFT_MOTOR_BIAS, mySpeed);
  if (mySpeed == 0 || stopIfFault()) Moving = false;
  else Moving = true;
  char message = 'F';
  meetAndroid.send(message);
  timeOutCheck = millis();
}

void moveBackwardaLittle(byte flag, byte numOfValues)
{
  mySpeed = -DEFAULT_SPEED; //meetAndroid.getInt(); // speed goes from 0 to 255
  Serial.println("moving, speed = ");
  Serial.println(mySpeed);
  if (brakesOn) coast();
  accelerate(mySpeed);
  motorDriver.setSpeedAB(mySpeed - LEFT_MOTOR_BIAS, mySpeed);
  if (mySpeed == 0 || stopIfFault()) Moving = false;
  else Moving = true;
  char message = 'g';
  meetAndroid.send(message);
  timeOutCheck = millis();
  delay(300);
  decelerate(mySpeed);
  Stop();
}

void moveBackward(byte flag, byte numOfValues)
{
  mySpeed = -DEFAULT_SPEED; //meetAndroid.getInt(); // speed goes from 0 to 255
  Serial.println("moving, speed = ");
  Serial.println(mySpeed);
  if (brakesOn) coast();
  accelerate(mySpeed);
  motorDriver.setSpeedAB(mySpeed - LEFT_MOTOR_BIAS, mySpeed);
  if (mySpeed == 0 || stopIfFault()) Moving = false;
  else Moving = true;
  char message = 'b';
  meetAndroid.send(message);
  timeOutCheck = millis();
  delay(MOVE_TIME);
  decelerate(mySpeed);
  Stop();
}

void moveBackwardForever(byte flag, byte numOfValues)
{
  mySpeed = -DEFAULT_SPEED; //meetAndroid.getInt(); // speed goes from 0 to 255
  Serial.println("moving, speed = ");
  Serial.println(mySpeed);
  if (brakesOn) coast();
  accelerate(mySpeed);
  motorDriver.setSpeedAB(mySpeed - LEFT_MOTOR_BIAS, mySpeed);
  if (mySpeed == 0 || stopIfFault()) Moving = false;
  else Moving = true;
  char message = 'B';
  meetAndroid.send(message);
  timeOutCheck = millis();
}

void turnRightForever(byte flag, byte numOfValues) // speed goes from 0 to 255
{
  mySpeed = DEFAULT_TURN_FOREVER_SPEED; //meetAndroid.getInt(); // speed goes from 0 to 255
  Serial.println("turning, speed = ");
  Serial.println(mySpeed);
  if (brakesOn) coast();
  motorDriver.setSpeedAB(mySpeed, -mySpeed);
  if (mySpeed == 0 || stopIfFault()) Moving = false;
  else Moving = true;
  char message = 'R';
  meetAndroid.send(message);
  timeOutCheck = millis(); 
}

void turnRight(byte flag, byte numOfValues) // speed goes from 0 to 255
{
  mySpeed = DEFAULT_SPEED; //meetAndroid.getInt(); // speed goes from 0 to 255
  Serial.println("turning, speed = ");
  Serial.println(mySpeed);
  if (brakesOn) coast();
  motorDriver.setSpeedAB(mySpeed,-mySpeed);
  if (mySpeed == 0 || stopIfFault()) Moving = false;
  else Moving = true;
  char message = 'r';
  meetAndroid.send(message);
  timeOutCheck = millis();
  delay(200);
  Stop();  
}

void turnLeftForever(byte flag, byte numOfValues) // speed goes from 0 to 255
{
  mySpeed = -DEFAULT_TURN_FOREVER_SPEED; //meetAndroid.getInt(); // speed goes from 0 to 255
  Serial.println("turning, speed = ");
  Serial.println(mySpeed);
  if (brakesOn) coast();
  motorDriver.setSpeedAB(mySpeed,-mySpeed);
  if (mySpeed == 0 || stopIfFault()) Moving = false;
  else Moving = true;
  char message = 'L';
  meetAndroid.send(message);
  timeOutCheck = millis();
}


void turnLeft(byte flag, byte numOfValues) // speed goes from 0 to 255
{
  mySpeed = -DEFAULT_SPEED; //meetAndroid.getInt(); // speed goes from 0 to 255
  Serial.println("turning, speed = ");
  Serial.println(mySpeed);
  if (brakesOn) coast();
  motorDriver.setSpeedAB(mySpeed, -mySpeed);
  if (mySpeed == 0 || stopIfFault()) Moving = false;
  else Moving = true;
  char message = 'l';
  meetAndroid.send(message);
  timeOutCheck = millis();
  delay(200);
  Stop();    
}

void getMotorCurrents()
{
  int currentLeftMotor = motorDriver.getCurrentA();
  int currentRightMotor = motorDriver.getCurrentB();
  int currentTopMotor = motorDriver.getCurrentC();
}

/*
void servoUp(byte flag, byte numOfValues)
{
      Serial.println("Moving servo up");
      int stepsToGo = 1; //meetAndroid.getInt();
      if (tiltPos - (TILT_DELTA *stepsToGo) >= TILT_MIN) tiltPos -= TILT_DELTA * stepsToGo;
      else tiltPos = TILT_MIN;
      tiltServo.write(tiltPos);
      char message = 'u';
      meetAndroid.send(message);
      timeOutCheck = millis();
}

void servoDown(byte flag, byte numOfValues) 
{
      Serial.println("Moving servo down");
      int stepsToGo = 1;//meetAndroid.getInt();
      if (tiltPos - (TILT_DELTA * stepsToGo) <= TILT_MAX) tiltPos += TILT_DELTA * stepsToGo;
      else tiltPos = TILT_MAX;
      tiltServo.write(tiltPos);
      char message = 'n';
      meetAndroid.send(message);
      timeOutCheck = millis();
}

void servoCenter(byte flag, byte numOfValues)
{
      tiltServo.write(TILT_CENTER);
      tiltPos = TILT_CENTER;
      char message = 'j';
      meetAndroid.send(message);
      timeOutCheck = millis();
}

void servoMaxDown(byte flag, byte numOfValues)
{
      tiltPos = TILT_MIN;
      tiltServo.write(tiltPos);
      char message = 'm';
      meetAndroid.send(message);
      timeOutCheck = millis();
}
*/
void commCheck(byte flag, byte numOfValues)
{
  //const char message[3] = {'O', 'K', '/0'};
  //char message = 67;
  const char message[] = "comm check good";  // works
  //long value= 110;
  //int base = 2;  
  //meetAndroid.send(n,m); // sends 110 in base 2 = 1101110
  
  // to convert a float to a character array:
  //dtostrf(value, width, precision, output);
  //where value is the float value you wish to convert,
  //width is the minimum number of characters to display before the decimal point
  //(padding with spaces as needed),
  //precision is the number of decimal places,
  //and output is the character array to put the results in.
  
  meetAndroid.send(message);
}

 
void setup()  
{
  Serial.begin(SerialSpeed);
  Serial1.begin(SerialSpeed);   // connect to bluetooth on serial1 
  motorDriver.setCoastAB();
  motorDriver.setBrakesC();
  
  batteryRange = FULL_BATTERY_VOLTAGE - ZERO_PERCENT_BATTERY_VOLTAGE;
  
  /*
  tiltServo.attach(tiltPin); 
  tiltServo.write(TILT_CENTER);
  tiltPos = TILT_CENTER;
  */
  
  // register callback functions, which will be called when an associated event occurs.
  meetAndroid.registerFunction(moveForwardaLittle, 't');
  meetAndroid.registerFunction(moveForward, 'f');
  meetAndroid.registerFunction(moveForwardForever, 'F');
  meetAndroid.registerFunction(moveBackwardaLittle, 'g');
  meetAndroid.registerFunction(moveBackward, 'b');
  meetAndroid.registerFunction(moveBackwardForever, 'B');  
  meetAndroid.registerFunction(turnRightForever, 'R'); 
  meetAndroid.registerFunction(turnRight, 'r');
  meetAndroid.registerFunction(turnLeft, 'l');
  meetAndroid.registerFunction(turnLeftForever, 'L');
  //meetAndroid.registerFunction(servoUp, 'u');
  //meetAndroid.registerFunction(servoDown, 'n');  
  //meetAndroid.registerFunction(servoCenter, 'j');
  //meetAndroid.registerFunction(servoMaxDown, 'm');
  meetAndroid.registerFunction(stopCallback, 'x');
  meetAndroid.registerFunction(checkBattery, 'p');
  meetAndroid.registerFunction(commCheck, 'c');
}

void loop()
{
  meetAndroid.receive(); // you need to keep this in your loop() to receive events
  long currentTime = millis();
 // if (currentTime - timeOutCheck > TIMED_OUT && Moving) Stop();  //if we are moving and haven't heard anything in a long time, stop moving
  delay(50); // some delay keeps the tablet or phone from getting too busy with this
}


