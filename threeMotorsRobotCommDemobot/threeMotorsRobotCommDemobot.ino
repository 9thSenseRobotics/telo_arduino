// accepts serial port inputs and responds with moves and tilts
// requires a Mega to work, not an Uno or Nano, due to the need for two serial inputs

// wiring:
// motor A is the left motor, motor B the right.
// hook them both up so that the outside wire from the motor goes into the outside terminal
// that corresponds to M1A for the right motor, M2B for the left motor
// see the comments in threeMotorsDriver.cpp for details.

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

#include <threeMotorsDriver.h> 

#define SERIAL_PORT Serial
#define SERIAL_PORT_BLUETOOTH Serial2
#define SERIAL_SPEED 115200
#define BLUETOOTH_SPEED 57600
#define INPUT_BUFFER_SIZE 256
#define COMMAND_END_CHARACTER '#'
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

threeMotorsDriver motorDriver;

char inputBuffer[INPUT_BUFFER_SIZE], charIn;
int tiltPos, inputLength, mySpeed; 
bool Moving, brakesOn;
long timeOutCheck;
float batteryRange;

void checkBattery()
{
  float voltage =  (float) ((analogRead(BATTERY_MONITOR_PIN) / 1023.) * 5.0 ) * VOLTAGE_DIVIDER_RATIO;
  int batteryPercent =  (int) ( 100. * ( ( voltage - ZERO_PERCENT_BATTERY_VOLTAGE) / batteryRange)); // returns percentage
  if (batteryPercent > 99) batteryPercent = 100;
  if (batteryPercent < 0) batteryPercent = 0;
  //batteryPercent = analogRead(batteryMonitorPin);  // for testing
  SERIAL_PORT_BLUETOOTH.println(batteryPercent); 
} 

bool stopIfFault()
{
  bool result = false;
  if (motorDriver.getStatusA())
  {
      motorDriver.setBrakesAB();
      SERIAL_PORT.println("Fault detected in left motor ");
      result = true;
  }
  if (motorDriver.getStatusB())
  {
      motorDriver.setBrakesAB();
      SERIAL_PORT.println("Fault detected in right motor ");
      result = true;
  }
  
  if (motorDriver.getStatusC())
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
  int goSpeed = MIN_ACCEL_SPEED;
  if (targetSpeed > 0)
  {
    while (goSpeed < targetSpeed - DELTA_SPEED)
    {
      motorDriver.setSpeedAB(goSpeed + LEFT_MOTOR_BIAS,goSpeed);
      //SERIAL_PORT.println("moving, speed = ");
      //SERIAL_PORT.println(goSpeed);
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
      //SERIAL_PORT.println("moving, speed = ");
      //SERIAL_PORT.println(-goSpeed);
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
      //SERIAL_PORT.println("moving, speed = ");
      //SERIAL_PORT.println(goSpeed);
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
      //SERIAL_PORT.println("moving, speed = ");
      //SERIAL_PORT.println(-goSpeed);
      goSpeed -= DELTA_SPEED;
      delay(ACCEL_DELAY);
    }
  }    
}
  
// calling moveForward with speed 0 causes motors to coast to stop
// while calling Stop() causes motors to stop and brake.

void moveForwardaLittle(int mySpeed)
{
  //mySpeed = DEFAULT_SPEED;  // speed goes from 0 to 255
  SERIAL_PORT.print("moving, speed = ");
  SERIAL_PORT.println(mySpeed);
  if (brakesOn) coast();
  motorDriver.setSpeedAB(mySpeed + LEFT_MOTOR_BIAS, mySpeed);
  if (mySpeed == 0 || stopIfFault()) Moving = false;
  else Moving = true;
  timeOutCheck = millis();
  delay(300);
  Stop();
}

void moveForward(int mySpeed)
{
  //mySpeed = DEFAULT_SPEED;  // speed goes from 0 to 255
  SERIAL_PORT.print("moving, speed = ");
  SERIAL_PORT.println(mySpeed);
  if (brakesOn) coast();
//  accelerate(mySpeed);
  motorDriver.setSpeedAB(mySpeed + LEFT_MOTOR_BIAS, mySpeed);
  if (mySpeed == 0 || stopIfFault()) Moving = false;
  else Moving = true;
  timeOutCheck = millis();
  delay(MOVE_TIME);
//  decelerate(mySpeed);
  Stop();
}

void moveForwardForever(int mySpeed)
{
  //mySpeed = DEFAULT_SPEED;  // speed goes from 0 to 255
  SERIAL_PORT.print("moving, speed = ");
  SERIAL_PORT.println(mySpeed);
  if (brakesOn) coast();
//  accelerate(mySpeed);
  motorDriver.setSpeedAB(mySpeed + LEFT_MOTOR_BIAS, mySpeed);
  if (mySpeed == 0 || stopIfFault()) Moving = false;
  else Moving = true;
  timeOutCheck = millis(); 
}

void moveBackwardaLittle(int mySpeed)
{
  mySpeed = -DEFAULT_SPEED + BW_REDUCTION;  // speed goes from 0 to 255
  SERIAL_PORT.print("moving, speed = ");
  SERIAL_PORT.println(mySpeed);
  if (brakesOn) coast();
  //accelerate(mySpeed);
  motorDriver.setSpeedAB(mySpeed - LEFT_MOTOR_BW_BIAS, mySpeed);
  if (mySpeed == 0 || stopIfFault()) Moving = false;
  else Moving = true;
  timeOutCheck = millis();
  delay(300);
  //decelerate(mySpeed);
  Stop();
}

void moveBackward(int mySpeed)
{
  mySpeed = -DEFAULT_SPEED + BW_REDUCTION;  // speed goes from 0 to 255// backward should be slower than forward
  SERIAL_PORT.print("moving, speed = ");
  SERIAL_PORT.println(mySpeed);
  if (brakesOn) coast();
  //accelerate(mySpeed);
  motorDriver.setSpeedAB(mySpeed - LEFT_MOTOR_BW_BIAS, mySpeed);
  if (mySpeed == 0 || stopIfFault()) Moving = false;
  else Moving = true;
  timeOutCheck = millis();
  delay(MOVE_TIME);
  //decelerate(mySpeed);
  Stop();
}

void moveBackwardForever(int mySpeed)
{
  mySpeed = -DEFAULT_SPEED + BW_REDUCTION;  // speed goes from 0 to 255 // backward should be slower than forward
  SERIAL_PORT.print("moving, speed = ");
  SERIAL_PORT.println(mySpeed);
  if (brakesOn) coast();
  //accelerate(mySpeed);
  motorDriver.setSpeedAB(mySpeed - LEFT_MOTOR_BW_BIAS, mySpeed);
  if (mySpeed == 0 || stopIfFault()) Moving = false;
  else Moving = true;
  timeOutCheck = millis();
}

void turnRightForever(int mySpeed) // speed goes from 0 to 255
{
  mySpeed = -DEFAULT_TURN_FOREVER_SPEED; 
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
  mySpeed = -DEFAULT_SPEED;  
  SERIAL_PORT.print("turning, speed = ");
  SERIAL_PORT.println(mySpeed);
  if (brakesOn) coast();
  motorDriver.setSpeedAB(mySpeed,-mySpeed);
  if (mySpeed == 0 || stopIfFault()) Moving = false;
  else Moving = true;
  timeOutCheck = millis();
  delay(200);
  Stop();  
}

void turnLeftForever(int mySpeed) // speed goes from 0 to 255
{
  mySpeed = DEFAULT_TURN_FOREVER_SPEED; 
  SERIAL_PORT.print("turning, speed = ");
  SERIAL_PORT.println(mySpeed);
  if (brakesOn) coast();
  motorDriver.setSpeedAB(mySpeed,-mySpeed);
  if (mySpeed == 0 || stopIfFault()) Moving = false;
  else Moving = true;
  timeOutCheck = millis();
}


void turnLeft(int mySpeed) // speed goes from 0 to 255
{
  mySpeed = DEFAULT_SPEED; 
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


void tiltUp(int myDistance) // distance goes from 0 to 255
{
  myDistance = TILT_TIME;
  mySpeed = DEFAULT_TILT_SPEED; 
  SERIAL_PORT.print("tilting up, distance = ");
  SERIAL_PORT.println(myDistance);
  motorDriver.setSpeedC(mySpeed);
  //if (mySpeed == 0 || stopIfFault()) Moving = false;
  //else Moving = true;
  timeOutCheck = millis();
  delay(TILT_TIME);
  motorDriver.setCoastC();    
}


void tiltDown(int myDistance) // distance goes from 0 to 255
{
  myDistance = TILT_TIME;
  mySpeed = -DEFAULT_TILT_SPEED; 
  SERIAL_PORT.print("tilting dwon, distance = ");
  SERIAL_PORT.println(myDistance);
  motorDriver.setSpeedC(mySpeed);
  //if (mySpeed == 0 || stopIfFault()) Moving = false;
  //else Moving = true;
  timeOutCheck = millis();
  delay(myDistance);
  motorDriver.setCoastC();   
}

void getMotorCurrents()
{
  int currentLeftMotor = motorDriver.getCurrentA();
  int currentRightMotor = motorDriver.getCurrentB();
  int currentTopMotor = motorDriver.getCurrentC();
}

// process a command string
void HandleCommand(char* input, int length)
{
  int speedToGo = DEFAULT_SPEED;
  int stepsToGo = 1;  // servo steps
  int value = 0;
  // calculate number following command
  if (length > 1)
  {
    value = atoi(&input[1]);
    speedToGo = value;                  // either servo steps or speed specified
    stepsToGo = value;
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
      moveForward(speedToGo);
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
      tiltUp(stepsToGo);
      break;
    case 'n':    // tilt down
      tiltDown(stepsToGo);
      break;
    case 'p':
      checkBattery(); // note that this writes a single char, so value should be in range 0-255
      break;    
    default:
      SERIAL_PORT.println("did not recognize command ");
      break;
  }
} 
 
void setup()  
{
  SERIAL_PORT.begin(SERIAL_SPEED);
  SERIAL_PORT.println("ready for commands");
  SERIAL_PORT_BLUETOOTH.begin(BLUETOOTH_SPEED);   // usually connect to bluetooth on serial2
  motorDriver.setCoastAB();
  motorDriver.setBrakesC();
  
  batteryRange = FULL_BATTERY_VOLTAGE - ZERO_PERCENT_BATTERY_VOLTAGE;
  
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
      //if (millis() - timeOutCheck > TIMED_OUT && Moving) Stop();  //if we are moving and haven't heard anything in a long time, stop moving    
      delay(10);
    }
    charIn = SERIAL_PORT_BLUETOOTH.read(); // read it in
    if (charIn != 13 && charIn != 10 && charIn != 32)  // ignore carriage returns, line feeds, and spaces
    {
       inputBuffer[inputLength] = charIn;
       inputLength++;
       SERIAL_PORT.print(charIn);
    }
  } 
  
  //now flush anything that came after the COMMAND_END_CHARACTER
  while (SERIAL_PORT_BLUETOOTH.available()) charIn = SERIAL_PORT_BLUETOOTH.read(); // read it in
  
  // throw away the line end character
  inputLength -= 1;  // -1 because it is incremented after the last character
  inputBuffer[inputLength] = 0;  // change COMMAND_END_CHARACTER to a 0 (remember, index is one behind inputLength)
  
  SERIAL_PORT.print("commanded: ");
  SERIAL_PORT.println(inputBuffer);

  // if the command == 'c' it is just a comm check
  if (inputBuffer[0] != 'c' && inputLength > 0) HandleCommand(inputBuffer, inputLength);
   
  // echo the command
  SERIAL_PORT_BLUETOOTH.println(inputBuffer); // need to println because android uses the CR as a delimiter

  for (int i=0; i < inputLength; i++) inputBuffer[i] = 0;
  }


