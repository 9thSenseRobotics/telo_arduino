

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

#include <DualVNH5019MotorShield.h>
#include <Servo.h> 

// pins 0 and 1 are used for serial comm with the laptop
// pins used by the pololu shield are the following:
// motor1_inA pin 2
// motor1_inB pin 4
// motor2_inA pin 7
// motor2_inB pin 8
// motor1_PWM pin 9
// motor2_PWM pin 10
// motor1 diagnostic pin 6
// motor2 diagnostic pin 12
// motor1 current out pin A0
// motor2 current out pin A1
// charging detection pin 13

// timers on the mega:
  // timer 0 pins A,B are 13,4
  // timer 1 pins A,B are 11,12
  // timer 2 pins A,B are 10,9  
  // timer 3 pins A,B,C are 5,2,3
  // timer 4 pins A,B,C are 6,7,8
  // timer 5 pins A,B,C are 44,45,46
  
#define tiltPin 3

#define SERIAL_PORT Serial
#define SERIAL_PORT_BLUETOOTH Serial2
#define SERIAL_SPEED 115200
#define BLUETOOTH_SPEED 57600

#define INPUT_BUFFER_SIZE 256
#define COMMAND_END_CHARACTER '#'
#define COMM_CHECK_CHARACTER 'c'
#define MESSAGE_BATTERY_PERCENT mb

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

#define TILT_CENTER 85
#define TILT_LOOK_DOWN 135
#define TILT_MIN 55
#define TILT_MAX 165
#define TILT_DELTA 10

#define BATTERY_MONITOR_PIN A4
#define ZERO_PERCENT_BATTERY_VOLTAGE 10.5
#define FULL_BATTERY_VOLTAGE 13.0
#define VOLTAGE_DIVIDER_RATIO 8.21

#define MOTOR_DRIVER_MAX 400
#define MOTOR_DRIVER_MIN -400

DualVNH5019MotorShield motorDriver;

Servo tiltServo;  // create servo objects to control the servos
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
  SERIAL_PORT_BLUETOOTH.print("MESSAGE_BATTERY_PERCENT");  // lead with "mb"  
                                      // 'm' indicates that this is a message for the server
                                      // 'b' indicates that it is a battery percent messsage
  SERIAL_PORT_BLUETOOTH.println(batteryPercent); 
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
  if (motorDriver.getM1Fault())
  {
      motorDriver.setSpeeds(0,0);
      Serial.println("Fault detected in Motor 1 ");
      result = true;
  }
  if (motorDriver.getM2Fault())
  {
      motorDriver.setSpeeds(0,0);
      Serial.println("Fault detected in Motor 2 ");
      result = true;
  }
  return result;
}

void coast()
{
  motorDriver.setBrakes(0,0);
  brakesOn = false;
}

void brakes()
{
  motorDriver.setBrakes(MOTOR_DRIVER_MAX, MOTOR_DRIVER_MAX);
  brakesOn = true;
}

void Stop()
{
  motorDriver.setSpeeds(0,0);
  brakes();
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
      motorDriver.setSpeeds(convert(goSpeed + LEFT_MOTOR_BIAS), convert(goSpeed));
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
      motorDriver.setSpeeds(convert(-goSpeed - LEFT_MOTOR_BIAS), convert(-goSpeed));
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
      motorDriver.setSpeeds(convert(goSpeed + LEFT_MOTOR_BIAS), convert(goSpeed));
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
      motorDriver.setSpeeds(convert(-goSpeed - LEFT_MOTOR_BIAS), convert(-goSpeed));
      //Serial.println("moving, speed = ");
      //Serial.println(-goSpeed);
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
  motorDriver.setSpeeds(convert(mySpeed + LEFT_MOTOR_BIAS), convert(mySpeed));
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
  motorDriver.setSpeeds(convert(mySpeed + LEFT_MOTOR_BIAS), convert(mySpeed));
  if (mySpeed == 0 || stopIfFault()) Moving = false;
  else Moving = true;
  timeOutCheck = millis();
  delay(MOVE_TIME);
  decelerate(mySpeed);
  Stop();
}

void moveForwardForever(int mySpeed)
{
  //mySpeed = DEFAULT_SPEED;  // speed goes from 0 to 255
  SERIAL_PORT.print("moving, speed = ");
  SERIAL_PORT.println(mySpeed);
  if (brakesOn) coast();
//  accelerate(mySpeed);
  motorDriver.setSpeeds(convert(mySpeed + LEFT_MOTOR_BIAS), convert(mySpeed));
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
  motorDriver.setSpeeds(convert(mySpeed - LEFT_MOTOR_BIAS), convert(mySpeed));
  if (mySpeed == 0 || stopIfFault()) Moving = false;
  else Moving = true;
  timeOutCheck = millis();
  delay(300);
  decelerate(mySpeed);
  Stop();
}

void moveBackward(int mySpeed)
{
  mySpeed = -DEFAULT_SPEED + BW_REDUCTION;  // speed goes from 0 to 255// backward should be slower than forward
  SERIAL_PORT.print("moving, speed = ");
  SERIAL_PORT.println(mySpeed);
  if (brakesOn) coast();
  //accelerate(mySpeed);
  motorDriver.setSpeeds(convert(mySpeed - LEFT_MOTOR_BIAS), convert(mySpeed));
  if (mySpeed == 0 || stopIfFault()) Moving = false;
  else Moving = true;
  timeOutCheck = millis();
  delay(MOVE_TIME);
  decelerate(mySpeed);
  Stop();
}

void moveBackwardForever(int mySpeed)
{
  mySpeed = -mySpeed;
  SERIAL_PORT.print("moving, speed = ");
  SERIAL_PORT.println(mySpeed);
  if (brakesOn) coast();
  //accelerate(mySpeed);
  motorDriver.setSpeeds(convert(mySpeed - LEFT_MOTOR_BIAS), convert(mySpeed));
  if (mySpeed == 0 || stopIfFault()) Moving = false;
  else Moving = true;
  timeOutCheck = millis();
}
void turnRightForever(int mySpeed) 
{
  mySpeed = -DEFAULT_TURN_FOREVER_SPEED; 
  SERIAL_PORT.print("turning, speed = ");
  SERIAL_PORT.println(mySpeed);
  if (brakesOn) coast();
  motorDriver.setSpeeds(convert(mySpeed), convert(-mySpeed));
  if (mySpeed == 0 || stopIfFault()) Moving = false;
  else Moving = true;
  timeOutCheck = millis(); 
}

void turnRight(int mySpeed) // speed goes from 0 to 255
{
  mySpeed = -mySpeed;  
  SERIAL_PORT.print("turning, speed = ");
  SERIAL_PORT.println(mySpeed);
  if (brakesOn) coast();
  motorDriver.setSpeeds(convert(mySpeed), convert(-mySpeed));
  if (mySpeed == 0 || stopIfFault()) Moving = false;
  timeOutCheck = millis();
  delay(200);
  Stop();  
}

void turnLeftForever(int mySpeed) 
{
  mySpeed = DEFAULT_TURN_FOREVER_SPEED; 
  Serial.println("turning, speed = ");
  Serial.println(mySpeed);
  if (brakesOn) coast();
  motorDriver.setSpeeds(convert(mySpeed), convert(-mySpeed));
  if (mySpeed == 0 || stopIfFault()) Moving = false;
  else Moving = true;
  timeOutCheck = millis();
}


void turnLeft(int mySpeed) // speed goes from 0 to 255
{
  mySpeed = DEFAULT_TURN_FOREVER_SPEED;
  Serial.println("turning, speed = ");
  Serial.println(mySpeed);
  if (brakesOn) coast();
  motorDriver.setSpeeds(convert(mySpeed), convert(-mySpeed));
  if (mySpeed == 0 || stopIfFault()) Moving = false;
  else Moving = true;
  timeOutCheck = millis();
  delay(200);
  Stop();    
}

void getMotorCurrents()
{
  int currentM1 = motorDriver.getM1CurrentMilliamps();
  int currentM2 = motorDriver.getM2CurrentMilliamps();
}

void servoUp(int stepsToGo)
{
      Serial.println("Moving servo up");
      stepsToGo = 1; // for now we just move a fixed distance
      if (tiltPos - (TILT_DELTA *stepsToGo) >= TILT_MIN) tiltPos -= TILT_DELTA * stepsToGo;
      else tiltPos = TILT_MIN;
      tiltServo.write(tiltPos);
      timeOutCheck = millis();
}

void servoDown(int stepsToGo) 
{
      Serial.println("Moving servo down");
      stepsToGo = 1;  // for now we just move a fixed distance
      if (tiltPos - (TILT_DELTA * stepsToGo) <= TILT_MAX) tiltPos += TILT_DELTA * stepsToGo;
      else tiltPos = TILT_MAX;
      tiltServo.write(tiltPos);
      timeOutCheck = millis();
}

void servoCenter()
{
      tiltServo.write(TILT_CENTER);
      tiltPos = TILT_CENTER;
      timeOutCheck = millis();
}

void servoMaxDown()
{
      tiltPos = TILT_MIN;
      tiltServo.write(tiltPos);
      timeOutCheck = millis();
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
      servoUp(stepsToGo);
      break;
    case 'n':    // tilt down
      servoDown(stepsToGo);
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
  SERIAL_PORT_BLUETOOTH.begin(BLUETOOTH_SPEED);
  motorDriver.init();
  coast();
  
  batteryRange = FULL_BATTERY_VOLTAGE - ZERO_PERCENT_BATTERY_VOLTAGE;
    
  tiltServo.attach(tiltPin); 
  tiltServo.write(TILT_CENTER);
  tiltPos = TILT_CENTER;
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
       //SERIAL_PORT.print(charIn);
    }
  } 
  
  //now flush anything that came after the COMMAND_END_CHARACTER
  while (SERIAL_PORT_BLUETOOTH.available()) charIn = SERIAL_PORT_BLUETOOTH.read(); // read it in
  
  // throw away the line end character
  inputLength -= 1;  // -1 because it is incremented after the last character
  inputBuffer[inputLength] = 0;  // change COMMAND_END_CHARACTER to a 0 (remember, index is one behind inputLength)
  
 // SERIAL_PORT.print("commanded: ");
 // SERIAL_PORT.println(inputBuffer);

  // if the command == COMM_CHECK_CHARACTER it is just a local bluetooth comm check
  if (inputBuffer[0] != COMM_CHECK_CHARACTER && inputLength > 0) HandleCommand(inputBuffer, inputLength);
   
  // echo the command
  SERIAL_PORT_BLUETOOTH.println(inputBuffer); // need to println because android uses the CR as a delimiter
  for (int i = 0; i < inputLength; i++) inputBuffer[i] = 0;
}



