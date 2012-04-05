
// accepts serial port inputs and responds with moves and servos
// requires a Mega to work, not an Uno or Nano, due to conflicts with the pololu motor board
// and the servo library (both want exclusive use of pins 9 and 10).

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

#include <VNH5019_motor_driver.h>
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

// timers on the mega:
  // timer 0 pins A,B are 13,4
  // timer 1 pins A,B are 11,12
  // timer 2 pins A,B are 10,9  
  // timer 3 pins A,B,C are 5,2,3
  // timer 4 pins A,B,C are 6,7,8
  // timer 5 pins A,B,C are 44,45,46
  
#define tiltPin 3
#define panPin 5
#define SerialSpeed 9600
#define BufferLength 16
#define LineEndCharacter '#' // serial input commands must end with this character

//encoder pins
#define encoderLeftRed 23
#define encoderLeftGreen 25
#define encoderLeftYellow 27
#define encoderLeftBlack 37
#define encoderRightRed 29
#define encoderRightGreen 31
#define encoderRightYellow 33
#define encoderRightBlack 35

// multicolor LED
#define OFF 0
#define RED 1
#define GREEN 2
#define BLUE 3
#define LEDground 47
#define LEDgreen 49
#define LEDblue 51
#define LEDred 53

#define TIMED_OUT 8000
#define DEFAULT_SPEED 100

#define TILT_CENTER 55
#define TILT_LOOK_DOWN 90
#define TILT_MIN 135
#define TILT_MAX 40
#define TILT_DELTA 10

#define PAN_CENTER 93
#define PAN_MIN 0
#define PAN_MAX 180
#define PAN_DELTA 10

#define BATTERY_MONITOR_PIN A8
#define BATTERY_MONITOR_GROUND_PIN 46
#define ZERO_PERCENT_BATTERY_VOLTAGE 11.5
#define FULL_BATTERY_VOLTAGE 13.5
#define VOLTAGE_DIVIDER_RATIO 3.2

VNH5019_motor_driver motorDriver;
Servo panServo, tiltServo;  // create servo objects to control the servos
int panPos, tiltPos;    // variable to store the servo position 
char inputBuffer[BufferLength];
bool Moving, brakesOn;
long timeOutCheck;
int batteryMonitorPin;
float batteryRange;
volatile unsigned long encoderRight, encoderLeft;
long currentLeftMotor, currentRightMotor;

char checkBattery()
{
  float voltage =  (float) ((analogRead(batteryMonitorPin) / 1023.) * 5.0 ) * VOLTAGE_DIVIDER_RATIO;
  char batteryPercent =  (char) ( 100. * ( ( voltage - ZERO_PERCENT_BATTERY_VOLTAGE) / batteryRange)); // returns percentage
  if (batteryPercent > 99) return 99;
  if (batteryPercent < 0) return 0;
  return batteryPercent;  
} 

void LEDlight(int color)
{
  digitalWrite(LEDred, LOW);
  digitalWrite(LEDgreen, LOW);
  digitalWrite(LEDblue, LOW);
  switch (color)
  {
    case RED:
      digitalWrite(LEDred, HIGH);
      break;
    case GREEN:
      digitalWrite(LEDgreen, HIGH);
      break;
    case BLUE:
      digitalWrite(LEDblue, HIGH);
      break;
    default:
      break; 
  }
}

bool stopIfFault()
{
  bool result = false;
  if (motorDriver.getM1Fault())
  {
      motorDriver.setSpeeds(0,0);
      //Serial.println("Fault detected in Motor 1 ");
      LEDlight(RED);
      result = true;
  }
  if (motorDriver.getM2Fault())
  {
      motorDriver.setSpeeds(0,0);
      //Serial.println("Fault detected in Motor 2 ");
      LEDlight(RED);
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
  motorDriver.setBrakes(255,255);
  brakesOn = true;
}

void Stop()
{
  motorDriver.setSpeeds(0,0);
  brakes();
  brakesOn = true;
}

void move(int speed) // speed goes from -255 to 255
{
  //Serial.println("moving, speed = ");
  //Serial.println(speed);
  //motorDriver.setM1Speed(speed);
  //motorDriver.setM2Speed(-speed);
  motorDriver.setSpeeds(speed,-speed);
  if (brakesOn)
  {
    coast();
    brakesOn = false;
  }
  if (speed == 0 || stopIfFault()) Moving = false;
  else Moving = true;
  timeOutCheck = millis();
}

void turn(int speed) // speed goes from -255 to 255
{
  //Serial.println("turning, speed = ");
  //Serial.println(speed);
  motorDriver.setSpeeds(speed,speed);
  if (brakesOn)
  {
    coast();
    brakesOn = false;
  }
  if (speed == 0 || stopIfFault()) Moving = false;
  else Moving = true;
  timeOutCheck = millis();  
}

void getMotorCurrents()
{
  currentLeftMotor += motorDriver.getM1CurrentMilliamps();
  currentRightMotor += motorDriver.getM2CurrentMilliamps();
}

void setMotorSpeeds(int speedLeft, speedRight)
{
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
    case 'W':    // move forward
    case 'w':
      move(speedToGo);
      break;
    case 'S':    // move backward
    case 's':
      move(-speedToGo);
      break;
    case 'D':    // turn right
    case 'd':
      turn(-speedToGo);
      break;
    case 'A':    // turn left
    case 'a':
      turn(speedToGo);
      break;
    case 'X':    // stop
    case 'x':
      Stop();
      break;
      
    case 'J':    // center servos
    case 'j':
      panServo.write(PAN_CENTER);
      tiltServo.write(TILT_CENTER);
      panPos = PAN_CENTER;
      tiltPos = TILT_CENTER;
      break;
    case 'N':    // tilt up
    case 'n':
      if (tiltPos - (TILT_DELTA * stepsToGo) <= TILT_MIN) tiltPos += TILT_DELTA * stepsToGo;
      else tiltPos = TILT_MIN;
      tiltServo.write(tiltPos);
      break;
    case 'U':    // tilt down
    case 'u':
      if (tiltPos + (TILT_DELTA * stepsToGo) >= TILT_MAX) tiltPos -= TILT_DELTA * stepsToGo;
      else tiltPos = TILT_MAX;
      tiltServo.write(tiltPos);
      break;
    case 'H':    // pan left
    case 'h':
      if (panPos - (PAN_DELTA * stepsToGo) >= PAN_MIN) panPos -= PAN_DELTA * stepsToGo;
      else panPos = PAN_MIN;
      panServo.write(panPos);
      break;
    case 'K':    // pan right
    case 'k':
      if (panPos + (PAN_DELTA * stepsToGo) <= PAN_MAX) panPos += PAN_DELTA * stepsToGo;
      else panPos = PAN_MAX;
      panServo.write(panPos);
      break;
    case 'M':    // tilt max down
    case 'm':
      tiltPos = TILT_MIN;
      tiltServo.write(tiltPos);
      break;
    case 'Y':    // tilt look down
    case 'y':
      tiltPos = TILT_LOOK_DOWN;
      tiltServo.write(tiltPos);
      break;
    case 'G':    // pan center
    case 'g':
      panPos = PAN_CENTER;
      panServo.write(panPos);
      break;
      
    /*case 'P':
    case 'p':
      digitalWrite(powerPin, LOW);
      delay(500);
      digitalWrite(powerPin,HIGH);
      delay(500);
      digitalWrite(powerPin,LOW);
      break;
     */
     
    case 'B':
    case 'b':
      //Serial.print(checkBattery(), BYTE); // version 1.0 does not use byte
      Serial.write((char)checkBattery());
      //Serial.write('a');
      break;
      
      default:
      //Serial.println("did not recognize command ");
      break;
  }
  //Serial.println(delayTime);
} 
/*
void timer_setup (int timer_number, byte mode, int prescale, byte outmode_A, byte outmode_B, byte capture_mode)
{
  // enforce field widths for sanity
  mode &= 15 ;
  outmode_A &= 3 ;
  outmode_B &= 3 ;
  capture_mode &= 3 ;

  byte clock_mode = 0 ; // 0 means no clocking - the counter is frozen.
  switch (prescale)
  {
    case 1: clock_mode = 1 ; break ;
    case 8: clock_mode = 2 ; break ;
    case 64: clock_mode = 3 ; break ;
    case 256: clock_mode = 4 ; break ;
    case 1024: clock_mode = 5 ; break ;
    default:
      if (prescale < 0)
        clock_mode = 7 ; // external clock
  }
  switch (timer_number)
  {    
    case 3:
      TCCR3A = (outmode_A << 6) | (outmode_B << 4) | (mode & 3) ;
      TCCR3B = (capture_mode << 6) | ((mode & 0xC) << 1) | clock_mode ;
      break;     
    case 4:
      TCCR4A = (outmode_A << 6) | (outmode_B << 4) | (mode & 3) ;
      TCCR4B = (capture_mode << 6) | ((mode & 0xC) << 1) | clock_mode ;
      break;
    case 5:
      TCCR5A = (outmode_A << 6) | (outmode_B << 4) | (mode & 3) ;
      TCCR5B = (capture_mode << 6) | ((mode & 0xC) << 1) | clock_mode ;
      break;
    default:
      break;
  } 
}

*/
void setup()
{ 
    
  Serial.begin(SerialSpeed);   // connect to laptop
  //Serial.println("serial connected");
  
  motorDriver.init();
  coast();
  currentLeftMotor = 0;
  currentRightMotor = 0;

  // using CTC (clear timer on compare) mode allows us to fire an interrupt when the timer overruns, so
  // we can get an accurate value even through timer overflows
  // The counter value (TCNTn) increases until a compare match occurs between TCNTn and OCRnA, and then counter
  // (TCNTn) is cleared, where "n" is the timer in use.
  // If we are never going to overflow, then just use normal mode (0)
 
 // timer_setup(3,4,-1,0,0,3);
  // timer 3, mode 4 (CTC), external clock, output mode A normal, output mode B normal,
  // capture mode = 3, so bits  6 and 7 of TCCRnB are = 1, meaning:
  // bit 7 = 1, noise filtering is on, so 4 bits of the same value are required for a transition
  // bit 6 = 1, rising edge is used for counting.

//  timer_setup(4,4,-1,0,0,3);
  // need to setup the registers for when the clock resets and also record when that 
  // happens
  
  batteryRange = FULL_BATTERY_VOLTAGE - ZERO_PERCENT_BATTERY_VOLTAGE;
  batteryMonitorPin = BATTERY_MONITOR_PIN;
  pinMode(BATTERY_MONITOR_GROUND_PIN, OUTPUT);
  digitalWrite(BATTERY_MONITOR_GROUND_PIN, LOW);
  
  tiltServo.attach(tiltPin); 
  panServo.attach(panPin);  // attaches the pan servo pin to the servo object
  panServo.write(PAN_CENTER);
  tiltServo.write(TILT_CENTER);
  panPos = PAN_CENTER;
  tiltPos = TILT_CENTER;
  
  // encoders
  // turning on the pullups saves having to hook up resistors
  // we can count pulses with the timers, don't need quadrature encoding,
  // since we already know which way the motors are turning.
  // The Input Capture Register ICR1 is a 16 bit register used to record the value of TCNT1
  // when an external event happens - typically a change on the ICP1 pin (Arduino pin 8). Only 16 bit timers have input capture.
  // we will use timers 2 and 3 for the encoders.
  /*
  encoderLeft = 0;
  encoderRight = 0;
       
  // encoders, could just tie the reds to +5, the blacks to ground
  pinMode(encoderLeftRed, OUTPUT);
  digitalWrite(encoderLeftRed,LOW); // this will have to drive high to turn on encoding
  pinMode(encoderLeftGreen, INPUT);
  digitalWrite(encoderLeftGreen, HIGH);       // turn on pullup resistor
  pinMode(encoderLeftYellow, INPUT);
  digitalWrite(encoderLeftYellow, HIGH);       // turn on pullup resistor
  pinMode(encoderLeftBlack, OUTPUT);
  digitalWrite(encoderLeftBlack,LOW);
  
  pinMode(encoderRightRed, OUTPUT); // this will have to drive high to turn on encoding
  digitalWrite(encoderRightRed,LOW);
  pinMode(encoderRightGreen, INPUT);
  digitalWrite(encoderRightGreen, HIGH);       // turn on pullup resistor
  pinMode(encoderRightYellow, INPUT);
  digitalWrite(encoderRightYellow, HIGH);       // turn on pullup resistor
  pinMode(encoderRightBlack, OUTPUT); 
  digitalWrite(encoderRightBlack,LOW);
  */
  
  // multicolor LED
  pinMode(LEDground, OUTPUT); 
  digitalWrite(LEDground, LOW);
  pinMode(LEDgreen, OUTPUT);
  digitalWrite(LEDgreen, LOW); 
  pinMode(LEDblue, OUTPUT);
  digitalWrite(LEDblue, LOW);
  pinMode(LEDred, OUTPUT);
  digitalWrite(LEDred, LOW);

  
  digitalWrite(LEDgreen, HIGH);
  delay(100);
  digitalWrite(LEDgreen, LOW);
  digitalWrite(LEDblue, HIGH);
  delay(100);
  digitalWrite(LEDblue, LOW);
  digitalWrite(LEDred, HIGH);
  delay(100);
  digitalWrite(LEDred, LOW);
}
 
void loop()
{ 
  // get a command from the serial port
  int inputLength = 0; 
  digitalWrite(LEDgreen,LOW); // show on LED that we are waiting for serial input
  do {
    while (!Serial.available()) // wait for input
    {
      if (millis() - timeOutCheck > TIMED_OUT && Moving) move(0);  //if we are moving and haven't heard anything in a long time, stop moving
      if (Moving)
      {
        getMotorCurrents();
        if (currentLeftMotor > currentRightMotor)
        {
          
    }
    inputBuffer[inputLength] = Serial.read(); // read it in
  } while (inputBuffer[inputLength] != LineEndCharacter && ++inputLength < BufferLength);
  inputBuffer[inputLength] = 0; //  add null terminator
      digitalWrite(LEDgreen,HIGH);  // show on LED that we received a serial input
  //Serial.println(inputBuffer);
  HandleCommand(inputBuffer, inputLength);
}

