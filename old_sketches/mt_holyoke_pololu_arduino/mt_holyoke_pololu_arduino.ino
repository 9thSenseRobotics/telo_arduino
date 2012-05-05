
// accepts serial port inputs and responds with moves and servos

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

// so the only remaining pins are 3,5,13,A2,A3,A4,A5

#define tiltPin 3
#define panPin 5
#define LEDpin 13  // indicator that a serial signal was received, off = waiting, on = working
#define SerialSpeed 9600
#define BufferLength 16
#define LineEndCharacter '#' // serial input commands must end with this character

#define TIMED_OUT 8000
#define DEFAULT_SPEED 255

#define TILT_CENTER 100
#define TILT_MIN 45
#define TILT_MAX 135
#define TILT_DELTA 10

#define PAN_CENTER 90
#define PAN_MIN 0
#define PAN_MAX 180
#define PAN_DELTA 10

DualVNH5019MotorShield motorDriver;
Servo panServo, tiltServo;  // create servo objects to control the servos
int panPos, tiltPos;    // variable to store the servo position 
char inputBuffer[BufferLength];
bool Moving;
long timeOutCheck;

bool stopIfFault()
{
  bool result = false;
/*  if (motorDriver.getM1Fault())
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
  }*/
  return result;
}

void coast()
{
  motorDriver.setBrakes(0,0);
}

void brakes()
{
  motorDriver.setBrakes(255,255);
}

void move(int speed) // speed goes from -255 to 255
{
  Serial.println("moving, speed = ");
  Serial.println(speed);
  analogWrite(9,127);
  //motorDriver.setM1Speed(speed);
  //motorDriver.setM2Speed(-speed);
  //if (speed == 0 || stopIfFault()) Moving = false;
  //else Moving = true;
  timeOutCheck = millis();
}

void turn(int speed) // speed goes from -255 to 255
{
  Serial.println("turning, speed = ");
  Serial.println(speed);
  motorDriver.setSpeeds(speed,speed);
  if (speed == 0 || stopIfFault()) Moving = false;
  else Moving = true;
  timeOutCheck = millis();  
}

void getMotorCurrents()
{
  int currentM1 = motorDriver.getM1CurrentMilliamps();
  int currentM2 = motorDriver.getM2CurrentMilliamps();
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
      move(0);
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
      if (tiltPos - (TILT_DELTA * stepsToGo) >= TILT_MIN) tiltPos -= TILT_DELTA * stepsToGo;
      else tiltPos = TILT_MIN;
      tiltServo.write(tiltPos);
      break;
    case 'U':    // tilt down
    case 'u':
      if (tiltPos + (TILT_DELTA * stepsToGo) <= TILT_MAX) tiltPos += TILT_DELTA * stepsToGo;
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
      
    /*case 'P':
    case 'p':
      digitalWrite(powerPin, LOW);
      delay(500);
      digitalWrite(powerPin,HIGH);
      delay(500);
      digitalWrite(powerPin,LOW);
      break;
     */
     
    default:
      //Serial.println("did not recognize command ");
      break;
  }
  //Serial.println(delayTime);
} 


void setup()
{ 
    
  Serial.begin(SerialSpeed);   // connect to laptop
  //Serial.println("serial connected");
  
  motorDriver.init();
  
  tiltServo.attach(tiltPin); 
  panServo.attach(panPin);  // attaches the pan servo pin to the servo object
  panServo.write(PAN_CENTER);
  tiltServo.write(TILT_CENTER);
  panPos = PAN_CENTER;
  tiltPos = TILT_CENTER;
       
  pinMode(LEDpin, OUTPUT); // LED indicator
  digitalWrite(LEDpin,LOW); 

      analogWrite(9,260);
  while(1);
}
 
void loop()
{ 
  // get a command from the serial port
  int inputLength = 0; 
  digitalWrite(LEDpin,LOW); // show on LED that we are waiting for a serial input
  do {
    while (!Serial.available()) // wait for input
    {
      if (millis() - timeOutCheck > TIMED_OUT && Moving) move(0);  //if we are moving and haven't heard anything in a long time, stop moving
    }
    inputBuffer[inputLength] = Serial.read(); // read it in
  } while (inputBuffer[inputLength] != LineEndCharacter && ++inputLength < BufferLength);
  inputBuffer[inputLength] = 0; //  add null terminator
      digitalWrite(LEDpin,HIGH);  // show on LED that we received a serial input
  //Serial.println(inputBuffer);
  HandleCommand(inputBuffer, inputLength);
}

