// accepts serial port inputs and responds with moves

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


#include <Servo.h> 


#define PwmPinMotorRight 3  //PWM control for motor outputs 1 and 2 is on digital pin 3 with ardumoto shield
#define PwmPinMotorLeft 11  //PWM control for motor outputs 3 and 4 is on digital pin 11 with ardumoto shield
#define DirectionPinMotorRight 12 //direction control for motor outputs 1 and 2 is on digital pin 12 with ardumoto shield
#define DirectionPinMotorLeft 13  //direction control for motor outputs 3 and 4 is on digital pin 13 with ardumoto shield
#define tiltPin 5
#define panPin 6
#define powerPin 7
#define LEDpin 8  // indicator that a serial signal was received, off = waiting, on = working
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

Servo panServo, tiltServo;  // create servo objects to control the servos
int panPos, tiltPos;    // variable to store the servo position 
char inputBuffer[BufferLength];
bool Moving;
long timeOutCheck;

void go(int speed)
{
  if (speed < 0) speed = 0;
  else if (speed > 255) speed = 255;
  
  analogWrite(PwmPinMotorRight, speed);
  analogWrite(PwmPinMotorLeft, speed);
  if (speed == 0) Moving = false;
  else Moving = true;
  timeOutCheck = millis();
}

void move(int speed) // speed goes from 0 to 255
{
  //Serial.println("moving, speed = ");
  //Serial.println(speed);
  if (speed > 0)  // go forward
  {
    digitalWrite(DirectionPinMotorRight, LOW);  //Set motor direction, 1 low, 2 high
    digitalWrite(DirectionPinMotorLeft, HIGH);  //Set motor direction, 3 high, 4 low
  }
  else  // go backward
  {
    speed = -speed;    
    digitalWrite(DirectionPinMotorRight, HIGH);
    digitalWrite(DirectionPinMotorLeft, LOW); 
  }
  go(speed);  
}

void turn(int speed) // speed goes from 0 to 255
{
  //Serial.println("turning ");
  if (speed > 0)  // turn right
  {
    digitalWrite(DirectionPinMotorRight, LOW);  //Set motor direction, 1 low, 2 high
    digitalWrite(DirectionPinMotorLeft, LOW);  //Set motor direction, 3 high, 4 low
  }
  else  // turn left
  {
    speed = -speed;
    digitalWrite(DirectionPinMotorRight, HIGH);
    digitalWrite(DirectionPinMotorLeft, HIGH); 
  }
  go(speed);  
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
      move(-speedToGo);
      break;
    case 'S':    // move backward
    case 's':
      move(speedToGo);
      break;
    case 'D':    // turn right
    case 'd':
      turn(speedToGo);
      break;
    case 'A':    // turn left
    case 'a':
      turn(-speedToGo);
      break;
    case 'X':    // stop
    case 'x':
      go(0);
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
    case 'P':
    case 'p':
      digitalWrite(powerPin, LOW);
      delay(500);
      digitalWrite(powerPin,HIGH);
      delay(500);
      digitalWrite(powerPin,LOW);
      break;
      
    default:
      //Serial.println("did not recognize command ");
      break;
  }
  //Serial.println(delayTime);
} 


void setup()
{
  // motor pins must be outputs
  pinMode(PwmPinMotorRight, OUTPUT);
  pinMode(PwmPinMotorLeft, OUTPUT);
  pinMode(DirectionPinMotorRight, OUTPUT);
  pinMode(DirectionPinMotorLeft, OUTPUT);
  pinMode(LEDpin, OUTPUT); // LED indicator
  pinMode(powerPin, OUTPUT); // when set from low to high, toggles base power
  
  go(0); //stop both motors
 
  tiltServo.attach(tiltPin); 
  panServo.attach(panPin);  // attaches the pan servo pin to the servo object
  panServo.write(PAN_CENTER);
  tiltServo.write(TILT_CENTER);
  panPos = PAN_CENTER;
  tiltPos = TILT_CENTER;
       
  digitalWrite(powerPin,LOW);  
  digitalWrite(LEDpin,LOW); 
  
  Serial.begin(SerialSpeed);   // connect to laptop
  //Serial.println("serial connected");
}
 
void loop()
{ 
  // get a command from the serial port
  int inputLength = 0; 
  digitalWrite(LEDpin,LOW); // show on LED that we are waiting for a serial input
  do {
    while (!Serial.available()) // wait for input
    {
      if (millis() - timeOutCheck > TIMED_OUT && Moving) go(0);  //if we are moving and haven't heard anything in a long time, stop moving
    }
    inputBuffer[inputLength] = Serial.read(); // read it in
  } while (inputBuffer[inputLength] != LineEndCharacter && ++inputLength < BufferLength);
  inputBuffer[inputLength] = 0; //  add null terminator
      digitalWrite(LEDpin,HIGH);  // show on LED that we received a serial input
  //Serial.println(inputBuffer);
  HandleCommand(inputBuffer, inputLength);
}

