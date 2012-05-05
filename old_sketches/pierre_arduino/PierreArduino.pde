

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
/*
 Use the motorGo(uint8_t motor, uint8_t direct, uint8_t pwm)
 function to get motors going in either CW, CCW, BRAKEVCC, or
 BRAKEGND. Use motorOff(int motor) to turn a specific motor off.

 The motor variable in each function should be either a 0 or a 1.
 pwm in the motorGo function should be a value between 0 and 255.
 */
#define BRAKEVCC 0
#define CW   1
#define CCW  2
#define BRAKEGND 3
#define CS_THRESHOLD 100
#define RIGHT_MOTOR 1
#define LEFT_MOTOR 0

/*  VNH2SP30 pin definitions
 xxx[0] controls '1' outputs
 xxx[1] controls '2' outputs */
int inApin[2] = {7, 4};  // INA: Clockwise input
int inBpin[2] = {8, 9}; // INB: Counter-clockwise input
int pwmpin[2] = {5, 6}; // PWM input
int cspin[2] = {2, 3}; // CS: Current sense ANALOG input
int enpin[2] = {0, 1}; // EN: Status of switches output (Analog pin)

int statpin = 12;


//#define PwmPinMotorRight 3  //PWM control for motor outputs 1 and 2 is on digital pin 3 with ardumoto shield
//#define PwmPinMotorLeft 11  //PWM control for motor outputs 3 and 4 is on digital pin 11 with ardumoto shield
//#define DirectionPinMotorRight 12 //direction control for motor outputs 1 and 2 is on digital pin 12 with ardumoto shield
//#define DirectionPinMotorLeft 13  //direction control for motor outputs 3 and 4 is on digital pin 13 with ardumoto shield
#define tiltPin 10
#define panPin 13
//#define LEDPin 13
#define SerialSpeed 9600
#define BufferLength 16
#define LineEndCharacter '#' // serial input commands must end with this character

#define TIMED_OUT 8000
#define DEFAULT_SPEED 255

#define TILT_CENTER 50
#define TILT_MIN 60
#define TILT_MAX 180
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

/* motorGo() will set a motor going in a specific direction
 the motor will continue going in that direction, at that speed
 until told to do otherwise.

 motor: this should be either 0 or 1, will selet which of the two
 motors to be controlled

 direct: Should be between 0 and 3, with the following result
 0: Brake to VCC
 1: Clockwise
 2: CounterClockwise
 3: Brake to GND

 pwm: should be a value between ? and 1023, higher the number, the faster
 it'll go
 */
void motorGo(uint8_t motor, uint8_t direct, uint8_t pwm)
{
 if (motor <= 1)
 {
   if (direct <=4)
   {
     // Set inA[motor]
     if (direct <=1)
       digitalWrite(inApin[motor], HIGH);
     else
       digitalWrite(inApin[motor], LOW);

     // Set inB[motor]
     if ((direct==0)||(direct==2))
       digitalWrite(inBpin[motor], HIGH);
     else
       digitalWrite(inBpin[motor], LOW);

     analogWrite(pwmpin[motor], pwm);
   }
 }
}

void motorOff(int motor)
{
 // Initialize braked
 for (int i=0; i<2; i++)
 {
   digitalWrite(inApin[i], LOW);
   digitalWrite(inBpin[i], LOW);
 }
 analogWrite(pwmpin[motor], 0);
}

void move(int speed_old) // speed_old goes from 0 to 255
{
  //Serial.println("moving, speed = ");
  //Serial.println(speed); 
  if (speed_old == 0) // stop
  {
    motorOff(RIGHT_MOTOR);
    motorOff(LEFT_MOTOR);
    Moving = false;
    timeOutCheck = millis();
    return;
  }  
  int speed = (speed_old * 4) + 3; // for this board, speed needs to go 0 to 1023, this is a quick fix
  Serial.println(speed);
  if (speed > 0)  // go forward
  {
    motorGo(RIGHT_MOTOR, CW, speed);
    motorGo(LEFT_MOTOR, CCW, speed);  
  }
  else  // go backward
  {
    speed = -speed;    
    motorGo(RIGHT_MOTOR, CCW, speed);
    motorGo(LEFT_MOTOR, CW, speed);   
  }
  Moving = true;
  timeOutCheck = millis();
}

void turn(int speed_old) // speed_old goes from 0 to 255
{
  //Serial.println("turning ");
    if (speed_old == 0) // stop
  {
    motorOff(RIGHT_MOTOR);
    motorOff(LEFT_MOTOR);
    Moving = false;
    timeOutCheck = millis();
    return;
  }  
  int speed = (speed_old * 4) + 3; // for this board, speed needs to go 0 to 1023, this is a quick fix
  if (speed > 0)  // turn right
  {
    motorGo(RIGHT_MOTOR, CCW, speed);
    motorGo(LEFT_MOTOR, CCW, speed);   
  }
  else  // turn left
  {
    speed = -speed;
    motorGo(RIGHT_MOTOR, CW, speed);
    motorGo(LEFT_MOTOR, CW, speed); 
  }
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
      turn(speedToGo);
      break;
    case 'A':    // turn left
    case 'a':
      turn(-speedToGo);
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
    case 'U':    // tilt up
    case 'u':
      if (tiltPos - (TILT_DELTA * stepsToGo) >= TILT_MIN) tiltPos -= TILT_DELTA * stepsToGo;
      else tiltPos = TILT_MIN;
      tiltServo.write(tiltPos);
      break;
    case 'N':    // tilt down
    case 'n':

      if (tiltPos + (TILT_DELTA * stepsToGo) <= TILT_MAX) tiltPos += TILT_DELTA * stepsToGo;
      else tiltPos = TILT_MAX;
      tiltServo.write(
      tiltPos);
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
   /* case 'P':
    case 'p':
      digitalWrite(powerPin, LOW);
      delay(500);
      digitalWrite(powerPin,HIGH);
      delay(500);
      digitalWrite(powerPin,LOW);
      break;
   */
    default:
    
      Serial.println("did not recognize command ");
      break;
  }
  //Serial.println(delayTime);
} 


void setup()
{
  //pinMode(powerPin, OUTPUT); // when set from low to high, toggles base power  
  pinMode(statpin, OUTPUT);
 // pinMode(LEDPin, OUTPUT);

 // Initialize digital pins as outputs
 for (int i=0; i<2; i++)
 {
   pinMode(inApin[i], OUTPUT);
   pinMode(inBpin[i], OUTPUT);
   pinMode(pwmpin[i], OUTPUT);
 }
 // Initialize braked
 for (int i=0; i<2; i++)
 {
   digitalWrite(inApin[i], LOW);
   digitalWrite(inBpin[i], LOW);
 }
 
  tiltServo.attach(tiltPin); 
  panServo.attach(panPin);  // attaches the pan servo pin to the servo object
  panServo.write(PAN_CENTER);
  tiltServo.write(TILT_CENTER);
  panPos = PAN_CENTER;
  tiltPos = TILT_CENTER;
       
  //digitalWrite(powerPin,LOW);  
  
  Serial.begin(SerialSpeed);   // connect to laptop
  //Serial.println("serial connected");
}
 
void loop()
{ 
  // get a command from the serial port
  int inputLength = 0; 
  //digitalWrite(LEDPin,LOW); // show on LED that we are waiting for a serial input
  do {
    while (!Serial.available()) // wait for input
    {
      if (millis() - timeOutCheck > TIMED_OUT && Moving) move(0);  //if we are moving and haven't heard anything in a long time, stop moving
      if ((analogRead(cspin[0]) < CS_THRESHOLD) && (analogRead(cspin[1]) < CS_THRESHOLD)) digitalWrite(statpin, HIGH); // check current thresholds
    }
    inputBuffer[inputLength] = Serial.read(); // read it in
  } while (inputBuffer[inputLength] != LineEndCharacter && ++inputLength < BufferLength);
  inputBuffer[inputLength] = 0; //  add null terminator
      //digitalWrite(LEDPin,HIGH);  // show on LED that we received a serial input
  //Serial.println(inputBuffer);
  HandleCommand(inputBuffer, inputLength);
}

