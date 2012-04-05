// accepts serial port inputs and responds with moves
// command form is two letters for direction:
// W, S, A, D, = move forward, move backward, right turn, left turn.
// case does not matter, so w, a, s, d are OK
// followed by the time to move from 1 to 9 which will be multiplied by the MIN_TIME for a move
// followed by the speed to move, 0 to 255
// followed by a character to indicate that the input is complete, in our case that character is #
// for example, move forward for 2 * MIN_TIME with a speed of 200 is
// W2200#
// to turn right for 5 * MIN_TIME at full speed is
// D5255#
// if you wish to use the default time and speed (a defined value), then just the letters is sufficient:
// D#
// if you wish to use the default time, just enter the speed:
// D255#
// it moves for a time = delayTime and then stops.  delayTime is a define.

 
#define PwmPinMotorA 3  //PWM control for motor outputs 1 and 2 is on digital pin 3
#define PwmPinMotorB 11  //PWM control for motor outputs 3 and 4 is on digital pin 11
#define DirectionPinMotorA 12 //direction control for motor outputs 1 and 2 is on digital pin 12
#define DirectionPinMotorB 13  //direction control for motor outputs 3 and 4 is on digital pin 13
#define SerialSpeed 9600
#define BufferLength 16
#define LineEndCharacter '#' // serial input commands must end with this character

#define MIN_DELAY_TIME 200
#define DEFAULT_DELAY_TIME 200
#define DEFAULT_SPEED 255

char inputBuffer[BufferLength];

void go(int speed)
{
  if (speed < 0) speed = 0;
  else if (speed > 255) speed = 255;
  
  analogWrite(PwmPinMotorA, speed);  //set both motors to run at 100% duty cycle (fast)
  analogWrite(PwmPinMotorB, speed);
}

void move(int speed) // speed goes from 0 to 255
{
  //Serial.println("moving, speed = ");
  //Serial.println(speed);
  if (speed > 0)  // go forward
  {
    digitalWrite(DirectionPinMotorA, LOW);  //Set motor direction, 1 low, 2 high
    digitalWrite(DirectionPinMotorB, HIGH);  //Set motor direction, 3 high, 4 low
  }
  else  // go backward
  {
    speed = -speed;    
    digitalWrite(DirectionPinMotorA, HIGH);
    digitalWrite(DirectionPinMotorB, LOW); 
  }
  go(speed);  
}

void turn(int speed) // speed goes from 0 to 255
{
  //Serial.println("turning ");
  if (speed > 0)  // turn right
  {
    digitalWrite(DirectionPinMotorA, LOW);  //Set motor direction, 1 low, 2 high
    digitalWrite(DirectionPinMotorB, LOW);  //Set motor direction, 3 high, 4 low
  }
  else  // turn left
  {
    speed = -speed;
    digitalWrite(DirectionPinMotorA, HIGH);
    digitalWrite(DirectionPinMotorB, HIGH); 
  }
  go(speed);  
}
 
// process a command string
void HandleCommand(char* input, int length)
{
  int speedToGo = DEFAULT_SPEED;
  int delayTime = DEFAULT_DELAY_TIME;
  int value = 0;
  // calculate number following command
  if (length > 1) {
    value = atoi(&input[1]);
    if (value > 1000)
    {
      delayTime = value - (value % 1000);  // time and speed specified
      speedToGo = value - delayTime;
    }
    else speedToGo = value;                  // speed specified, default time
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
    default:
      //Serial.println("did not recognize command ");
      break;
  }
  //Serial.println(delayTime);
  delay(MIN_DELAY_TIME * (delayTime/1000)); // delay time goes (1 to 9) * MIN_DELAY_TIME
  go(0);
} 
 
void setup()
{
  // motor pins must be outputs
  pinMode(PwmPinMotorA, OUTPUT);
  pinMode(PwmPinMotorB, OUTPUT);
  pinMode(DirectionPinMotorA, OUTPUT);
  pinMode(DirectionPinMotorB, OUTPUT);
  
  go(0); //stop both motors
 
  Serial.begin(SerialSpeed);   
  //Serial.println("serial connected");
}
 
void loop()
{ 
  // get a command from the serial port
  int inputLength = 0;  
  do {
    while (!Serial.available()); // wait for input
    inputBuffer[inputLength] = Serial.read(); // read it in
  } while (inputBuffer[inputLength] != LineEndCharacter && ++inputLength < BufferLength);
  inputBuffer[inputLength] = 0; //  add null terminator
  //Serial.println(inputBuffer);
  HandleCommand(inputBuffer, inputLength);
}

