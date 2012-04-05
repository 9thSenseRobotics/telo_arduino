// this takes inputs from the serial monitor and 
// outputs to pan and tilt servos
//
// input starts with P or T, case insensitive, to specify pan or tilt
// followed by the degrees desired (between values of MIN and MAX defined below)
// followed by the end character #
//
// so to get the pan servo to move to 90 degrees, enter P90#
// into the serial monitor


#include <Servo.h> 

#define tiltPin 5
#define panPin 6
#define LEDpin 8  // indicator that a serial signal was received, off = waiting, on = working
#define SerialSpeed 9600
#define BufferLength 16
#define LineEndCharacter '#' // serial input commands must end with this character

#define TILT_CENTER 70
#define TILT_MIN 35
#define TILT_MAX 180
#define TILT_DELTA 10

#define PAN_CENTER 85
#define PAN_MIN 0
#define PAN_MAX 180
#define PAN_DELTA 10


Servo tiltServo, panServo;  // create servo object to control a servo 
char inputBuffer[BufferLength];
int panCommand = true ;


// process a command string
void HandleCommand(char* input) //, int length)
{
  int moveTo, panCommand;
  if (input[0] == 'p' || input[0] == 'P') panCommand =  true;
  else if (input[0] == 't' || input[0] == 'T') panCommand = false;
  else
  {
     Serial.print("Invalid command, no P or T");
     return;
  }
  moveTo = atoi(&input[1]);
  if (panCommand)
  {
    if (moveTo < PAN_MIN  || moveTo > PAN_MAX)
    {
       Serial.print("Command is out of pan range, moveTo = ");
       Serial.println(moveTo);
       return;
    }
    
    panServo.write(moveTo);
    Serial.print("Pan moved to ");
    Serial.println(moveTo);
    return;
  }
  
  if (moveTo < TILT_MIN  || moveTo > TILT_MAX)
  {
    Serial.print("Command is out of tilt range, moveTo = ");
    Serial.println(moveTo);
    return;
  }  
  tiltServo.write(moveTo);
  Serial.print("Tilt moved to ");
  Serial.println(moveTo);
}
    
  
void setup() 
{ 
  tiltServo.attach(tiltPin);  // attaches the servo to the servo object
  panServo.attach(panPin); 
  pinMode(LEDpin, OUTPUT);
  Serial.begin(9600);
  
  panServo.write(PAN_CENTER);
  tiltServo.write(TILT_CENTER);
} 
 
 
void loop() 
{ 
  // get a command from the serial port
  int inputLength = 0; 
  digitalWrite(LEDpin,LOW); // show on LED that we are waiting for a serial input
  do {
    while (!Serial.available()); // wait for input
    inputBuffer[inputLength] = Serial.read(); // read it in
  } while (inputBuffer[inputLength] != LineEndCharacter && ++inputLength < BufferLength);
  inputBuffer[inputLength] = 0; //  add null terminator
  digitalWrite(LEDpin,HIGH);  // show on LED that we received a serial input
  Serial.print("Command received = ");
  Serial.println(inputBuffer);
  HandleCommand(inputBuffer); //, inputLength);
  delay(500);
} 
