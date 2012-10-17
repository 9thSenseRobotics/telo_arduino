// EEPROM write to setup default values and version number in
// Robot arduinos EEPROM, so that they can be changed remotely
// version 0.74 matches threeMotorsRobotCommPCBsimple_v_074
// changes are that we added:
//  EEPROM.write(210, RIGHT_MOTOR_BIAS);
//  EEPROM.write(211, RIGHT_MOTOR_BW_BIAS);
//  EEPROM.write(212, RIGHT_MOTOR_STOP_DELAY);

#include <EEPROM.h>

#define SERIAL_PORT Serial
#define SERIAL_SPEED 115200
#define INPUT_BUFFER_SIZE 256
#define COMMAND_END_CHARACTER '#'
#define COMM_CHECK_CHARACTER 'c'

// set customer number and the software/hardware versions we are going to use with this bot
#define customer_number 0
#define software_version 72
#define hardware_version 1
#define BT "00:06:66:46:5A:60"

// these are used to determine if this program was run, indicating bot should use EEPROM data
#define EEPROM_TEST_VALUE_10 2
#define EEPROM_TEST_VALUE_11 4
#define EEPROM_TEST_VALUE_12 8

#define TIMED_OUT 30  // this gets multiplied by 100 in actual use.  have to keep parameters < 256
#define DEFAULT_SPEED 220
#define BW_REDUCTION 50
#define DEFAULT_TILT_UP_SPEED 180
#define DEFAULT_TILT_DOWN_SPEED 135
#define DEFAULT_DEGREES 10
#define TICKS_PER_DEGREE_OF_TILT 30
#define DEFAULT_TURN_FOREVER_SPEED 220
#define TURN_TIME 500
#define MOVE_TIME 1000
#define TILT_TIME 500
#define NUDGE_TURN_TIME 200
#define NUDGE_MOVE_TIME 300
#define NUDGE_TILT_TIME 200
#define MIN_ACCEL_SPEED 120
#define MIN_DECEL_SPEED 60
#define DELTA_SPEED 60
#define ACCEL_DELAY 200
#define LEFT_MOTOR_BIAS 10
#define LEFT_MOTOR_BW_BIAS 23
#define RIGHT_MOTOR_BIAS 0
#define RIGHT_MOTOR_BW_BIAS 0
#define LEFT_MOTOR_STOP_DELAY 0
#define RIGHT_MOTOR_STOP_DELAY 0
// next two are modified so that they stay under 255
#define CURRENT_LIMIT_TOP_MOTOR 20  // this gets multiplied by 100 in actual use
#define CURRENT_LIMIT_DRIVE_MOTORS 40 // this gets multiplied by 100 in actual use
#define CURRENT_LIMIT_ENABLED 1
// one turn of the motor = 960 steps in the encoder.
// with large vex wheel, one rotation = 42 cm
// so we have 23 steps per cm
#define ENCODER_TICKS_PER_CM 23

//#define ZERO_PERCENT_BATTERY_VOLTAGE 10.5
//#define FULL_BATTERY_VOLTAGE 13.0
//#define VOLTAGE_DIVIDER_RATIO 3.2

// represent the above values with bytes:
#define INTEGER_ZERO_PERCENT_BATTERY_VOLTAGE 10
#define DECIMAL_ZERO_PERCENT_BATTERY_VOLTAGE 5
#define INTEGER_FULL_BATTERY_VOLTAGE 13
#define DECIMAL_FULL_BATTERY_VOLTAGE 0
#define INTEGER_VOLTAGE_DIVIDER_RATIO 3
#define DECIMAL_VOLTAGE_DIVIDER_RATIO 2

boolean enableEEPROMwrite = false;

void setDefaults()
{
  // set customer number and the software/hardware versions we are going to use with this bot
  EEPROM.write(0, customer_number);
  EEPROM.write(1, software_version);
  EEPROM.write(2, hardware_version);
  
  // these are used to determine if this program was run, indicating bot should use EEPROM data
  EEPROM.write(10, EEPROM_TEST_VALUE_10);
  EEPROM.write(11, EEPROM_TEST_VALUE_11);
  EEPROM.write(12, EEPROM_TEST_VALUE_12);
  
  EEPROM.write(101, TIMED_OUT);  // have to keep it down under 256
  EEPROM.write(102, DEFAULT_SPEED);
  EEPROM.write(103, BW_REDUCTION);
  EEPROM.write(104, DEFAULT_TILT_UP_SPEED);
  EEPROM.write(105, DEFAULT_TILT_DOWN_SPEED);
  EEPROM.write(106, DEFAULT_DEGREES);
  EEPROM.write(107, TICKS_PER_DEGREE_OF_TILT);
  EEPROM.write(108, DEFAULT_TURN_FOREVER_SPEED);
  EEPROM.write(109, TURN_TIME);
  EEPROM.write(110, MOVE_TIME);
  EEPROM.write(111, TILT_TIME);
//  EEPROM.write(201, NUDGE_TURN_TIME);  // put in below to keep list in address number order
//  EEPROM.write(202, NUDGE_MOVE_TIME);
//  EEPROM.write(203, NUDGE_TILT_TIME);
  EEPROM.write(112, MIN_ACCEL_SPEED);
  EEPROM.write(113, MIN_DECEL_SPEED);
  EEPROM.write(114, DELTA_SPEED);
  EEPROM.write(115, ACCEL_DELAY);
  EEPROM.write(116, LEFT_MOTOR_BIAS);
  EEPROM.write(117, LEFT_MOTOR_BW_BIAS);
  //EEPROM.write(210, RIGHT_MOTOR_BIAS);
  //EEPROM.write(211, RIGHT_MOTOR_BW_BIAS);
  //EEPROM.write(212, RIGHT_MOTOR_STOP_DELAY);
  EEPROM.write(118, LEFT_MOTOR_STOP_DELAY);
  EEPROM.write(119, CURRENT_LIMIT_TOP_MOTOR);     // multiply to keep the eeprom parameter < 255
  EEPROM.write(120, CURRENT_LIMIT_DRIVE_MOTORS); // multiply to keep the eeprom parameter < 255
  //EEPROM.write(204, CURRENT_LIMIT_ENABLED);
  EEPROM.write(121, ENCODER_TICKS_PER_CM);
  EEPROM.write(122, INTEGER_ZERO_PERCENT_BATTERY_VOLTAGE);
  EEPROM.write(123, DECIMAL_ZERO_PERCENT_BATTERY_VOLTAGE);
  EEPROM.write(124, INTEGER_FULL_BATTERY_VOLTAGE);
  EEPROM.write(125, DECIMAL_FULL_BATTERY_VOLTAGE);
  EEPROM.write(126, INTEGER_VOLTAGE_DIVIDER_RATIO);
  EEPROM.write(127, DECIMAL_VOLTAGE_DIVIDER_RATIO);
  
  EEPROM.write(201, NUDGE_TURN_TIME);
  EEPROM.write(202, NUDGE_MOVE_TIME);
  EEPROM.write(203, NUDGE_TILT_TIME);
  EEPROM.write(204, CURRENT_LIMIT_ENABLED);
  
  EEPROM.write(210, RIGHT_MOTOR_BIAS);
  EEPROM.write(211, RIGHT_MOTOR_BW_BIAS);
  EEPROM.write(212, RIGHT_MOTOR_STOP_DELAY);
  writeBTaddress();
  }
  
  void writeToEEPROM(int address, byte value)
  {
    if (address > 4095 || address < 0) return;
    EEPROM.write(address, value);
  }
  
  int readFromEEPROM(int address)
  {
    if (address > 4095 || address < 0) return -1;
    return EEPROM.read(address);
  }
  
  void writeBTaddress()
  {
    char buffer[18] = BT;
    for (int i= 0; i < 17; i++)
    {
      EEPROM.write(300 + i, buffer[i]);
    }
  }
  
  void readBTaddress()
  {
    char buffer[18];
    for (int i= 0; i < 17; i++)
    {
      buffer[i] = EEPROM.read(300 + i);
    }
    SERIAL_PORT.print("Bluetooth address is: ");
    for (int i=0; i < 17; i++) SERIAL_PORT.print(buffer[i]);
    SERIAL_PORT.println();
  }
      

// process a command string
void HandleCommand(char* input, int length)
{
  int speedToGo, degreesToGo, EEPROMvalue, EEPROMaddress;
  long value = 0;
  // calculate number following command
  if (length > 1)
  {
    value = atoi(&input[1]);
    if (value < 256)
    {
      EEPROMaddress = value;
    }
    else  // means we are writing to the EEPROM
    {
      EEPROMvalue =  value % 1000;  // the lower three digits are the value
      EEPROMaddress = value - EEPROMvalue;  // the upper three digits are the address
    }
  }

  // check commands
  // ************note that if you use more than one character here
  // the bytes are swapped, ie 'FM' means command MF *****************
  // you can use this stmt to get the command:
  // int* command = (int*)input;
  // but not needed when we just have a single character command format
  switch(input[0]) { 
    case 'E':
      EEPROMvalue = readFromEEPROM(EEPROMaddress);
      //SERIAL_PORT_BLUETOOTH.print("MESSAGE_EEPROM_VALUE");  // lead with "mE"  
                                      // 'm' indicates that this is a message for the server
                                      // 'E' indicates that it is an EEPROM value
      //SERIAL_PORT_BLUETOOTH.println(EEPROMvalue); 
        SERIAL_PORT.print("Value = ");
        SERIAL_PORT.print(EEPROMvalue);
        SERIAL_PORT.print(" was read from EEPROM address = ");
        SERIAL_PORT.print(EEPROMaddress);
        if (EEPROMaddress == 101 || EEPROMaddress == 120)
        {
           SERIAL_PORT.print(" but actual parameter used is ");
           SERIAL_PORT.println(EEPROMvalue * 100);
        } 
        if (EEPROMaddress == 119)
        {
           SERIAL_PORT.print(" but actual parameter used is ");
           SERIAL_PORT.println(EEPROMvalue * 10);
        }
      break;
    case 'Z':
      enableEEPROMwrite = true;
      SERIAL_PORT.print("EEPROM writing enabled.");
      break;
    case 'z':
      enableEEPROMwrite = false;
      SERIAL_PORT.print("EEPROM writing disabled.");
      break;
    case 'v':
      if (enableEEPROMwrite)
      {
        writeToEEPROM(EEPROMaddress, EEPROMvalue);
        SERIAL_PORT.print("Value = ");
        SERIAL_PORT.print(EEPROMvalue);
        SERIAL_PORT.print(" was written to EEPROM address = ");
        SERIAL_PORT.print(EEPROMaddress);
      }
      else SERIAL_PORT.print("EEPROM write requested when writing not enabled");
      break;
    case 'a':
    case 'A':
      readBTaddress();
      break;
        
    default:
      SERIAL_PORT.print("did not recognize command: ");
       if (input[0] == 13) SERIAL_PORT.print(" 13 ");
       else if (input[0] == 10) SERIAL_PORT.print(" 10 ");
       else if (input[0] == 32) SERIAL_PORT.print(" 32 ");
       else if (input[0] == 0) SERIAL_PORT.print(" 0 ");
       else SERIAL_PORT.print(input[0]);
      break;
  }
} 
  void setup()
  {
    SERIAL_PORT.begin(SERIAL_SPEED);
    SERIAL_PORT.println("ready for commands");
    setDefaults();
  }
  
  void loop()
{
  int inputLength = 0;
  char inputBuffer[INPUT_BUFFER_SIZE], charIn;
  while (charIn != COMMAND_END_CHARACTER && inputLength < INPUT_BUFFER_SIZE)
  {
    while (!SERIAL_PORT.available()) // wait for input
    {
       delay(10);
    }
    charIn = SERIAL_PORT.read(); // read it in
    if (charIn != 13 && charIn != 10 && charIn != 32)  // ignore carriage returns, line feeds, and spaces
    {
       inputBuffer[inputLength] = charIn;
       inputLength++;
       //SERIAL_PORT.print(charIn);
    }
  } 
  
  //now flush anything that came after the COMMAND_END_CHARACTER
  if (SERIAL_PORT.available())
  {
    SERIAL_PORT.print("Extra characters received are: ");  // this is usually the null char end string (0)
    while (SERIAL_PORT.available())
    {
       charIn = SERIAL_PORT.read(); // read it in
       if (charIn == 13) SERIAL_PORT.print(" 13 ");
       else if (charIn == 10) SERIAL_PORT.print(" 10 ");
       else if (charIn == 32) SERIAL_PORT.print(" 32 ");
       else if (charIn == 0) SERIAL_PORT.print(" 0 ");
       else SERIAL_PORT.print(charIn);
    }
    SERIAL_PORT.println();
  }
  
  // throw away the line end character
  inputLength -= 1;  // -1 because it is incremented after the last character
  inputBuffer[inputLength] = 0;  // change COMMAND_END_CHARACTER to a 0 (remember, index is one behind inputLength)
  
 SERIAL_PORT.print("commanded: ");
 SERIAL_PORT.println(inputBuffer);

  // if the command == COMM_CHECK_CHARACTER it is just a local bluetooth comm check
  if (inputBuffer[0] != COMM_CHECK_CHARACTER && inputLength > 0) HandleCommand(inputBuffer, inputLength);
   
  // echo the command
  //SERIAL_PORT.println(inputBuffer); // need to println because android uses the CR as a delimiter
  for (int i = 0; i < inputLength; i++) inputBuffer[i] = 0;
}

  
