#define SERIAL_SPEED 115200
#define BLUETOOTH_SPEED 115200
#define INPUT_BUFFER_SIZE 256
#define COMMAND_END_CHARACTER '#'
#define COMM_CHECK_CHARACTER 'c'
#define COMMAND_ECHO_CHARACTER 'e'

char inputBuffer[INPUT_BUFFER_SIZE], charIn;
int inputLength;
bool exceededCurrentLimitC = false, enableEEPROMwrite = false;


int program_version;
int timed_out_default, speed_default;
int nudge_turn_time_default, nudge_move_time_default, nudge_tilt_time_default;
int tilt_up_speed_default;
int tilt_down_speed_default, degrees_default; 

// process a command string
void HandleCommand(char* input, int length)
{
  int speedToGo = speed_default;
  int degreesToTurn = degrees_default;  // we currently always use this, until website supports sending degrees to turn
  long value = 0, EEPROMaddress, EEPROMvalue;
  // calculate number following command
  long powerOf10 = 1;
  for (int j=0; j < length - 2; j++) powerOf10 *= 10;
  //SERIAL_PORT.print("length = ");
  //SERIAL_PORT.println(length);
  if (length > 1)
  {
     for (int i = 1; i < length; i++) 
     {
       char singlechar[2];
       singlechar[0] = input[i];
       singlechar[1] = 0;
       value += atoi(&singlechar[0]) * powerOf10;
       //SERIAL_PORT.print("powerof10, input[i], i, value = ");
       //SERIAL_PORT.print(powerOf10);
       //SERIAL_PORT.print(", ");
       //SERIAL_PORT.print(input[i]);
       //SERIAL_PORT.print(", ");
       //SERIAL_PORT.print(i);
       //SERIAL_PORT.print(", ");
       //SERIAL_PORT.println(value);
       powerOf10 /= 10;
     }
       
    //  {value = atoi(&input[1]);  // doesn't work for large numbers, as we have for EEPROM writes.
    SERIAL_PORT.print("atoi value = ");
    SERIAL_PORT.println(value);
    if (value < 256)
    {
      // speed or turn specified or eeprom address to read
      // if there is a gyro, use the input as degrees, speed will be default
      // otherwise, use the input as speed, degreesToTurn won't be used, nudgeturnTime will be default
      
      
 //     if (gyroPresent) degreesToTurn = value; // waiting for website to support this by sending degrees to turn*********
      
      
      speedToGo = value; // use this speed for turns and for moving forward and backward
      EEPROMaddress = value;
    }
    else  // means we are writing to the EEPROM
    {
      SERIAL_PORT.print("EEPROM command string parameter is ");
      SERIAL_PORT.println(value);
      EEPROMvalue=  value%1000;  // the lower three digits are the value
      SERIAL_PORT.print("EEPROM command value is ");
      SERIAL_PORT.println(EEPROMvalue);
      EEPROMaddress = (value - EEPROMvalue) / 1000; // the upper three digits are the address
      SERIAL_PORT.print("EEPROM command address is ");
      SERIAL_PORT.println(EEPROMaddress);
    }
  }

  // check commands
  // ************note that if you use more than one character here
  // the bytes are swapped, ie 'FM' means command MF *****************
  // you can use this stmt to get the command:
  // int* command = (int*)input;
  // but not needed when we just have a single character command format
  switch(input[0]) {
    
    // move forward
    case 't':
      move(speedToGo, nudge_move_time_default);  // forward a little
      break;
    case 'f':    
      move(speedToGo, 0);  // forward for the default time
      break;
    case 'F':
      move(speedToGo, -1);  // forward forever
      break;
      
    // move backward
    case 'g':
      move(-speedToGo, nudge_move_time_default);  // backward a little
      break;
    case 'b':
      move(-speedToGo, 0);  // backward for the default time
      break;
    case 'B':
      move(-speedToGo, 0);  // backward forever
      break;
      
    // turn right
    case 'y':
      if (gyroPresent) turn(speedToGo, degrees_default / 2);
      else turn(speedToGo, nudge_turn_time_default);  // turn right a little
      break;
    case 'r':    
      if (gyroPresent) turn(speedToGo, degreesToTurn);
      turn(speedToGo, turn_time_default);  // turn right for the default time
      break;
    case 'R':    
      if (gyroPresent) turn(speedToGo, degreesToTurn);
      else turn(speedToGo, turn_time_default);    // turn right forever
      break;
      
    // turn left
    case 'h':
      if (gyroPresent) turn(-speedToGo, degrees_default / 2);
      else turn(-speedToGo, nudge_turn_time_default);  // turn left a little
      break;
    case 'l':   // turn left
      if (gyroPresent) turn(-speedToGo, degrees_default);
      turn(-speedToGo, turn_time_default);  // turn left for the default time
      break;
    case 'L':
      if (gyroPresent) turn(-speedToGo, degreesToTurn);
      else turn(-speedToGo, turn_time_default);    // turn left for default time
      break;    
      
    // tilt up
    case 'i':
      tilt(tilt_up_speed_default, nudge_tilt_time_default);  // nudge tilt up
      break;
    case 'u':    // tilt up
      tilt(tilt_up_speed_default, 0);  // tilt up for the default time
      break;
    case 'U':    // tilt up forever
      tilt(tilt_up_speed_default, -1);  // tilt up forever
      break;
      
    // tilt down
    case 'k':
      tilt(-tilt_down_speed_default, nudge_tilt_time_default);  // nudge tilt down
      break;
    case 'n':    // tilt down
      tilt(-tilt_down_speed_default, 0);  // tilt down for the default time
      break;
    case 'N':    // tilt down forever
      tilt(-tilt_down_speed_default, -1);  // tilt down forever
      break;
      
      
    case 'x':    // stop drive wheels
    case 'X':
      Stop();
      break;
      
    case 'p':
      SERIAL_PORT_BLUETOOTH.print("MESSAGE_BATTERY_PERCENT");  // lead with "mb"  
                                      // 'm' indicates that this is a message for the server
                                      // 'b' indicates that it is a battery percent messsage
      SERIAL_PORT_BLUETOOTH.println(checkBattery()); 
      break;
      
    // EEPROM commands
    case 'E':
      EEPROMvalue = readFromEEPROM(EEPROMaddress);
      SERIAL_PORT_BLUETOOTH.print("MESSAGE_EEPROM_VALUE");  // lead with "mE"  
                                      // 'm' indicates that this is a message for the server
                                      // 'E' indicates that it is an EEPROM value
      SERIAL_PORT_BLUETOOTH.println(EEPROMvalue); 
      SERIAL_PORT.print("EEPROM requested:  address, value: ");
      SERIAL_PORT.print(EEPROMaddress);
      SERIAL_PORT.print(", ");
      SERIAL_PORT.println(EEPROMvalue); 
      break;
    case 'Z':
      enableEEPROMwrite = true;
      SERIAL_PORT.println("EEPROM writing enabled.");
      SERIAL_PORT_BLUETOOTH.println("EEPROM writing enabled.");
      break;
    case 'z':
      enableEEPROMwrite = false;
      SERIAL_PORT.println("EEPROM writing disabled.");
      SERIAL_PORT_BLUETOOTH.println("EEPROM writing disabled.");
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
      Stop();
      break;
  }
} 


