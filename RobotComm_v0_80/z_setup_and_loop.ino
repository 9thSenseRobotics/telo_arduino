
bool firstTimeThroughMoveForever = true;
long previousTime = 0;  // prevents weird error if there is a bug and this is used before being set

void baselineGyro()
{
   if ((!(Moving || Turning)) && gyroPresent)
   {
      updateGyroBaseline(1); // might as well update the gyro baseline a little
             // since we are just waiting around anyway.  It only takes 1 to 2 msec
             // so there is no need to shorten the delay time (next step) when it runs.
      firstTimeThroughMoveForever = true; // this resets the parameter for the move forever code below
   }
}
       
void checkMovingForwardForever()
{
   double initialYaw;
   if (Moving && gyroPresent) // implies that we got a move forever command
   {
      if (firstTimeThroughMoveForever)
      {
        firstTimeThroughMoveForever = false;
        initialYaw = totalYaw;
      }
      else
      {
       // goStraight(initialYaw, millis() - previousTime);
        commandMove(commandForeverSpeed);
      }
      previousTime = millis();  // bad practice to set this after it is used, but cannot find another way.
          // will be fine as long as firstTimeThroughMoveForever is set properly, but if it is false when it
          // should be true, that will induce some wild errors, as totalYaw will get a bad deltaT input
          // so the totalYaw global value will then be wrong
   }    
}


void setup()  
{
  setDefaults();
  // get currents at the start, since the first value seems to often be a large number
  getMotorCurrents();
  
  SERIAL_PORT.begin(SERIAL_SPEED);
  SERIAL_PORT_BLUETOOTH.begin(BLUETOOTH_SPEED);   // usually connect to bluetooth on serial2
  SERIAL_PORT.println("Serial ports initialized, ready for commands");
  
  coast();
  coastTilt();
  
  Wire.begin();
  gyroPresent = Gyro_Init();
  
  monitorMotorCurrents();  // should be 0
  SERIAL_PORT.print("left_motor_bias_default, right_motor_bias_default =  ");
  SERIAL_PORT.print(left_motor_bias_default);
  SERIAL_PORT.print(", ");
  SERIAL_PORT.println(right_motor_bias_default);
  
  SERIAL_PORT.println("website does not yet support specifying turn degrees, so we just use ");
  SERIAL_PORT.print("degrees_default =  ");
  SERIAL_PORT.println(degrees_default);  
  
  // clear the input buffer
  for (int i=0; i< INPUT_BUFFER_SIZE; i++) inputBuffer[i] = 0; 
  
}

void loop()
{
  double initialYaw;
  long previousTime;
  bool firstTimeThroughMoveForever;
  
  inputLength = 0;
  charIn = 0;
  while (charIn != COMMAND_END_CHARACTER && inputLength < INPUT_BUFFER_SIZE)
  {
    while (!SERIAL_PORT_BLUETOOTH.available()) // wait for input
    {
      /*if ((millis() - timeOutCheck > timed_out_default) && (Moving || Turning || Tilting))
      {
        SERIAL_PORT.println("timed out");
        Stop();  //if we are moving and haven't heard anything in a long time, stop moving
        coastTilt();  // stop tilting too        
      }*/
      // some things to do while waiting for serial inputs
      //getMotorCurrents();
      monitorMotorCurrents();
      checkMovingForwardForever();
      baselineGyro();    
      delay(10);
    }
    // got a character on the bluetooth port
    charIn = SERIAL_PORT_BLUETOOTH.read(); // read it in
    if (charIn != 13 && charIn != 10 && charIn != 32)  // ignore carriage returns, line feeds, and spaces
    {
       inputBuffer[inputLength] = charIn;  // building the command string
       inputLength++;
       //SERIAL_PORT.print(charIn);
    }
  } // this loop keeps going until the COMMAND_END_CHARACTER is received or the string gets too long
  
  //now flush anything that came after the COMMAND_END_CHARACTER
  if (SERIAL_PORT_BLUETOOTH.available())
  {
    SERIAL_PORT.print("Extra character(s) received have ASCII values = ");  // this is usually the null char end string (0)
    while (SERIAL_PORT_BLUETOOTH.available())
    {
       charIn = SERIAL_PORT_BLUETOOTH.read(); // read it in so we can see what is really getting sent
       if (charIn == 13) SERIAL_PORT.print(" 13 ");
       else if (charIn == 10) SERIAL_PORT.print(" 10 ");
       else if (charIn == 32) SERIAL_PORT.print(" 32 ");
       else if (charIn == 0) SERIAL_PORT.print(" 0 (NULL character) ");
       else SERIAL_PORT.print(charIn);
       SERIAL_PORT.print(" ");
    }
    SERIAL_PORT.println();
  }
  
  // throw away the command end character
  inputLength -= 1;  // -1 because it is incremented after the last character
  inputBuffer[inputLength] = 0;  // change COMMAND_END_CHARACTER to a 0 (remember, index is one behind inputLength)
  
 SERIAL_PORT.print("commanded: ");
 SERIAL_PORT.println(inputBuffer);

  // if the command == COMM_CHECK_CHARACTER it is just a local bluetooth comm check
  if (inputBuffer[0] != COMM_CHECK_CHARACTER && inputLength > 0)
  {
      // echo the command, so that the android app knows we are alive
    SERIAL_PORT_BLUETOOTH.print(COMMAND_ECHO_CHARACTER); // lead with this character to indicate just a command echo
    SERIAL_PORT_BLUETOOTH.println(inputBuffer); // need to println because android uses the CR as a delimiter
    HandleCommand(inputBuffer, inputLength);  // this goes after the echo, so that responses will show up on the details screen
  }
  else  // just a comm check
  {
    SERIAL_PORT_BLUETOOTH.print("c");  // this indicates it is just a response to a comm check
    SERIAL_PORT_BLUETOOTH.println(checkBattery()); // might as well send along the battery state
  } 
    
  // clear the buffer for the next command
  for (int i = 0; i < inputLength; i++) inputBuffer[i] = 0;
}

