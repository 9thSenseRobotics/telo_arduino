#define BufferLength 255
#define LineEndCharacter '#' // serial input commands must end with this character

char inputBuffer[BufferLength];

void getSerialCommandAndSendItToBluetooth()
{
  int inputLength = 0; 
  for (int i=0; i < inputLength; i++) inputBuffer[i] = 0;
  Serial.println("Enter command: ");  
  do
  {
      while (!Serial.available()) // wait for input
      {
        delay(100);
      }     
      inputBuffer[inputLength] = Serial.read(); // read it in
  } while (inputBuffer[inputLength] != LineEndCharacter && ++inputLength < BufferLength);
    
  inputBuffer[inputLength] = 0; // delete the end character
  Serial2.println(inputBuffer);  // send it to bluetooth
  Serial.print(inputBuffer); // echo what we sent
  Serial.println(" sent to Bluetooth");
}

void displayBluetoothResponse()
{
  // get a response from bluetooth
  int outputLength = 0, count = 0, fullCount = 0;
  do
  {
    while (!Serial2.available() && count < 3) // wait for input
    {
      delay(100);
      count++;
    }
    if (Serial2.available())
    {
      char inChar = Serial2.read(); // read it in
      Serial.print(inChar); // show it
      outputLength++;     
    }
    else break;
    count = 0;
    //fullCount++;
  } while (outputLength < BufferLength); // && fullCount < 7 );
  if (outputLength == 0) Serial.println("Nothing received from Bluetooth");
  Serial.println();
  Serial.println();
}
 
void setup()
{    
  Serial.begin(115200);   // connect to laptop
  Serial2.begin(115200);  // connect to bluetooth
   
  Serial.println("Note: $$$ is the command to go to command mode.");   
  Serial.println("Connected.  Power cycle the bluetooth module.");
  Serial.println("When you are ready, send E$ to enter command mode: ");
  while (!Serial.available());
  Serial.println("Going into command mode, wait for data to showup, then test with D#");
   
  
  Serial2.print("$$$"); // NOTE-- this MUST not have println
  delay(100);
  Serial2.println("D");
  
  displayBluetoothResponse();
}

void loop()
{
  getSerialCommandAndSendItToBluetooth();
  displayBluetoothResponse();
}

