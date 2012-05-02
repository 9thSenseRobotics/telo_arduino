/*
  Receives Test Events from your phone.
  After it gets a test message the led 13 will blink.
*/
 
#include <MeetAndroidMega.h>

MeetAndroidMega meetAndroid;
int onboardLed = 13;
int sentValue, oldValue;

void setup()  
{
  // use the baud rate your bluetooth module is configured to 
  // not all baud rates are working well, i.e. ATMEGA168 works best with 57600
  SERIAL_PORT_BLUETOOTH.begin(115200); 
  Serial.begin(115200);
  
  // register callback functions, which will be called when an associated event occurs.
  // - the first parameter is the name of your function (see below)
  // - match the second parameter ('A', 'B', 'a', etc...) with the flag on your Android application
  meetAndroid.registerFunction(testEvent, 'A'); 
 meetAndroid.registerFunction(testEvent, 'r');
 meetAndroid.registerFunction(testEvent, 's');
 meetAndroid.registerFunction(testEvent, 'v');
 meetAndroid.registerFunction(testEvent, 'm'); 

  pinMode(onboardLed, OUTPUT);
  digitalWrite(onboardLed, HIGH);

}

void loop()
{
  meetAndroid.receive(); // you need to keep this in your loop() to receive events
  if (sentValue != oldValue) meetAndroid.send(sentValue);
  oldValue = sentValue;
  delay(1000);
}

/*
 * This method is called constantly.
 * note: flag is in this case 'A' and numOfValues is 1 (since test event sends a random int)
 */
void testEvent(byte flag, byte numOfValues)
{
  // the test event in Amarino generates a random value between 0 and 255
  //sentValue = meetAndroid.getInt();
  sentValue = (int) flag;
  Serial.print("value recieved = ");
  Serial.println(sentValue);
  flushLed(1000);
}

void flushLed(int time)
{
  digitalWrite(onboardLed, LOW);
  delay(time);
  digitalWrite(onboardLed, HIGH);
  delay(time);
}

