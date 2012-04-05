/* Freeduino Simple sketch
* Serial speed is 57500 because Arduino serial monitor locks at 115200
* No analogWrite on pins 9 and 10 if use servo lib with UNO board
*/

#include <Max3421e.h>
#include <Usb.h>
#include <FHB.h>
#include <Servo.h>


#define tiltPin 2
#define panPin 3
#define rightMotorPWM 5
#define leftMotorPWM 6
#define rightMotorDirectionA 4
#define rightMotorDirectionB 7
#define leftMotorDirectionA 8
#define leftMotorDirectionB 9

#define SERIALSPEED 57600

#define TIMED_OUT 8000
#define DEFAULT_SPEED 220
#define LEFT_MOTOR_BIAS 10

#define TILT_CENTER 53
#define TILT_LOOK_DOWN 80
#define TILT_MIN 20
#define TILT_MAX 105
#define TILT_DELTA 10

#define PAN_CENTER 83
#define PAN_MIN 0
#define PAN_MAX 170
#define PAN_DELTA 10

#define BATTERY_MONITOR_PIN A5
#define ZERO_PERCENT_BATTERY_VOLTAGE 10.5
#define FULL_BATTERY_VOLTAGE 13.0
#define VOLTAGE_DIVIDER_RATIO 3.18


AndroidAccessory acc("Google, Inc.",
		     "DemoKit",
		     "DemoKit Arduino Board",
		     "1.0",
		     "http://www.android.com",
		     "0000000012345678");

Servo panServo, tiltServo;  // create servo objects to control the servos
int panPos, tiltPos;    // variable to store the servo position 
long timeOutCheck;
int batteryMonitorPin;
float batteryRange;

void move(int speed) // speed goes from -255 to 255
{
  if (speed < 0) {
    digitalWrite(rightMotorDirectionA, LOW);
    digitalWrite(rightMotorDirectionB, HIGH);
    digitalWrite(leftMotorDirectionA, LOW);
    digitalWrite(leftMotorDirectionB, HIGH);
    speed = -speed;
  }
  else {
    digitalWrite(rightMotorDirectionA, HIGH);
    digitalWrite(rightMotorDirectionB, LOW);
    digitalWrite(leftMotorDirectionA, HIGH);
    digitalWrite(leftMotorDirectionB, LOW);
  }

  if (speed > 255) speed = 255;  
    
  analogWrite(rightMotorPWM, speed);
  analogWrite(leftMotorPWM, speed);
}  

void turn(int speed) // speed goes from -255 to 255
{
  if (speed < 0) {
    digitalWrite(rightMotorDirectionA, LOW);
    digitalWrite(rightMotorDirectionB, HIGH);
    digitalWrite(leftMotorDirectionA, HIGH);
    digitalWrite(leftMotorDirectionB, LOW);
    speed = -speed;
  }
  else {
    digitalWrite(rightMotorDirectionA, HIGH);
    digitalWrite(rightMotorDirectionB, LOW);
    digitalWrite(leftMotorDirectionA, LOW);
    digitalWrite(leftMotorDirectionB, HIGH);
  }

  if (speed > 255) speed = 255;  
    
  analogWrite(rightMotorPWM, speed);
  analogWrite(leftMotorPWM, speed);
}  

void Stop()
{
  analogWrite(rightMotorPWM, 0);
  analogWrite(leftMotorPWM, 0);
}
  
int checkBattery()
{
  float voltage =  (float) ((analogRead(batteryMonitorPin) / 1023.) * 5.0 ) * VOLTAGE_DIVIDER_RATIO;
  int batteryPercent =  (int) ( 100. * ( ( voltage - ZERO_PERCENT_BATTERY_VOLTAGE) / batteryRange)); // returns percentage
  if (batteryPercent > 99) return 99;
  if (batteryPercent < 0) return 0;
  return batteryPercent;  
} 


void setup()
{
	Serial.begin(SERIALSPEED);
        
        pinMode(rightMotorDirectionA, OUTPUT);
	pinMode(leftMotorDirectionA, OUTPUT);
        pinMode(rightMotorDirectionB, OUTPUT);
	pinMode(leftMotorDirectionB, OUTPUT);

	acc.powerOn();

        batteryRange = FULL_BATTERY_VOLTAGE - ZERO_PERCENT_BATTERY_VOLTAGE;
        batteryMonitorPin = BATTERY_MONITOR_PIN;
        
        tiltServo.attach(tiltPin); 
        panServo.attach(panPin);  // attaches the pan servo pin to the servo object
        panServo.write(PAN_CENTER);
        tiltServo.write(TILT_CENTER);
        panPos = PAN_CENTER;
        tiltPos = TILT_CENTER;
}

void loop()
{
	/*byte msg[3];
        int speedToGo = DEFAULT_SPEED;
        int stepsToGo = 1;
	if (acc.isConnected()) {
	  int len = acc.read(msg, sizeof(msg), 1);
          if (len > 0) {
            Serial.println();
            Serial.println(msg[0]);
            Serial.println(msg[1]);
            Serial.println(msg[2]);

	    if (msg[1])
            {
              speedToGo = msg[2];
              stepsToGo = msg[2];
            }
            
            switch (msg[0])
            {
                  case 'W':    // move forward
                  case 'w':
                  case 'f':    // just for testing with web ***********************************
                    move(-speedToGo);
                    break;
                  case 'S':    // move backward
                  case 's':
                  case 'b':   // just for testing with web ***********************************
                    move(speedToGo);
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
                    Stop();
                    break;
                    
                  case 'J':    // center servos
                  case 'j':
                    panServo.write(PAN_CENTER);
                    tiltServo.write(TILT_CENTER);
                    panPos = PAN_CENTER;
                    tiltPos = TILT_CENTER;
                    break;
                  case 'N':    // tilt down
                  case 'n':
                    if (tiltPos + (TILT_DELTA * stepsToGo) <= TILT_MAX) tiltPos += TILT_DELTA * stepsToGo;
                    else tiltPos = TILT_MAX;
                    tiltServo.write(tiltPos);
                    break;
                  case 'U':    // tilt up
                  case 'u':
                    if (tiltPos - (TILT_DELTA * stepsToGo) >= TILT_MIN) tiltPos -= TILT_DELTA * stepsToGo;
                    else tiltPos = TILT_MIN;
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
                    tiltPos = TILT_MAX;
                    tiltServo.write(tiltPos);
                    break;
                  case 'Y':    // tilt look down
                  case 'y':
                    tiltPos = TILT_LOOK_DOWN;
                    tiltServo.write(tiltPos);
                    break;
                  case 'G':    // pan center
                  case 'g':
                    panPos = PAN_CENTER;
                    panServo.write(panPos);
                    break;
                  case 'R': // relax the servos
                  case 'r':
                    break;  // not implemented yet
                  
                   */ 
                  /*case 'P':
                  case 'p':
                    digitalWrite(powerPin, LOW);
                    delay(500);
                    digitalWrite(powerPin,HIGH);
                    delay(500);
                    digitalWrite(powerPin,LOW);
                    break;
                   
                   
                  case 'B':
                  case 'b':
                    Serial.print(checkBattery());
                    break;
                  */ 
                 /* 
                  default:
                    //Serial.println("did not recognize command ");
                    break;
                }
            }
          
        }
        
        */
        Serial.println("forward");
        for (int i = 80; i<256; i++)
        {
         move(i);
         delay(100);
        } 
        Serial.println("reverse");
        move(-200);
        delay(5000);
        move(0);
        Serial.println("tilt");
        for (int i = 80; i<120; i++)
        {
         tiltServo.write(i);
         delay(100);
        } 
        Serial.println("pan");
        for (int i = 80; i<120; i++)
        {
         panServo.write(i);
         delay(100);
        }
}

