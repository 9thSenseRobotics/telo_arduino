#include "threeMotorsDriverPCB.h"
threeMotorsDriverPCB::threeMotorsDriverPCB()
{
    // designed for use with the VNH2SP30 motor driver boards from pololu
	// and our custom 9th Sense PCB
	// Pin map
    // pins used are 0,1,3,4,5,6,16,17,18,19,22,24,26,30,32,34,40,42
    // the analog pins used, A5,A6,A7,A8, are set as #defines in the threeMotorsDriverPCB.h file
    // if we later decide to use the servo library, then pins 9 and 10 will not be available for analogWrite()
    // available for future use: interrupt pins 2, PWM pins 7 - 13
    // available for I2C are pins 20 and 21 (these are also interrupt pins)
    IN1A = 22;
    IN2A = 24;
    STATUSA = 28;
    PWMA = 4;  // PWM: 0 to 13. Provide 8-bit PWM output with the analogWrite() function.
    IN1B = 32;
    IN2B = 30;
    STATUSB = 28;
    PWMB = 5;
    IN1C = 40;
    IN2C = 42;
    STATUSC = 36;
    PWMC = 6;
    // interrupt pins on the mega are:
    // 2 (interrupt 0), 3 (interrupt 1), 18 (interrupt 5), 19 (4), 20 (3), and 21 (2)
    // we use them for motor encoders
    ENCA = 18; 
	INTERRUPTA = 5;
    ENCB = 19;
	INTERRUPTB = 4;
    ENCC = 20;
	INTERRUPTC = 3;
	

    pinMode(IN1A,OUTPUT);
    pinMode(IN2A,OUTPUT);
    pinMode(STATUSA,INPUT);
    pinMode(PWMA,OUTPUT);  // PWM: 0 to 13. Provide 8-bit PWM output with the analogWrite() function.
    
    pinMode(IN1B,OUTPUT);
    pinMode(IN2B,OUTPUT);
    pinMode(STATUSB,INPUT);
    pinMode(PWMB,OUTPUT);
    
    pinMode(IN1C,OUTPUT);
    pinMode(IN2C,OUTPUT);
    pinMode(STATUSC,INPUT);
    pinMode(PWMC,OUTPUT);
    
    // interrupt pins on the mega are:
    // 2 (interrupt 0), 3 (interrupt 1), 18 (interrupt 5), 19 (4), 20 (3), and 21 (2)
    // we use them for motor encoders
    pinMode(ENCA,INPUT); 
    pinMode(ENCB, INPUT);
    pinMode(ENCC, INPUT);
	
	// turn on pullup resistors
	digitalWrite(ENCA, HIGH);
	digitalWrite(ENCB, HIGH);
	digitalWrite(ENCC, HIGH);
    
    // start disabled, with directions set to forward
    digitalWrite(IN1A,HIGH);
    digitalWrite(IN2A,LOW);    
    analogWrite(PWMA, 0);
    
    digitalWrite(IN1B,HIGH);
    digitalWrite(IN2B,LOW); 
    analogWrite(PWMB, 0);
    
    digitalWrite(IN1C,HIGH);
    digitalWrite(IN2C,LOW); 
    analogWrite(PWMC, 0);
}

void threeMotorsDriverPCB::setSpeedA(int speed)
{
    if (speed < 0)
    {
        digitalWrite(IN1A,LOW);
        digitalWrite(IN2A,HIGH);   // turn reverse
        speed = -speed;
    }
    else
    {
        digitalWrite(IN1A,HIGH);
        digitalWrite(IN2A,LOW);   // turn forward
    }
    if (speed > 255) speed = 255;
    analogWrite(PWMA, speed);
}

void threeMotorsDriverPCB::setSpeedB(int speed)
{
    if (speed < 0)
    {
        digitalWrite(IN1B,LOW);
        digitalWrite(IN2B,HIGH);   // turn reverse
        speed = -speed;
    }
    else
    {
        digitalWrite(IN1B,HIGH);
        digitalWrite(IN2B,LOW);   // turn forward
    }
    if (speed > 255) speed = 255;
    analogWrite(PWMB, speed);
}   

void threeMotorsDriverPCB::setSpeedC(int speed)
{
    if (speed < 0)
    {
        digitalWrite(IN1C,LOW);
        digitalWrite(IN2C,HIGH);   
        speed = -speed;
    }
    else
    {
        digitalWrite(IN1C,HIGH);
        digitalWrite(IN2C,LOW);  
    }
    if (speed > 255) speed = 255;
    analogWrite(PWMC, speed);
}

void threeMotorsDriverPCB::setSpeedAB(int speedA, int speedB)
{
    if (speedA < 0)
    {
        digitalWrite(IN1A,LOW);
        digitalWrite(IN2A,HIGH);   // move backwards
        speedA = -speedA;
    }
    else
    {
        digitalWrite(IN1A,HIGH);
        digitalWrite(IN2A,LOW);   // move forward
    }
    if (speedA > 255) speedA = 255;
    
    if (speedB < 0)
    {
        digitalWrite(IN1B,LOW);
        digitalWrite(IN2B,HIGH);   // turn reverse
        speedB = -speedB;
    }
    else
    {
        digitalWrite(IN1B,HIGH);
        digitalWrite(IN2B,LOW);   // turn forward
    }
    analogWrite(PWMA, speedA);
    analogWrite(PWMB, speedB);
}

void threeMotorsDriverPCB::setBrakesC()
{
    analogWrite(PWMC, 0);
}

void threeMotorsDriverPCB::setBrakesAB()
{
    analogWrite(PWMA, 0); 
	analogWrite(PWMB, 0); 
    digitalWrite(IN1A, LOW);
    digitalWrite(IN2A, LOW);
	digitalWrite(IN1B, LOW);
    digitalWrite(IN2B, LOW);
}
    
void threeMotorsDriverPCB::setCoastAB()
{
    analogWrite(PWMA, 0); 
	analogWrite(PWMB, 0);
    digitalWrite(IN1A, LOW);
    digitalWrite(IN2A, HIGH);
	digitalWrite(IN1B, LOW);
    digitalWrite(IN2B, HIGH);	
}

void threeMotorsDriverPCB::setCoastC()
{
    analogWrite(PWMC, 0); 
	digitalWrite(IN1C, LOW);
    digitalWrite(IN2C, HIGH);  
}

int threeMotorsDriverPCB::getCurrentA()  // returns mA
{
    // .13 volts per amp = 37.6 ma per count (0 - 1023), we'll use 38 to stick with integer arithmetic
    return (analogRead(FBA) * 38);
}

int threeMotorsDriverPCB::getCurrentB()
{
    return (analogRead(FBB) * 38);
}

int threeMotorsDriverPCB::getCurrentC()
{
    return (analogRead(FBC) * 38);
}

unsigned char threeMotorsDriverPCB::getStatusA()
{
    return(digitalRead(STATUSA));
}

unsigned char threeMotorsDriverPCB::getStatusB()
{
    return(digitalRead(STATUSB));
}

unsigned char threeMotorsDriverPCB::getStatusC()
{
    return(digitalRead(STATUSC));
}



