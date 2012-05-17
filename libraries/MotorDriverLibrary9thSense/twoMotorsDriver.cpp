#include "twoMotorsDriver.h"

// Motors are A: left motor, B: right motor
// Serial0 (pins 0 (RX) and 1 (TX)) is used for programming and debugging
// Serial2 (pins 17 (RX) and 16 (TX)) is used for bluetooth comm, note that impacts the bluetooth library files
// wiring of the 33926 driver:
// if we do not care to get coasting vs braking, then we can tie IN1 HIGH and IN2 LOW and use INV to select direction
// if we want coasting in addition to braking, then set IN1 and IN2 both LOW (or HIGH) and PWM to 255 to coast
// to brake, set ENABLE LOW or PWM to 0
// we will drive PWM using (not) D2, so that when it is LOW, the outputs are in the high impedence state
// so we keep D1 permanently LOW
// we will go with PWM frequencies below 10KHz, so we can use the slow slew, which in turn means lower peak current.
// so we can leave the SLEW pin unattached.  If we want to get up to high PWM frequencies, then we need to set it HIGH.
// we make the following switchable:  enable (EN), direction (INV), PWM (not D2)
// we read the following:  status (not SF), current draw (FB)
// internal VDD is generated in the chip, the external VDD is only for the jumper lines for overriding defaults,
// which allows fewer wires to be attached.
// So our jumper overrides are as follows:
// D1 to GND
// INVERT to GND, since we are using both IN1 and IN2 (so we can coast)
//
// For feedback, When running in the forward or reverse direction, a ground-referenced 0.24% of load current
// is output to the FB pin.  Since our board uses a 200 ohm resistor to ground from this pin, we get about
// 525 mV per amp on that pin.  Since the analog range is 0 to 1023 corresponding to 0 to 5V, or 4.89 mV per analog value,
// we get a value of 107.4 per amp, so full scale 1023 corresponds to 9.52 amps
// inverting, get a count of 9.3 mA per count
//
// wiring standards:
// red = +12
// black = ground
// yellow = +5
// white = motorA outside
// blue = motorA inside
// grey = motorB outside
// green = motor B inside
// orange = stepper 1A
// violet = stepper 1B
// yellow = stepper 2A  // have to start repeating colors, use ones that we have powerwerx colors
// green = stepper 2B
//

twoMotorsDriver::twoMotorsDriver()
{
    // Pin map
    // pins used are 4,5,18,19,26,32,34,40,44,50,52
    // this effectively uses pins 4,5,18,19, and even numbered pins from 26 - 52, since that allows
    // a continuous mapping directly frm the pololu dual driver board to the row of pin connections
    // which makes running the jumpers easy
    // the analog pins used, A5,A6 are set as #defines in the twoMotorsDriver.h file
    // if we later decide to use the servo library, then pins 9 and 10 will not be available for analogWrite()
    // available for future use: interrupt pins 2,3, PWM pins 9 - 13
    // available for I2C are pins 20 and 21 (these are also interrupt pins)
    
    IN1A = 50;
    IN2A = 52;
    STATUSA = 44;
    PWMA = 4;  // PWM: 0 to 13. Provide 8-bit PWM output with the analogWrite() function.
    
    IN1B = 32;
    IN2B = 34;
    STATUSB = 26;
    PWMB = 5;
    
    ENABLEAB = 40;
    

    // interrupt pins on the mega are:
    // 2 (interrupt 0), 3 (interrupt 1), 18 (interrupt 5), 19 (4), 20 (3), and 21 (2)
    // we use them for motor encoders
    ENCA = 18; 
    ENCB = 19;

    pinMode(IN1A,OUTPUT);
    pinMode(IN2A,OUTPUT);
    pinMode(STATUSA,INPUT);
    pinMode(ENABLEAB,OUTPUT);
    pinMode(PWMA,OUTPUT);  // PWM: 0 to 13. Provide 8-bit PWM output with the analogWrite() function.
    
    pinMode(IN1B,OUTPUT);
    pinMode(IN2B,OUTPUT);
    pinMode(STATUSB,INPUT);
    pinMode(PWMB,OUTPUT);
    
    pinMode(ENCA,INPUT); 
    pinMode(ENCB, INPUT);
    
    // start disabled, with directions set to forward
    digitalWrite(ENABLEAB, LOW);
    digitalWrite(IN1A,LOW);
    digitalWrite(IN2A,HIGH);    
    analogWrite(PWMA, 0);
    
    digitalWrite(ENABLEAB, LOW);
    digitalWrite(IN1B,LOW);
    digitalWrite(IN2B,HIGH); 
    analogWrite(PWMB, 0);
}

void twoMotorsDriver::setSpeedA(int speed)
{
    digitalWrite(ENABLEAB, HIGH);
    if (speed < 0)
    {
        digitalWrite(IN1A,HIGH);
        digitalWrite(IN2A,LOW);   // turn reverse
        speed = -speed;
    }
    else
    {
        digitalWrite(IN1A,LOW);
        digitalWrite(IN2A,HIGH);   // turn forward
    }
    if (speed > 255) speed = 255;
    analogWrite(PWMA, speed);
}

void twoMotorsDriver::setSpeedB(int speed)
{
    digitalWrite(ENABLEAB, HIGH);
    if (speed < 0)
    {
        digitalWrite(IN1B,HIGH);
        digitalWrite(IN2B,LOW);   // turn reverse
        speed = -speed;
    }
    else
    {
        digitalWrite(IN1B,LOW);
        digitalWrite(IN2B,HIGH);   // turn forward
    }
    if (speed > 255) speed = 255;
    analogWrite(PWMB, speed);
}   

void twoMotorsDriver::setSpeedAB(int speedA, int speedB)
{
    digitalWrite(ENABLEAB, HIGH);
    if (speedA < 0)
    {
        digitalWrite(IN1A,HIGH);
        digitalWrite(IN2A,LOW);   // turn reverse
        speedA = -speedA;
    }
    else
    {
        digitalWrite(IN1A,LOW);
        digitalWrite(IN2A,HIGH);   // turn forward
    }
    if (speedA > 255) speedA = 255;
    
    if (speedB < 0)
    {
        digitalWrite(IN1B,HIGH);
        digitalWrite(IN2B,LOW);   // turn reverse
        speedB = -speedB;
    }
    else
    {
        digitalWrite(IN1B,LOW);
        digitalWrite(IN2B,HIGH);   // turn forward
    }
    analogWrite(PWMA, speedA);
    analogWrite(PWMB, speedB);
}

void twoMotorsDriver::setBrakesAB()
{
    digitalWrite(ENABLEAB, LOW);
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);
}

void twoMotorsDriver::setCoastA()
{
    digitalWrite(ENABLEAB, HIGH);
    digitalWrite(IN1A, LOW);
    digitalWrite(IN2A, LOW);
    analogWrite(PWMA, 255);   
}
    
void twoMotorsDriver::setCoastB()
{
    digitalWrite(ENABLEAB, HIGH);
    digitalWrite(IN1B, LOW);
    digitalWrite(IN2B, LOW);
    analogWrite(PWMB, 255);   
}


void twoMotorsDriver::setCoastAB()
{
    digitalWrite(ENABLEAB, HIGH);
    digitalWrite(IN1A, LOW);
    digitalWrite(IN2A, LOW);
    digitalWrite(IN1B, LOW);
    digitalWrite(IN2B, LOW);
    analogWrite(PWMA, 255); 
    analogWrite(PWMB, 255);    
}

int twoMotorsDriver::getCurrentA()  // returns mA
{
    // 9.3 ma per count, we'll use 10 to stick with integer arithmetic
    return (analogRead(FBA) * 10);
}

int twoMotorsDriver::getCurrentB()
{
    return (analogRead(FBB) * 10);
}

unsigned char twoMotorsDriver::getStatusA()
{
    return(digitalRead(STATUSA));
}

unsigned char twoMotorsDriver::getStatusB()
{
    return(digitalRead(STATUSB));
}

