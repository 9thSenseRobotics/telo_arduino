#ifndef twoMotorsDriver_h
#define twoMotorsDriver_h

#include <Arduino.h>

// analog inputs

#define FBA A5
#define FBB A6
    
class twoMotorsDriver
{
  public:  
    // CONSTRUCTOR
    twoMotorsDriver(); // pin selection and initial config
    
    // PUBLIC METHODS
    void setSpeedA(int speed); // Set speed for left motor
    void setSpeedB(int speed); // Set speed for right motor
    void setSpeedAB(int speedA, int speedB); // Set speed for left and right motors
    void setBrakesAB();
    void setCoastA();
    void setCoastB();
    void setCoastAB();
    int getCurrentA();
    int getCurrentB();
    unsigned char getStatusA(); // Get status of left motor
    unsigned char getStatusB(); // Get status of right motor
    
  private:
    unsigned char IN1A;
    unsigned char IN2A;
    unsigned char STATUSA;
    unsigned char ENABLEAB;
    unsigned char PWMA;  // PWM: 0 to 13. Provide 8-bit PWM output with the analogWrite() function.
    
    unsigned char IN1B;
    unsigned char IN2B;
    unsigned char STATUSB;
    unsigned char PWMB;

    // interrupt pins on the mega are:
    // 2 (interrupt 0), 3 (interrupt 1), 18 (interrupt 5), 19 (4), 20 (3), and 21 (2)
    // we use them for motor encoders
    unsigned char ENCA; 
    unsigned char ENCB;
    
};

#endif
