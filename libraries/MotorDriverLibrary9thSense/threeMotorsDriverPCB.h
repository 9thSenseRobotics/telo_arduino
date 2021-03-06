#ifndef threeMotorsDriverPCB_h
#define threeMotorsDriverPCB_h

#include <Arduino.h>

// analog inputs
#define FBA A5
#define FBB A6
#define FBC A7
    
class threeMotorsDriverPCB
{
  public:  
    // CONSTRUCTOR
    threeMotorsDriverPCB(); // pin selection an initial config
    
    // PUBLIC METHODS
    void setSpeedA(int speed); // Set speed for left motor
    void setSpeedB(int speed); // Set speed for right motor
    void setSpeedC(int speed); // Set speed for top motor
    void setSpeedAB(int speedA, int speedB); // Set speed for left and right motors
    void setBrakesAB();
    void setBrakesC();
    void setCoastAB();
	void setCoastC();
    int getCurrentA();
    int getCurrentB();
    int getCurrentC();
    unsigned char getStatusA(); // Get status of left motor
    unsigned char getStatusB(); // Get status of right motor
    unsigned char getStatusC(); // Get status of top motor
	
	// interrupt pins on the mega are:
    // 2 (interrupt 0), 3 (interrupt 1), 18 (interrupt 5), 19 (4), 20 (3), and 21 (2)
    // we use them for motor encoders
    unsigned char ENCA; 
	unsigned char INTERRUPTA;
    unsigned char ENCB;
	unsigned char INTERRUPTB;
    unsigned char ENCC;
	unsigned char INTERRUPTC;
	
    
  private:
    unsigned char IN1A;
    unsigned char IN2A;
    unsigned char STATUSA;
    unsigned char PWMA;  // PWM: 0 to 13. Provide 8-bit PWM output with the analogWrite() function.
    
    unsigned char IN1B;
    unsigned char IN2B;
    unsigned char STATUSB;
    unsigned char PWMB;
    
    unsigned char IN1C;
    unsigned char IN2C;
    unsigned char STATUSC;
    unsigned char PWMC;
    
};

#endif
