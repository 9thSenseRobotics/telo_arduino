
// accepts serial port inputs and responds with moves and tilts
// requires a Mega to work, not an Uno or Nano, due to the need for two serial inputs

// version 0.81:
// cleans up commands to move forever to accomodate local driving
// turns now go a commanded number of degrees, note that the website still does not support this
// goes straight on all moves, both limited time and go forever
// allows multiple parameters to be input, separated by commas
// note that this means turn commands now have the form of direction (R or L) then speed then a comma, then degrees then the end character,e.g., R220,8#

// version 0.80:
// To make code much easier to read, it is now broken up into multiple files.
// The files have odd names because they are concatenated by alphabetical order (except that the
// filename that corresponds to the sketch folder name loads first), so the
// names have leading letters to control that.  Mostly we just need to get EEPROM loaded last
// since it uses many of the variable definitions from the other files.
// Seems that filenames have to start with a letter, otherwise I would have led with numbers
// Also, you can just put comments in the file that has the sketch name and then put setup and loop
// sections in files with names like z_setup and z_loop, so that they are concatenated last
// IMU is now incorporated, for now this allows precise turns.
// Future development will add auto straightening of paths, awareness of stalled state, and some
// data on speed and distance
// 
// version 0.74:
// EEPROM reading and writing work nicely
// added some default parameters
// version 0.73:
// Uses coast instead of Stop when commanded move or turn speed = 0
// Sends correct lead character ('e') when echoing command
// version 0.72:
// revised current limits and also added boolean for whether or not to use current limits, default is true
// version 0.71:
// even simpler routines now, just turn and move
// fixed but in turn that had us turning only to the right
//
// version 0.70 changes:
// EEPROM writes and reads are now used for all dynamic parameters
// move, turn, and tilt routines simplified

// version 0.63 changes:
// Bias values to compensate for turning are now written to eeprom, so they can be changed remotely
// commands added to do that.
// Bug fixed in MoveForward to get sign of mySpeed correct


// wiring:
// motor A is the left motor, motor B the right.
// hook them both up so that the outside wire from the motor goes into the outside terminal
// that corresponds to M1A for the right motor, M2B for the left motor
// see the comments in threeMotorsDriverPCB.cpp for details.

// for the battery monitor, hook it up with the + side soldered to Vout on the motor driver (+12)
// the negative side to ground and the middle to A4.  With the battery attached, 
// measure the voltage on Vout and divide it by the voltage on A4
// and enter that as the parameter VOLTAGE_DIVIDER_RATIO (assuming 22K and 10K resistors, the value should be 3.2)
// Also measure the fully charged battery value and enter that with the discharged value into the parameters
// FULL_BATTERY_VOLTAGE and ZERO_PERCENT_BATTERY_VOLTAGE

// command form is a letter for direction:
// f,b,r,d = move forward, move backward, right turn, left turn.
// upper case means move forever
// t,g move forward or back a little
// followed by the speed to move, 0 to 255
// followed by a character to indicate that the input is complete, in our case that character is #
// for example, move forward with a speed of 200 is
// f200#
// to turn right at full speed is
// r255#
// if you wish to use the default speed (a defined value), then just the letters is sufficient:
// r#
// 
// for tilting
// u, n = move up, down
// followed by the number of steps to take, for example, to move 2 steps up:
// U2#
// if there is no number, then the servo will just move by one step (TILT_DELTA)
// U#

// for the arduino mega, pin 47 corresponds to pin T5 on the ATMEL 2560
// ATMEL T0 = arduino mega 38
// ATMEL T1 = arduino mega NOT MAPPED uno pin 5
// ATMEL T3 = arduino mega NOT MAPPED
// ATMEL T4 = arduino mega NOT MAPPED
// ATMEL T5 = arduino mega 47

//The UNO has 3 Timers and 6 PWM output pins. The relation between timers and PWM outputs is:
//Pins 5 and 6: controlled by timer0
//Pins 9 and 10: controlled by timer1 and the servo library uses timer1, which is why 9 and 10 are not avail for PWM when you are using the servo library
//Pins 11 and 3: controlled by timer2

//On the Arduino Mega we have 6 timers and 15 PWM outputs:
//Pins 4 and 13: controlled by timer0
//Pins 11 and 12: controlled by timer1
//Pins 9 and10: controlled by timer2
//Pin 2, 3 and 5: controlled by timer 3
//Pin 6, 7 and 8: controlled by timer 4
//Pin 46, 45 and 44:: controlled by timer 5


// these are used by most of the programs:

// SERIAL_PORT is used to print locally to the serial monitor (diagnostics) and is the port for program updates
#define SERIAL_PORT Serial  
// SERIAL_PORT_BLUETOOTH is the port used for comm with the laptop or tablet.  Note that you can set this to the same value as SERIAL_PORT
// and then the serial monitor I/O will mimic the I/O from a robot.  This is very useful for testing. 
#define SERIAL_PORT_BLUETOOTH Serial2


unsigned long timeOutCheck;  // if the robot is moving and the arduino has not heard
                    // from the laptop or tablet in this time, then the robot will stop moving.

