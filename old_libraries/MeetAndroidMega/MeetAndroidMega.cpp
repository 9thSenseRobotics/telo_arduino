/*
  MeetAndroidMega.cpp - Arduino Library for Amarino
  Copyright (c) 2009 Bonifaz Kaufmann.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

// Includes
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#include "WConstants.h"
#endif

#include "HardwareSerial.h"
#include "MeetAndroidMega.h"

extern "C" {
#include <stdlib.h>
}



// Private methods
void MeetAndroidMega::processCommand(){
	if(buffer[0]-FunctionBufferOffset < FunctionBufferLenght){
		void (*H_FuncPtr)(uint8_t, uint8_t) = intFunc[buffer[0]-FunctionBufferOffset];
		if (H_FuncPtr != 0) {
			H_FuncPtr(buffer[0], getArrayLength());
		}
		else {
			send("Flag not registered: ");
			send(buffer[0]);
		}
	}
	else {
		if (customErrorFunc)
			errorFunc(buffer[0], getArrayLength());
		else {
			send("Flag out of bounds: ");
			send(buffer[0]);
		}
	}
}


void MeetAndroidMega::init()
{
	waitTime = 30;
	startFlag = 18;
	ack = 19;
	abord = 27;
	delimiter = 59; //';'

	numberOfValues = 0;
	
	for(int a = 0;a < FunctionBufferLenght;a++){
		intFunc[a] = errorFunc;
	}
}


// public methods
MeetAndroidMega::MeetAndroidMega()
{
    // it is hard to use member function pointer together with normal function pointers.
    customErrorFunc = false;
	errorFunc = 0;
	init();
}

// Constructur for use with HardwareSerial library
MeetAndroidMega::MeetAndroidMega(H_voidFuncPtr err)
{
    customErrorFunc = true;
	errorFunc = err;
	init();
}

void MeetAndroidMega::registerFunction(void(*userfunction)(uint8_t, uint8_t),uint8_t command){
	intFunc[command-FunctionBufferOffset] = userfunction;
}
void MeetAndroidMega::unregisterFunction(uint8_t command){
	intFunc[command-FunctionBufferOffset] = errorFunc;
}

bool MeetAndroidMega::receive(){
	uint8_t lastByte;
	boolean timeout = false;
	while(!timeout)
	{
		while(SERIAL_PORT_BLUETOOTH.available() > 0)
		{
			lastByte = SERIAL_PORT_BLUETOOTH.read();
			
			if(lastByte == abord){
				flush();
			}
			else if(lastByte == ack){
				processCommand();
				flush();
			}
			else if(bufferCount < ByteBufferLenght){
				buffer[bufferCount] = lastByte;
				bufferCount++;
			}
			else return false;
		}
		
		if(SERIAL_PORT_BLUETOOTH.available() <= 0 && !timeout){
			if(waitTime > 0) delayMicroseconds(waitTime);
			if(SERIAL_PORT_BLUETOOTH.available() <= 0) timeout = true;
		}
	}
	return timeout;
}




void MeetAndroidMega::getBuffer(uint8_t buf[]){

	for(int a = 0;a < bufferCount;a++){
		buf[a] = buffer[a];
	}
}

void MeetAndroidMega::getString(char string[]){

	for(int a = 1;a < bufferCount;a++){
		string[a-1] = buffer[a];
	}
	string[bufferCount-1] = '\0';
}

int MeetAndroidMega::getInt()
{
	uint8_t b[bufferCount];
	for(int a = 1;a < bufferCount;a++){
		b[a-1] = buffer[a];
	}

	b[bufferCount-1] = '\0';
	return atoi((char*)b);
}

long MeetAndroidMega::getLong()
{
	uint8_t b[bufferCount];
	for(int a = 1;a < bufferCount;a++){
		b[a-1] = buffer[a];
	}

	b[bufferCount-1] = '\0';
	return atol((char*)b);
}

float MeetAndroidMega::getFloat()
{
	return (float)getDouble();
}

int MeetAndroidMega::getArrayLength()
{
	if (bufferCount == 1) return 0; // only a flag and ack was sent, not data attached
	numberOfValues = 1;
	// find the amount of values we got
	for (int a=1; a<bufferCount;a++){
		if (buffer[a]==delimiter) numberOfValues++;
	}
	return numberOfValues;
}

void MeetAndroidMega::getFloatValues(float values[])
{
	int t = 0; // counter for each char based array
	int pos = 0;

	int start = 1; // start of first value
	for (int end=1; end<bufferCount;end++){
		// find end of value
		if (buffer[end]==delimiter) {
			// now we know start and end of a value
			char b[(end-start)+1]; // create container for one value plus '\0'
			t = 0;
			for(int i = start;i < end;i++){
				b[t++] = (char)buffer[i];
			}
			b[t] = '\0';
			values[pos++] = atof(b);
			start = end+1;
		}
	}
	// get the last value
	char b[(bufferCount-start)+1]; // create container for one value plus '\0'
	t = 0;
	for(int i = start;i < bufferCount;i++){
		b[t++] = (char)buffer[i];
	}
	b[t] = '\0';
	values[pos] = atof(b);
}

// not tested yet
void MeetAndroidMega::getDoubleValues(float values[])
{
	getFloatValues(values);
}

// not tested yet
void MeetAndroidMega::getIntValues(int values[])
{
	int t = 0; // counter for each char based array
	int pos = 0;

	int start = 1; // start of first value
	for (int end=1; end<bufferCount;end++){
		// find end of value
		if (buffer[end]==delimiter) {
			// now we know start and end of a value
			char b[(end-start)+1]; // create container for one value plus '\0'
			t = 0;
			for(int i = start;i < end;i++){
				b[t++] = (char)buffer[i];
			}
			b[t] = '\0';
			values[pos++] = atoi(b);
			start = end+1;
		}
	}
	// get the last value
	char b[(bufferCount-start)+1]; // create container for one value plus '\0'
	t = 0;
	for(int i = start;i < bufferCount;i++){
		b[t++] = (char)buffer[i];
	}
	b[t] = '\0';
	values[pos] = atoi(b);
}


double MeetAndroidMega::getDouble()
{
	char b[bufferCount];
	for(int a = 1;a < bufferCount;a++){
		b[a-1] = (char)buffer[a];
	}

	b[bufferCount-1] = '\0';
	return atof(b);
	
}


#if defined(ARDUINO) && ARDUINO >= 100
size_t MeetAndroidMega::write(uint8_t b){
	return SERIAL_PORT_BLUETOOTH.print(b);
}
#else
void MeetAndroidMega::write(uint8_t b){
	SERIAL_PORT_BLUETOOTH.print(b);
}
#endif
	


void MeetAndroidMega::send(char c ){
	SERIAL_PORT_BLUETOOTH.print(startFlag);
	SERIAL_PORT_BLUETOOTH.print(c);
	SERIAL_PORT_BLUETOOTH.print(ack);
}

void MeetAndroidMega::send(const char str[]){
	SERIAL_PORT_BLUETOOTH.print(startFlag);
	SERIAL_PORT_BLUETOOTH.print(str);
	SERIAL_PORT_BLUETOOTH.print(ack);
}
void MeetAndroidMega::send(uint8_t n){
	SERIAL_PORT_BLUETOOTH.print(startFlag);
	SERIAL_PORT_BLUETOOTH.print(n);
	SERIAL_PORT_BLUETOOTH.print(ack);
}
void MeetAndroidMega::send(int n){
	SERIAL_PORT_BLUETOOTH.print(startFlag);
	SERIAL_PORT_BLUETOOTH.print(n);
	SERIAL_PORT_BLUETOOTH.print(ack);
}
void MeetAndroidMega::send(unsigned int n){
	SERIAL_PORT_BLUETOOTH.print(startFlag);
	SERIAL_PORT_BLUETOOTH.print(n);
	SERIAL_PORT_BLUETOOTH.print(ack);
}
void MeetAndroidMega::send(long n){
	SERIAL_PORT_BLUETOOTH.print(startFlag);
	SERIAL_PORT_BLUETOOTH.print(n);
	SERIAL_PORT_BLUETOOTH.print(ack);
}
void MeetAndroidMega::send(unsigned long n){
	SERIAL_PORT_BLUETOOTH.print(startFlag);
	SERIAL_PORT_BLUETOOTH.print(n);
	SERIAL_PORT_BLUETOOTH.print(ack);
}
void MeetAndroidMega::send(long n, int base){
	SERIAL_PORT_BLUETOOTH.print(startFlag);
	SERIAL_PORT_BLUETOOTH.print(n, base);
	SERIAL_PORT_BLUETOOTH.print(ack);
}
void MeetAndroidMega::send(double n){
	SERIAL_PORT_BLUETOOTH.print(startFlag);
	SERIAL_PORT_BLUETOOTH.print(n);
	SERIAL_PORT_BLUETOOTH.print(ack);
}
void MeetAndroidMega::sendln(void){
	SERIAL_PORT_BLUETOOTH.print(startFlag);
	SERIAL_PORT_BLUETOOTH.println();
	SERIAL_PORT_BLUETOOTH.print(ack);
}

void MeetAndroidMega::flush(){
	for(uint8_t a=0; a < ByteBufferLenght; a++){
		buffer[a] = 0;
	}
	bufferCount = 0;
	numberOfValues = 0;
}
