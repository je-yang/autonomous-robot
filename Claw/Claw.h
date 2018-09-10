/*
	Claw.h - Library for claw actuation.
	Created by Team Calgary
*/

#ifndef Claw_h
#define Claw_h

#include "Arduino.h"

class Claw
{
	public:
		Claw(int pin);
		void open();
		void close();
	//private: 
		//int _pinClaw;
};

#endif