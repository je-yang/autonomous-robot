/*
	Arm.h - Library for arm actuation.
	Created by Team Calgary
*/

#include "Arduino.h"
#include "Claw.h"
#include "Servo.h"

//Input: pin input for arm servo
Arm::Arm(int pin)
{
	pinMode(pin, OUTPUT);
	_pinArm = pin;
}

//lifting the arm up
void Arm::lift()
{
	Servo servoArm;
	servoArm.attach(_pinArm);

	//lift arm to max height
	servoArm.write(90);

	//hold the arm at this height
	servoArm.detach();
}

//lowering the arm down
void Arm::lower()
{
	Servo servoArm;
	servoArm.attach(_pinArm);	

	//lower arm to minimum chosen height (height we want the claw to be)
	servoArm.write(55);

		//hold the arm at this height
	servoArm.detach();
}