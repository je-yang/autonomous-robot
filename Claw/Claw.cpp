/*
	Claw.h - Library for claw actuation.
	Created by Team Calgary
*/

#include "Arduino.h"
#include "LiquidCrystal.h"
#include "phys253.h"
#include "Claw.h"
// #include "Servo.h"

//Input: pin input for claw servo
Claw::Claw()
{
	//no variables
}

//Open claw fully and hold 
void Claw::open()
{
	ServoTimer2 RCServo0();

	//Open claw fully
	RCServo0.write(180);
	LCD.print(RCServo0.read());

	//hold claw in open position
	RCServo0.detach();

}

//Close claw around passenger and hold 
void Claw::close()
{
	ServoTimer2 RCServo0();

	//Close claw around passenger
	RCServo0.write(90);
	LCD.print(RCServo0.read());

	//hold claw in closed position
	RCServo0.detach();
}