/*
Code for claw mechanism 
1. Arm turns to face direction of passenger
2. Arm lowers to level of passenger
3. Arm extends forward until passenger is within reach
4. Claw closes around passenger
*/

#include <Servo.h>

int microSwitch = 1 //pin input: Microswitch for passenger detection

Servo servoArm; 
Servo servoClaw;

int servoArmPin = 2; //pin input: Servo for arm
int servoClawPin = 3; //pin input: Servo for claw

int posArm = 0; //initial servo position in degrees

void setup() {
  // put your setup code here, to run once:
  servoArm.attach(servoArmPin);
  servoClaw.attach(servoClawPin);
  pinMode(microSwitch, INPUT_PULLUP);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  //Initial position of arm and claw
  servoClaw.write(180); //Claw starts opened 
  servoArm.write(0); //Arm starts contracted

  //ARM ACTUATION - turning to face passenger 
  //might not need this depending on how and where we stop the vehicle


  //ARM ACTUATION - descending to correct passenger height 
  //occurs 

  //ARM ACTUATION - extension
  //when IR detected within certain strength, extend arm forward (in direction of passenger), 
  //until microswitch pushed in (i.e. passenger is pressed against claw)
  if(/*TODO: if IR detection is at certain strength*/){
    while(digitalRead(microSwitch) == LOW && posArm < 180){
      servoArm.write(posArm++);
    }
  } 

  //CLAW ACTUATION
  if(digitalRead(microSwitch) == HIGH){
    //When microswitch is on high, close the claw around passenger
    for(posClaw = 180; posArm < 90; posClaw--){
      servoClaw.write(posClaw);
    }
    //TODO: When destination beacon reached, drop off passenger 
  }
  
}