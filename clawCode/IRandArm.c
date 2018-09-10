/*
Testing IR detection with arm motion
*/

#include <LiquidCrystal.h>
//#include <Claw.h>
#include <phys253.h>

//Claw claw(0);
//Pin allocations
int irPin_right = 0;
int irPin_left = 1;
int irPin_arm = 2;

//IR detection on left and right of vehicle

//IR detection on arm
double currentIRSignal_arm = 0;
double strongestIRSignal_arm = 0;
double locationStrongestIRSignal_arm = 0;

int nextStep_height = 0;
int pos = 0;
double thresholdIRSignal = 0.5;


void setup() {
#include <phys253setup.txt>

  ServoTimer2 RCServo0; //Controls the rotation and position of the base
  ServoTimer2 RCServo1; //Controls ascending and descending of arm
  //ServoTimer2 RCServo2; //Controls claw actuation

 // RCServo0.attach(0);
  //RCServo1.attach(1);
  //RCServo2.attach(2);
  //Initial positions
  RCServo0.write(90); //Position of the base starts in the middle
  RCServo1.write(180); //Arm starts up
  //RCServo2.write(180); //Claw starts open

  //Set pin mode for microswitch passenger detection
  //pinMode(detectionPin_passenger, INPUT);
  //digitalWrite(detectionPin_passenger, LOW);
  //pinMode(0, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  //Check if servos are properly attached
  //LCD.clear();
  //LCD.print(RCServo0.attached());
  //LCD.setCursor(0, 6);
  //LCD.print(RCServo1.attached());
  //locationIRSignal = RCServo0.read();
  strongestIRSignal_arm = analogRead(irPin_arm) * 5.0 / 1024.0;
  LCD.clear();
  LCD.print(strongestIRSignal_arm);
  delay(1000);

  //Base rotation - Find the direction of the strongest IR signal
  //Left side
      //GOOD--------------
  if (analogRead(irPin_arm) * 5.0 / 1024.0 > thresholdIRSignal) {
    RCServo0.write(0);
    LCD.clear();
    LCD.print("Passed left");
    delay(1000);
    //------
    for (pos = 0; pos <= 90; pos++) {
      RCServo0.write(pos);
      LCD.clear();
      LCD.print(pos);
      currentIRSignal_arm = analogRead(irPin_arm) * 5.0 / 1024.0;
      if (currentIRSignal_arm > strongestIRSignal_arm) {
        strongestIRSignal_arm = currentIRSignal_arm;
        locationStrongestIRSignal_arm = pos;
        LCD.setCursor(0, 1);
        LCD.print(locationStrongestIRSignal_arm);
        LCD.setCursor(8, 1);
        LCD.print(strongestIRSignal_arm);
      }
      delay(500);
    }
    //------
    nextStep_height = 1;
    LCD.clear();
    LCD.print("Made it!");
    delay(1000);
    LCD.setCursor(5, 0);
    LCD.print(locationStrongestIRSignal_arm);
    RCServo0.write(locationStrongestIRSignal_arm);
    //LCD.clear();
    //LCD.print("Strong IR left");
  }
  //GOOD--------------
  
/*
  //Right side
  if (analogRead(irPin_right) * 5.0 / 1024.0 > thresholdIRSignal) {
    RCServo0.write(180);
    LCD.clear();
    LCD.print("Passed right");
    delay(1000);
    //------
    for (pos = 180; pos >= 90; pos--) {
      RCServo0.write(pos);
      LCD.clear();
      LCD.print(pos);
      currentIRSignal_arm = analogRead(irPin_arm) * 5.0 / 1024.0;
      if (currentIRSignal_arm > strongestIRSignal_arm) {
        strongestIRSignal_arm = currentIRSignal_arm;
        locationStrongestIRSignal_arm = pos;
        LCD.setCursor(0, 1);
        LCD.print(locationStrongestIRSignal_arm);
        LCD.setCursor(8, 1);
        LCD.print(strongestIRSignal_arm);
      }
      delay(500);
    }
    //------


    nextStep_height = 1;
    LCD.clear();
    LCD.print("Made it!");
    delay(1000);
    LCD.setCursor(5, 0);
    LCD.print(locationStrongestIRSignal_arm);
    RCServo0.write(locationStrongestIRSignal_arm);
    //LCD.clear();
    //LCD.print("Strong IR left");
  }
*/  
  if (nextStep_height == 1) {
    //Arm height - lower arm to height of passenger
    RCServo1.write(0);
    LCD.clear();
    LCD.print("Height change");
    delay(1000);
    nextStep_height = 0;
    //TODO - Arm extension
    //microswitch controlling when claw closes
/*
    //Claw closure - picking up passenger
    RCServo2.write(100);
    LCD.clear();
    LCD.print("claw closing");
    delay(1000);
*/
    //
    //hold claw in closed position
    //RCServo2.detach();
    RCServo0.write(90); //Position of the base starts in the middle
    RCServo1.write(180); //Arm starts up
  }

}