#include <LiquidCrystal.h>
//#include <Claw.h>
#include <phys253.h>

//Claw claw(0);
int posClaw = 0;

void setup() {
    #include <phys253setup.txt>
    ServoTimer2 RCServo0();
    //RCServo0.attach(0);
    pinMode(0, OUTPUT);
    Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  //claw.open();
  //delay(100);
  //claw.close();
  //delay(100);
    // put your main code here, to run repeatedly:

  //Close claw around passenger
  for(posClaw = 0; posClaw <= 180; posClaw++){
    RCServo0.write(posClaw);
    LCD.clear();
    LCD.print(posClaw);
    delay(15);
  }
  
  delay(1000);
  
  for(posClaw = 180; posClaw >= 0; posClaw--){
    RCServo0.write(posClaw);
    LCD.clear();
    LCD.print(posClaw);
    delay(15);
  }
  
  delay(1000);
  //hold claw in closed position
  //RCServo0.detach();
}

OR ----------------------------------------

#include <LiquidCrystal.h>
//#include <Claw.h>
#include <phys253.h>

//Claw claw(0);
int posClaw = 0;

void setup() {
    #include <phys253setup.txt>
    ServoTimer2 RCServo0();
    //RCServo0.attach(0);
    pinMode(0, OUTPUT);
    Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  //claw.open();
  //delay(100);
  //claw.close();
  //delay(100);
    // put your main code here, to run repeatedly:

  //Close claw around passenger
    RCServo0.write(180);
    LCD.clear();
    LCD.print(180);
    delay(1000);
  
    RCServo0.write(0);
    LCD.clear();
    LCD.print(posClaw);
    delay(1000);
  
//    delay(1000);
  //hold claw in closed position
  //RCServo0.detach();
}