/*
Testing of the arm and claw, with integration of IR detection
*/

#include <LiquidCrystal.h>
#include <phys253.h>

//Pin allocations
int irPin_right = 1; //ir pin - analog input
int irPin_left = 3; //ir pin - analog input
int irPin_arm = 7; //ir pin - analog input
int detectionPin_passenger1 = 4; //microswitch pin - digital input 
int detectionPin_passenger2 = 6; //microswitch pin - digital input 

//IR detection on left and right of vehicle
double leftIRSignal = 0;
double rightIRSignal = 0;
int pickup_left = 0;
int pickup_right = 0;
int dropoff_left = 0;
int dropoff_right = 0;

//IR detection on arm
double currentIRSignal_arm = 0;
double strongestIRSignal_arm = 0;
double locationStrongestIRSignal_arm = 0;

//checkpoint tokens
int nextStep_armLower = 0; //aimed at passenger, adjust arm height
int nextStep_clawClose = 0; //arm extended, close claw
int nextStep_armLift = 0; //claw closed, lift arm back up
int nextStep_dropOff = 0; //arm lifted, determine drop off side
int nextStep_clawOpen = 0; //arm lifted, open claw
int reset_arm = 0; //bring back arm and claw to initial positions

int pos = 0; //variable servo angle
int counter = 0; //counter for microswitch sensing
double thresholdIRSignal = 1.0;

void setup() {
#include <phys253setup.txt>

  ServoTimer2 RCServo0; //Controls the rotation and position of the base
  ServoTimer2 RCServo1; //Controls ascending and descending of arm
  ServoTimer2 RCServo2; //Controls claw actuation

  //Initial positions
  RCServo0.write(90); //Position of the base starts in the middle
  RCServo1.write(0); //Arm starts up
  RCServo2.write(50); //Claw starts open

  //Set pin mode for microswitch passenger detection
  pinMode(detectionPin_passenger1, INPUT);
  digitalWrite(detectionPin_passenger1, LOW);
  pinMode(detectionPin_passenger2, INPUT);
  digitalWrite(detectionPin_passenger2, LOW);

  Serial.begin(9600);
}
void loop() {


//Test 1: Testing left and right IR signal detection
//Detects if IR signal is coming from right or left side (adjust/test threshold values)
//----------------Test 1 start-----------------
  LCD.clear();
  LCD.setCursor(0, 0);
  LCD.print("Left:");
  LCD.setCursor(0, 1);
  LCD.print("Right:");
  leftIRSignal = analogRead(irPin_left) * 5.0 / 1024.0;
  LCD.setCursor(8, 0);
  LCD.print(leftIRSignal);
  delay(1000);
  rightIRSignal = analogRead(irPin_right) * 5.0 / 1024.0;
  LCD.setCursor(8, 1);
  LCD.print(rightIRSignal);
  delay(1000);

  if(analogRead(irPin_left)*5.0/1024.0 > thresholdIRSignal){
    LCD.clear();
    LCD.print("left works");
  }

  if(analogRead(irPin_right)*5.0/1024.0 > thresholdIRSignal){
    LCD.clear();
    LCD.print("right works");
  }
//----------------Test 1 end-----------------


//Test 2: Testing microswitch detection
//If both microswitches LOW - display "Low"; Of both microswiched HIGH - display "high"
//----------------Test 2 start-----------------
  //if low, print "Low" on the LCD
  if(digitalRead(detectionPin_passenger1) == HIGH && digitalRead(detectionPin_passenger2) == HIGH){
    LCD.clear();
    LCD.print("Low");
    delay(1000);
  }
  //if high, print "High" on the LCD
  if(digitalRead(detectionPin_passenger1) == LOW || digitalRead(detectionPin_passenger2) == LOW){
    LCD.clear();
    LCD.print("High");
    delay(1000);
  }
//----------------Test 2 end-----------------



//Test 3: Testing arm IR detection
//Sweep for highest IR signal and return to location of highest signal detected 
//----------------Test 3 start-----------------
  strongestIRSignal_arm = analogRead(irPin_arm) * 5.0 / 1024.0;
  LCD.clear();
  LCD.print(strongestIRSignal_arm);
  delay(1000);

  RCServo0.write(0);
    LCD.clear();
    LCD.print("Starting sweep");
    delay(1000);
    for (pos = 0; pos <= 50; pos++) {
      RCServo0.write(pos);
      LCD.clear();
      LCD.setCursor(0, 0);
      LCD.print("Pos:");
      LCD.setCursor(5, 0);
      LCD.print(pos);
      currentIRSignal_arm = analogRead(irPin_arm) * 5.0 / 1024.0;
      if (currentIRSignal_arm > strongestIRSignal_arm) {
        strongestIRSignal_arm = currentIRSignal_arm;
        locationStrongestIRSignal_arm = pos;
        //LCD.setCursor(0, 1);\
        
        //LCD.print("L:");
        LCD.setCursor(3, 1);
        LCD.print(locationStrongestIRSignal_arm);
        delay(500);
        //LCD.setCursor(6, 1);
        //LCD.print("V:");
        LCD.setCursor(9, 1);
        LCD.print(strongestIRSignal_arm);
        delay(500);
      }
      delay(200);
    }
    LCD.clear();
    LCD.print("Loc:");
    delay(2000);
    LCD.setCursor(5, 0);
    LCD.print(locationStrongestIRSignal_arm);
    RCServo0.write(locationStrongestIRSignal_arm);
//----------------Test 3 end-----------------    


//Test 4: Testing arm readjustment
//Three tries - Try centered; try a bit to the left; try a bit to the right
//----------------Test 4 start-----------------  
    pos = 30;
    RCServo0.write(pos);
    RCServo1.write(45);
    LCD.clear();
    LCD.print("Claw closing");
    do{
      RCServo2.write(115);
      //if microswitch detects passenger, proceed to lifting arm up
      if(digitalRead(detectionPin_passenger1) == HIGH || digitalRead(detectionPin_passenger2) == HIGH){
        nextStep_armLift = 1;
        break;
      }
      counter = counter + 1;
    }while(counter < 10);
    delay(2000);

    //set counter back to 0
    counter = 0;

    //if microswitch does not detect passenger, readjust arm position
    if(nextStep_armLift == 0){
      RCServo1.write(0); //lift arm
      RCServo2.write(50); //open claw
      
      //try readjusting left first
      LCD.clear();
      LCD.print("Retrying left");
      RCServo0.write(pos - 25); //rotate base a bit left
      RCServo1.write(45); //lower arm
      
      do{
        RCServo2.write(115);
        //if microswitch detects passenger, proceed to lifting arm up
        if(digitalRead(detectionPin_passenger1) == HIGH || digitalRead(detectionPin_passenger2) == HIGH){
          nextStep_armLift = 1;
          break;
        }
        counter = counter + 1;
      }while(counter < 10);
      delay(2000);

      //set counter back to 0
      counter = 0;

      if(nextStep_armLift == 0){
        RCServo1.write(0); //lift arm
        RCServo2.write(50); //open claw
        
        //final readjustment right
        LCD.clear();
        LCD.print("Retrying right");
        RCServo0.write(pos + 25); //rotate base a bit to the right
        RCServo1.write(45); //lower arm
        
        do{
          RCServo2.write(115);
          //if microswitch detects passenger, proceed to lifting arm up
          if(digitalRead(detectionPin_passenger1) == HIGH || digitalRead(detectionPin_passenger2) == HIGH){
            nextStep_armLift = 1;
            break;
          }
          counter = counter + 1;
        }while(counter < 10);
      
        //set counter back to 0
        counter = 0;
      }
    }
    LCD.clear();
    LCD.print("Done 3 tries");
//----------------Test 4 end-----------------  

//Test 5:Left IR and right IR test
//----------------Test 5 start----------------- 

  LCD.clear();
  LCD.setCursor(0, 0);
  LCD.print("Left:");
  LCD.setCursor(0, 1);
  LCD.print("Right:");
  //read in IR detection from side detectors 
  leftIRSignal = analogRead(irPin_left) * 5.0 / 1024.0;
  LCD.setCursor(8, 0);
  LCD.print(leftIRSignal);
  delay(200);
  rightIRSignal = analogRead(irPin_right) * 5.0 / 1024.0;
  LCD.setCursor(8, 1);
  LCD.print(rightIRSignal);
  delay(200);
  
  //TODO: at a certain strength, the car will stop...

  //Passenger on left side
  if(leftIRSignal > thresholdIRSignal){
    pickup_left = 1;
  }

  //Passenger on right side
  if(rightIRSignal > thresholdIRSignal){
    pickup_right = 1;
  }

//----------------Test 5 end-----------------   
  
}

