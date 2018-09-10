#include <LiquidCrystal.h>
#include <phys253.h>

//Pin allocations
int irPin_right = 1; //ir pin - analog input
int irPin_left = 3; //ir pin - analog input
int detectionPin_passenger1 = 4; //microswitch pin - digital input 
int detectionPin_passenger2 = 6; //microswitch pin - digital input

//IR detection on left and right of vehicle
double leftIRSignal = 0;
double rightIRSignal = 0;
int pickup_left = 0;
int pickup_right = 0;
int dropoff_left = 0;
int dropoff_right = 0;

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
  
  //Base rotation - Find the direction of the strongest IR signal
  
  //Left side
  if (pickup_left == 1) {
    RCServo0.write(0);
    LCD.clear();
    LCD.print("Left side");
    delay(1000);
    nextStep_armLower = 1;
    pickup_left = 0;
  }
  

  //Right side
  if (pickup_right == 1) {
    RCServo0.write(180);
    LCD.clear();
    LCD.print("Right side");
    delay(1000);
    nextStep_armLower = 1;
    pickup_right = 0;
  }

  //Arm height - lower arm to height of passenger
  if (nextStep_armLower == 1) {
    LCD.clear();
    LCD.print("Lowering arm");
    delay(1000);
    RCServo1.write(45);
    nextStep_clawClose = 1;
    nextStep_armLower = 0 ;
  }

//microswitch controlling when claw closes
  while(nextStep_clawClose == 1){
    LCD.clear();
    LCD.print("Claw closing");
    do{
      RCServo2.write(115);
      //if microswitch detects passenger, proceed to lifting arm up
      if(digitalRead(detectionPin_passenger1) == LOW || digitalRead(detectionPin_passenger2) == LOW){
        nextStep_armLift = 1;
        nextStep_clawClose = 0;
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
        if(digitalRead(detectionPin_passenger1) == LOW || digitalRead(detectionPin_passenger2) == LOW){
          nextStep_armLift = 1;
          nextStep_clawClose = 0;
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
          if(digitalRead(detectionPin_passenger1) == LOW || digitalRead(detectionPin_passenger2) == LOW){
            nextStep_armLift = 1;
            nextStep_clawClose = 0;
            break;
          }
          counter = counter + 1;
        }while(counter < 10);
      
        //set counter back to 0
        counter = 0;
      }
    }
  }

//if no passenger detected by microswitch after 3 tries, reset arm and keep looking
  if(nextStep_armLift == 0){
    LCD.clear();
    LCD.print("Restart");
    LCD.setCursor(0, 1);
    LCD.print("No passenger");
    reset_arm = 1;
    nextStep_clawClose = 0;
  }

//TODO: continue with path following

//when passenger obtained, lift arm back up
  if(nextStep_armLift == 1){
    RCServo1.write(0);
    LCD.clear();
    LCD.print("Lifting arm");
    delay(1000);
    nextStep_armLift = 0;
    nextStep_dropOff = 1;
  }

//TODO - Determine direction... where does arm turn to/go to?
//Jacob's node code?
/*
if(LEFT){
  dropoff_left = 1;
}
if(RIGHT){
  dropoff_right = 1;
}
*/

//when at destination, turn and lower arm
if(nextStep_dropOff == 1){
  //drop off is on the left side of vehicle
  LCD.clear();
  LCD.print("Drop off:");
  
  if(dropoff_left == 1){
    LCD.setCursor(0, 1);
    LCD.print("Left");
    delay(1000);
    RCServo0.write(0); //rotate arm to left side
    RCServo1.write(45); //lower arm
    nextStep_dropOff = 0;
    dropoff_left = 0;
    nextStep_clawOpen = 1;
  }

  //drop off is on the right side of vehicle
  if(dropoff_right == 1){
    LCD.setCursor(0, 1);
    LCD.print("Right");
    delay(1000);
    RCServo0.write(180); //rotate arm to right side
    RCServo1.write(45); //lower arm
    nextStep_dropOff = 0;
    dropoff_right = 0;
    nextStep_clawOpen = 1;
  }
}

//When facing correct direction, open claw and drop off passenger
  if(nextStep_clawOpen == 1){
    LCD.clear();
    LCD.print("Claw opening");
    RCServo2.write(50);
    delay(1000);
    nextStep_clawOpen = 0;
  }

  reset_arm = 1; //if end of loop, reset and start again 
  
  if(reset_arm == 1){
    //rest arm, claw and base to initial positions
    LCD.clear();
    LCD.print("Restart");
    LCD.setCursor(0, 1);
    LCD.print("New loop");
    RCServo0.write(90); //Position of the base starts in the middle
    RCServo1.write(0); //Arm starts up
    RCServo2.write(50); //Claw starts open
    reset_arm = 0;
  }
}