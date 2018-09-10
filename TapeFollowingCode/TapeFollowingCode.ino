#include <phys253.h>          
#include <LiquidCrystal.h>     


  ////////////////////////////////////////////////
// TINAH Demo Program - UBC Engineering Physics 253
// (nakane, 2015 May 7) -  Update for IDE 1.6.4   
// (nakane, 2015 Jan 2)  - Updated for use in Arduino IDE (1.0.6)
/////////////////////////////////////////////////

void setup()
{  
  #include <phys253setup.txt>
  Serial.begin(9600) ;
  LCD.clear();  LCD.home();
  
}

/////////////////////////////////////////////////
// DIGITAL INPUT TEST
/////////////////////////////////////////////////
void loop()
{
  int pinNumber = 0; 
  //int variable;

   
// 16 positions on LCD screen:     1234567890123456
  //  variable = portRead(0) ;
    LCD.setCursor(0,0);    
      /*variable = digitalRead(pinNumber) ;
      if (variable == HIGH)
         {LCD.print("Open  ") ;} 
      else
          {LCD.print("Closed") ;}*/
    int count = 0;
    unsigned long t1 = millis();
    bool tempState = digitalRead(pinNumber);
    double frequency;
    unsigned long t2;
    int i;

    for(i=0 ; i<=10000 ; i++){
      if(tempState != digitalRead(pinNumber)){
        tempState = !tempState;
        count++;
      }
    }

    t2 = millis();
    frequency = (double)count/(t2-t1)*500;
    LCD.clear();
    LCD.print(frequency);
    Serial.print(frequency);
    
     
    delay(500) ;
}


