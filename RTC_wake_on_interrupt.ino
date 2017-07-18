//taken from example in RTClibExtended library example
#include <Wire.h>
#include <RTClibExtended.h>
#include "LowPower.h"//avr only

#define interruptPin 2    //use interrupt 0 (pin 2) and run function wakeUp when pin 2 gets LOW
#define ledPin 13    //use arduino on-board led for indicating sleep or wakeup status
RTC_DS3231 RTC;      //we are using the DS3231 RTC

volatile uint16_t interuptCount = 0;
volatile bool interuptFlag = false;
byte i;
byte AlarmFlag = 0;
byte ledStatus = 1;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
char dateBuffer[12];
//-------------------------------------------------

void an_isr()        // here the interrupt is handled after wakeup
{
    interuptCount++;
    interuptFlag = true;  
}

//------------------------------------------------------------
void clear_alarms(){
  RTC.armAlarm(1, false);
  RTC.clearAlarm(1);
  RTC.alarmInterrupt(1, false);
  RTC.armAlarm(2, false);
  RTC.clearAlarm(2);
  RTC.alarmInterrupt(2, false);
}

void setup() {
  Serial.begin(9600);
  Serial.println("Starting wakeup alarm/timer sketch");
  //Set pin D2 as INPUT for accepting the interrupt signal from DS3231
  pinMode(interruptPin, INPUT_PULLUP);
  //switch-on the on-board led for 1 second for indicating that the sketch is ok and running
  pinMode(ledPin, OUTPUT);
  //Initialize communication with the clock
  Wire.begin();
  RTC.begin();
  //clear any pending alarms
  clear_alarms();
  
  //Set SQW pin to OFF (in my case it was set by default to 1Hz)
  //The output of the DS3231 INT pin is connected to this pin
  //It must be connected to arduino D2 pin for wake-up
  RTC.writeSqwPinMode(DS3231_OFF);

  //Set alarm2 every minute
  RTC.setAlarm( ALM1_MATCH_SECONDS, 0, 0, 0, 0 ); // alarm1 flag will be set every minute

  RTC.alarmInterrupt(1, true); //enable alarm 2
  attachInterrupt(digitalPinToInterrupt(interruptPin), an_isr, FALLING);//attach isr to alarm interupt
}

//------------------------------------------------------------

void loop() 
{
  //attach the interrupt conncted to the rtc
    attachInterrupt(digitalPinToInterrupt(interruptPin), an_isr, FALLING);
    //go to sleep
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); 
    //resumes program here
    detachInterrupt(interruptPin); //detach the interupt
  
    DateTime now = RTC.now(); //get the current time 
    clear_alarms();//clear previous RTC alarms
    //blink the LED to show that we have woken up
    for (i = 0; i < 11; i++){      
      digitalWrite(ledPin, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(100);                   // wait for a while
      digitalWrite(ledPin, LOW);    // turn the LED off by making the voltage LOW
      delay(100);                   // wait for a while
    }
    //setup the alarms again
    RTC.setAlarm( ALM1_MATCH_SECONDS, 0, 0, 0, 0 ); // alarm1 flag will be set every minute
    RTC.alarmInterrupt(1, true); //enable alarm 2
    //print the time that when we have woken up
    Serial.print("Woken up from sleep at ");    
    sprintf(dateBuffer,"%02u-%02u-%04u ",now.month(),now.day(),now.year());
    Serial.print(dateBuffer);
    sprintf(dateBuffer,"%02u:%02u:%02u ",now.hour(),now.minute(),now.second());
    Serial.println(dateBuffer);
    //Print that we are going back to sleep in the following loop
    Serial.println("Going back to sleep...");
    delay(100);
  
}
