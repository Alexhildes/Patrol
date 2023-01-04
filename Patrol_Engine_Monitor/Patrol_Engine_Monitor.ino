 //Author: Alex Hildebrand   GQ Patrol Engine and Turbo Monitoring  March 2020

#include <genieArduino.h> //Touch Screen Library Interface
#include <max6675.h>  //Temperature Sensor EGT
#include <OneWire.h> //Temperature Sensors Coolant, External
#include <DallasTemperature.h> //As Above
#include <ResponsiveAnalogRead.h>
#include <Wire.h>     //RTC Library
#include "RTClib.h"   //RTC Library

//Real time clock
RTC_DS1307 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
int datemonth;
int dateday;
int monthday;

int dateminute;
int datehour;
int hourminute;

//Global
boolean engineOn;

//Fuel pump
boolean pumpAuto;
boolean pumpOn;
boolean pumpOff;

int autoSwitch;
int onoffSwitch;

int fuelPump = 10;
int auxPump = 36;

//4D Systems Display
Genie genie;    //Initialises Screen Comunication
#define RESETLINE 4

//DS18B20 Sensor variables
int engTemp;
int ambTemp;

//MAX6675 variable
int exhaustTemperature;

//Oil Pressure Variables
float oil1;
int oil2;

//Thermocouple Variables
int thermoDO = 13; //Brown Wire
int thermoCS = 12; //White Wire
int thermoCLK = 11;//Orange Wire
MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

//DS18B20 Vairables
#define ONE_WIRE_BUS 8                // Data wire is plugged into digital port 8 on the Arduino (Blue Wire)
OneWire oneWire(ONE_WIRE_BUS);          // Setup a oneWire instance to communicate with any OneWire devices 
DallasTemperature sensors(&oneWire);    // Pass our oneWire reference to Dallas Temperature.


//MAP Sensor Variables
int pressureValue;  //Input value from pressure sensor, range 0:1023
float boostVoltage; //Variable for storing Vout from MAP sensor
float boostPress;   //Pressure value in KPA
int boostPress2;   //Pressure value in Psi

//Test for turbo timer
int led1 = 22;

//Contrast
int contrast;
int dayContrast = 15;
int nightContrast = 5;

//DC-DC Charger
int chargerLED = 6;    //DC-DC Charger Status to Digital 6
boolean chargerStatus;

//Spare
int spareInput = 7;    //Spare input to Digital 7

//Fuel Tank
bool auxTankEna20 = false;     //Aux tank enable, 20 litres

boolean auxTankEna40 = false;     //Aux tank enable, 40 litres

int fuelRate = 2;         //Fuel rate is 2 litres per minute

//Reverse Camera relay
int revCamera = 34;

//Millis
unsigned long startMillis;  //some global variables available anywhere in the program
unsigned long currentMillis;
const unsigned long period = 350;  //the value is a number of milliseconds

void setup() 
{

  
  startMillis = millis();  //initial start time

  //Serial configurations
  Serial.begin(9600);          //Serial1 @ 9600 for Serial Monitor
  Serial2.begin(200000);       //Serial2 @ 200000 for Genie
  genie.Begin(Serial2);        //Begins communications with Genie

  genie.AttachEventHandler(myGenieEventHandler); // Attach the user function Event Handler for processing events

  Serial.println("Beginning Setup");
  
  //PinMode Fuel pumps
  pinMode(fuelPump, OUTPUT);
  pinMode(auxPump, OUTPUT);


  //Temperature Sensors begin
  sensors.begin();

  //Contrast
  contrast = dayContrast;


  //Begin Real Time Clock
  rtc.begin();

  //Turn Rear Camera on
  digitalWrite(revCamera, HIGH);  //Write high to turn fuel pump on


  //Screen Reset
  pinMode(RESETLINE, OUTPUT);  // Set D4 on Arduino to Output (4D Arduino Adaptor V2 - Display Reset)
  digitalWrite(RESETLINE, 1);  // Reset the Display via D4
  delay(2000);
  digitalWrite(RESETLINE, 0);  // unReset the Display via D4
  delay(5000); //let the display start up after the reset (This is important)

  //Fuel pump Mode Auto by default
  digitalWrite(fuelPump, LOW);
  genie.WriteObject(GENIE_OBJ_4DBUTTON,0,1);
  pumpAuto = true;

  Serial.println("End Setup");
    
} 
void loop()
{


  currentMillis = millis();  //get the current "time" (actually the number of milliseconds since the program started)

  if (currentMillis - startMillis >= period)  //test whether the period has elapsed
  {


        //Run the screen display loops
        Serial.println("Beginning Loop");
        
        tempSensors();            //Runs Temp Sensor Script 
        oilPressure();            //Runs Oil Pressure Script          
        mapSensor();              //Runs Map Sensor Script
        dateTime();               //Runs dateTime() Script
        exhaustTemp();            //Runs exhaust temp script
        liftPump();
        
        genie.DoEvents();         //Check for events from Screen
  
        Serial.println("Main Loop End");
        startMillis = currentMillis;  //IMPORTANT to save the start time of the current LED state.
    }
}
  
void tempSensors() {
          //Request temperatures form all DS18b20 sensors on the common Bus (BLUE WIRE)
          sensors.requestTemperatures(); 
          //Reading temperature values form sensors
          engTemp = sensors.getTempCByIndex(0); //Temperature from coolant Sensor (BLUE WIRE)
          ambTemp = sensors.getTempCByIndex(1); //Temperature form 2nd Sensor on BUS (BLUE WIRE)
          
          //Writing to touch screen display
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, 3, engTemp); //Displays Coolant Temp as Digits (Form 0)
          genie.WriteObject(GENIE_OBJ_GAUGE, 3, engTemp);      //Display Coolant Temp on horizontal gauge (Form 0)
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, 6, engTemp);      //Display Ambient Temp on on Top (Form 0)
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, 5, ambTemp);      //Display Ambient Temp on on Top (Form 0)
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, 12, ambTemp);      //Display Ambient Temp on on Top (Form 3)

          Serial.println("Temp Sensors Complete");
}

void oilPressure() {
  //Reading Analogue Pin 1 and converting to PSI
  //Linear from 0.5 Volts,   
    oil1= (25.0*((5.0*(analogRead(A2)/1023.0)) - 0.5)) - 2;         //Oil Pressure sensor in PSI (GREEN WIRE)
  
    if (oil1 > 5) {

      engineOn = true;
      
    } else {

      engineOn = false;
    }

      //Writing to touch screen display
      genie.WriteObject(GENIE_OBJ_LED_DIGITS, 2, oil1);    //Displays Oil Press as Digits (Form 0)
      genie.WriteObject(GENIE_OBJ_GAUGE, 2, oil1);         //Disiplays Oil Press on horizontal gauge (Form 0)
      genie.WriteObject(GENIE_OBJ_LED_DIGITS, 11, oil1);         //Disiplays Oil Press on horizontal gauge (Form 3)

      Serial.println("Oil Pressure Complete");
}


void exhaustTemp() {
    //Reading Exhaust Temperature from Digital pins 7,6,and 5
    exhaustTemperature = thermocouple.readCelsius();

    //Writing to touch screen display
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 1, exhaustTemperature);    //Displays EGT as Digits (Form 0)
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 9, exhaustTemperature);    //Displays EGT as Digits (Form 0)
    genie.WriteObject(GENIE_OBJ_GAUGE, 1, exhaustTemperature);         //Displays EGT on horizontal gauge (Form 0)

    Serial.println("Oil Pressure Complete");

}

void mapSensor () {
  //Reading Analogue Pin 1
  //Sensor Model MPX4250AP  

  
        pressureValue = analogRead(A1);               //Reads signal between 0 - 1023 from Analogue Pin 1 (YELLOW WIRE)
       
         boostPress =(((pressureValue/1023.0)+0.04)/0.004) * 0.145; //by 0.145 to calc psi 
         boostPress2 = boostPress - 1;

         if (boostPress2 < 0) {
          boostPress2 = 0;
         } else {
          boostPress2 = boostPress2;
         }
        
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0, boostPress2); //Displays MAP on LED Digits (Form 0)
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, 10, boostPress2); //Displays MAP on LED Digits (Form 0)
          genie.WriteObject(GENIE_OBJ_GAUGE, 0, boostPress2);      //Displays MAP on horizontal gauge (Form 0)

          Serial.println("Map complete");

}

/*void auxfuelTanks() {

    currentMillis = millis();                                                                   //Get the current time

          if (auxTankEna20 == true && (currentMillis - startMillis < 600000)) {                 //Is the tank on and has enough time passed?
              return;                                                                           //Keep on if true
          } else if (auxTankEna20 == true && (currentMillis - startMillis > 600000)) {          //Is the tank on and has enough time passed?{
              auxTankEna20 = false;
              digitalWrite(auxPump, LOW);
              genie.WriteObject(GENIE_OBJ_USER_LED, 2, 0);                                      //Turn off once the time has passed
          }
                              
          if (auxTankEna40 == true && (currentMillis < startMillis + 120000)) {                 //Is the tank on and has enough time passed?
              return;                                                                           //Keep on if true
          } else if (auxTankEna40 == true && (currentMillis > startMillis + 120000)) {          //Is the tank on and has enough time passed?{
              auxTankEna40 = false;
              digitalWrite(auxPump, LOW);                                                       //Turn off once the time has passed
              genie.WriteObject(GENIE_OBJ_USER_LED, 2, 0); 
          }

}
*/

void liftPump () {

    if (pumpOn == true)
    {
      digitalWrite(fuelPump, HIGH);
      Serial.println("Pump on");
      return;
    }

    if (engineOn == true && pumpAuto == true)
    {
      digitalWrite(fuelPump, HIGH);
      Serial.println("Pump on");
    }

    if (engineOn == true && pumpAuto == false)
    {
      digitalWrite(fuelPump, LOW);
      Serial.println("Pump off");
    }
    

    if (pumpOff == true && pumpAuto == false)
    {
      digitalWrite(fuelPump, LOW);
      Serial.println("Pump off");
    }
    
}
    
// Event Handler for comms from Screen
 void myGenieEventHandler(void)
{
  genieFrame Event;
  genie.DequeueEvent(&Event); // Remove the next queued event from the buffer, and process it below
  
  
  //If the cmd received is from a Reported Event (Events triggered from the Events tab of Workshop4 objects)
  if (Event.reportObject.cmd == GENIE_REPORT_EVENT)
  {
    if (Event.reportObject.object == GENIE_OBJ_4DBUTTON)              // If the Reported Message was from a WIN Button
    {
      if (Event.reportObject.index == 0)                              // If Button (Index = 0), Tank is full
      {
        //Auto Rocker ON will run on oil pressure only
        //If other switch goes to "ON", turn off AUTO and remain ON
        //If other switch goes to "OFF", turn off pump
        
        autoSwitch = genie.GetEventData(&Event);
        Serial.println(autoSwitch);

        if (autoSwitch == 0)
        {
          pumpAuto = false;
        } else if (autoSwitch == 1)
        {
          pumpAuto = true;
          genie.WriteObject(GENIE_OBJ_4DBUTTON,1,0);
        }
        
      
      }
      if (Event.reportObject.index == 1)            
      { 
         onoffSwitch = genie.GetEventData(&Event);
         if (onoffSwitch == 0)
         {
           pumpOff = true;
           pumpOn = false;
           pumpAuto = false;
           
         } else if (onoffSwitch == 1)
         {
           pumpOn = true;
           pumpAuto = false;
           genie.WriteObject(GENIE_OBJ_4DBUTTON,0,0);
         }
         
         
      }   
      
  } 
  
    
   if (Event.reportObject.object == GENIE_OBJ_USERBUTTON)
    {
      if (Event.reportObject.index == 2)
        {
        if  (contrast == dayContrast) 
          {
            genie.WriteContrast(nightContrast);
            contrast = nightContrast;

          } else if (contrast == nightContrast) 
            {
              genie.WriteContrast(dayContrast);
              contrast = dayContrast;
          
            }
          
        }

    }

  }

} 

void dateTime () {

    DateTime now = rtc.now();

    datemonth = now.month();
    dateday = now.day();
    datehour = now.hour();
    dateminute = now.minute();

    monthday = dateday*100 + datemonth;
    hourminute = datehour*100 + dateminute;

    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 7, monthday);
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 8, hourminute);
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 4, hourminute);
}
