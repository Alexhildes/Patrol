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

//Fuel pumps
int fuelPump = 38;
int auxPump = 36;

//4D Systems Display
Genie genie;    //Initialises Screen Comunication
#define RESETLINE 4

//DS18B20 Sensor variables
int temperature1;
int temperature2;
int temperature3;

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

//Battery Monitor Variables
ResponsiveAnalogRead mainBatteryRead(A5, true);     //Responsive Analogue Read Library reading Analogue 5 Pin (Main Battery)
ResponsiveAnalogRead auxBatteryRead(A4, true);      //Responsive Analogue Read Libary reading Analogue 6 Pin, Wire #2 (Aux Battery)
ResponsiveAnalogRead solarRead(A3, true);           //Responsive Analogue Read Libary reading Analogue 7 Pin, Wire #4 (Solar input)

int mainBatteryValue;                               //Input value from voltage divider,range 0:1023
float mainBatteryVoltage;                          //Stores voltage as float to get more accuracy
float mainBatteryDivider;
int mainResistor1 = 19900;
int mainResistor2 = 10020;

int auxBatteryValue;                               //Input value from voltage divider,range 0:1023
float auxBatteryVoltage;                           //Stores voltage as float to get more accuracy
float auxBatteryDivider;
int auxResistor1 = 20000;
int auxResistor2 = 10030; 

int solarValue;
float solarVoltage;
float solarDivider;
int solarResistor1 = 29820;
int solarResistor2 = 10010;

float errorCorrection = 1.026;
float auxBatteryCurrent;

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


  //PinMode Fuel pumps
  pinMode(fuelPump, OUTPUT);
  pinMode(auxPump, OUTPUT);

  //PinMode Reverse Camera
  pinMode(revCamera, OUTPUT);


  //Temperature Sensors begin
  sensors.begin();

  //Contrast
  contrast = dayContrast;

  //mainBatteryVoltage Divider
  mainBatteryDivider = (float) 10*(mainResistor1 + mainResistor2)/mainResistor1;

  //auxBatteryVoltage Divider
  auxBatteryDivider = (float) 10*(auxResistor1 + auxResistor2)/auxResistor1;

  //solarBatteryVoltage Divider
  //solarDivider = (float) 14.93*(solarResistor1 + solarResistor2)/solarResistor1;
    solarDivider = 20.0;
  

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
    
} 
void loop()
{

  Serial.println("Loop Beginning");

  currentMillis = millis();  //get the current "time" (actually the number of milliseconds since the program started)

  if (currentMillis - startMillis >= period)  //test whether the period has elapsed
  {
      Serial.println("Hello World!");

        //Run the screen display loops
        
        tempSensors();            //Runs Temp Sensor Script
        oilPressure();            //Runs Oil Pressure Script
        batteryVoltage();         //Runs Battery Voltage Script           
        mapSensor();              //Runs Map Sensor Script
        dateTime();               //Runs dateTime() Script
        exhaustTemp();            //Runs exhaust temp script
        
        genie.DoEvents();         //Check for events from Screen
  
        Serial.println("Main Loop");
        startMillis = currentMillis;  //IMPORTANT to save the start time of the current LED state.
    }
}
  
void tempSensors() {
          //Request temperatures form all DS18b20 sensors on the common Bus (BLUE WIRE)
          sensors.requestTemperatures(); 
          //Reading temperature values form sensors
          temperature1 = sensors.getTempCByIndex(0); //Temperature from coolant Sensor (BLUE WIRE)
          //temperature3 = sensors.getTempCByIndex(1); //Temperature form 2nd Sensor on BUS (BLUE WIRE)
  
          temperature2 = 10*temperature1;            //Multiple by 10 for Screen Display
        
          //Writing to touch screen display
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, 3, temperature2/10); //Displays Coolant Temp as Digits (Form 1)
          genie.WriteObject(GENIE_OBJ_GAUGE, 3, temperature2/10);      //Display Coolant Temp on horizontal gauge (Form 0)
}

void oilPressure() {
  //Reading Analogue Pin 1 and converting to PSI
    
    oil1= (25.0*((5.0*(analogRead(A2)/1023.0)) - 0.5)) - 2;         //Oil Pressure sensor in PSI (GREEN WIRE)
  
  
  /*if (oil1 > 5) {

    digitalWrite(fuelPump, HIGH);  //Write high to turn fuel pump on

  } else {

    digitalWrite(fuelPump, LOW);
  }*/

      //Writing to touch screen display
      genie.WriteObject(GENIE_OBJ_LED_DIGITS, 2, oil1);    //Displays Oil Press as Digits (Form 0)
      genie.WriteObject(GENIE_OBJ_GAUGE, 2, oil1);         //Disiplays Oil Press on horizontal gauge (Form 0)
      
}

void batteryVoltage() {
  
  mainBatteryRead.update();                                             //Updates Main Battery Value
  mainBatteryValue = mainBatteryRead.getValue();                       //Reads signal between 0 - 1023 from Analogue Pin 1 (YELLOW WIRE)
  mainBatteryVoltage = 10*mainBatteryValue*(mainBatteryDivider/1023.0); //Converts this to a Voltage between 0 and 150V (For Screen Display)

  //Writing to touch screen display
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 4, mainBatteryVoltage*errorCorrection);    //Displays Voltage as Digits (Form 0, Engine Monitor Screen)
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 10, mainBatteryVoltage*errorCorrection);    //Displays Voltage as Digits (Form 5, Battery Mangement Screen) 
   
  auxBatteryRead.update();                                              //Updates Aux Battery Value
  auxBatteryValue = auxBatteryRead.getValue();                          //Reads signal between 0 - 1023
  auxBatteryVoltage = 10*auxBatteryValue*(auxBatteryDivider/1023.0);    //Converts this to a Voltage between 0 and 150V (For Screen Display)

  //Writing to touch screen display
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 5, auxBatteryVoltage*errorCorrection);    //Displays Voltage as Digits (Form 0, Engine Monitor Screen)
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 11, auxBatteryVoltage*errorCorrection);    //Displays Voltage as Digits (Form 5, Battery Mangement Screen) 
   
  solarRead.update();                                                   //Updates Solar Battery value
  solarValue = solarRead.getValue();                                    //Reads singal between 0 - 1023
  solarVoltage = 10*solarValue*(solarDivider/1023.0);                   //Converts this to a Voltage between 0 and 150V (For Screen Display)

  //Writing to touch screen display
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 13, solarVoltage*errorCorrection);    //Displays Voltage as Digits (Form 5, Battery Management Screen)

}

void batteryCharger() {



  if (digitalRead(chargerLED) == HIGH) {
        
      genie.WriteObject(GENIE_OBJ_USER_LED, 0x00, 1);     //If pin is High - battery is charging
    
  }
      genie.WriteObject(GENIE_OBJ_USER_LED, 0x00, 0);    //If pin is Low - battery is not charging
      
}


void exhaustTemp() {
    //Reading Exhaust Temperature from Digital pins 7,6,and 5
    exhaustTemperature = thermocouple.readCelsius();

    //Writing to touch screen display
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 1, exhaustTemperature);    //Displays EGT as Digits (Form 0)
    genie.WriteObject(GENIE_OBJ_GAUGE, 1, exhaustTemperature);         //Displays EGT on horizontal gauge (Form 0)

}

void mapSensor () {
  //Reading Analogue Pin 1  
        pressureValue = analogRead(A1);               //Reads signal between 0 - 1023 from Analogue Pin 1 (YELLOW WIRE)
       
         boostPress =(((pressureValue/1023.0)+0.04)/0.004) * 0.145; //by 0.145 to calc psi 
         boostPress2 = boostPress - 1;

         if (boostPress2 < 0) {
          boostPress2 = 0;
         } else {
          boostPress2 = boostPress2;
         }
        
          genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0, boostPress2); //Displays MAP on LED Digits (Form 0)
          genie.WriteObject(GENIE_OBJ_GAUGE, 0, boostPress2);      //Displays MAP on horizontal gauge (Form 0)

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
    
// Event Handler for comms from Screen
 void myGenieEventHandler(void)
{
  genieFrame Event;
  genie.DequeueEvent(&Event); // Remove the next queued event from the buffer, and process it below
  
  
  //If the cmd received is from a Reported Event (Events triggered from the Events tab of Workshop4 objects)
  if (Event.reportObject.cmd == GENIE_REPORT_EVENT)
  {
    if (Event.reportObject.object == GENIE_OBJ_WINBUTTON)              // If the Reported Message was from a WIN Button
    {
      if (Event.reportObject.index == 0)                              // If Button (Index = 0), Tank is full
      {
        genie.WriteObject(GENIE_OBJ_LED_DIGITS, 9, 120);              //Write to digits saying Tank is full
      }
      if (Event.reportObject.index == 1)                              // If Button (Index = 1), 20 Litres option
      { 
          if (auxTankEna20 == false)
        {
          digitalWrite(auxPump, HIGH);                                //Turn pump on
          auxTankEna20 = true;

          genie.WriteObject(GENIE_OBJ_USER_LED, 2, 1);                //Turn LED on
          startMillis = millis();                                     //Start the timer
          
        } else if (auxTankEna20 == true)
          {
            digitalWrite(auxPump, LOW);                               //Turn pump off
            auxTankEna20 = false;                                     //Start the timer

            genie.WriteObject(GENIE_OBJ_USER_LED, 2, 0);              //Turn LED off
          
        }
      }   
      if (Event.reportObject.index == 2)                              // If Button (Index = 2), 40 Litres option
      {
          if (auxTankEna40 == false)
        {
          digitalWrite(auxPump, HIGH);                                //Turn pump on
          auxTankEna40 = true;

          genie.WriteObject(GENIE_OBJ_USER_LED, 2, 1);                //Turn LED on
          
        } else if (auxTankEna40 == true)
          {
            digitalWrite(auxPump, LOW);                               //Turn pump off
            auxTankEna40 = false;

            genie.WriteObject(GENIE_OBJ_USER_LED, 2, 0);              //Turn LED off
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

}


