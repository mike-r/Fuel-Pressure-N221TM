#include <SoftwareSerial.h>

/*
 * 
 *                      N873PW Arduino
 * 
 *          **********  Version 3.0 **********
 *          **** From N221TM Version 2.6 *****
 *          
 Reads an analog input pin, maps the result to a range from 0 to 255
 and uses the result to set the pulsewidth modulation (PWM) of an output pin.
 Also prints the results to the serial monitor.

Fuel Pressure circuit:
   Garmin Fuel Pressure sensor connected to analog pin 0.
   Sensor ouputs 0.5 volts at 0.0 PSIG and 4.5 volts at 15.0 PSIG.
   Output is divided using 2K fixed resistor and a 200K 20-turn pot.
  
Smoke Tank Level circuit:
   eTape Smoke Oil Tank Level sensor connected to analog pin A2.


   
 
 Original AI and maping created 29 Dec. 2008
 modified 9 Apr 2012 by Tom Igoe
 Addapted Jan 2016 by Mike Rehberg for N221TM Fuel Pressure transducer
 Moved to N873PW in 2021 by Mike Rehberg 
                                        ouputs 0.5Vdc to 4.5Vdc while VM1000 is expecting 0 to 50mv.
                                        

                                        Added a timer on the analog read/write of the fuel pressure sensor.
                                        The VM1000 is slow anyway so no need to waste Arduino cycles.  Currently
                                        set to update once every 2.925 seconds.

 V 2.6  Feb 24, 2017 by Mike Rehberg:   Added Smoke oil tank level sensor to Analog Input pin A2.
                                        Level is read by eTape sensor which outputs 0-5 volts based on 
                                        0 to 12" of level in tank.  (0.404 Volts/Inch
                                        *****  NEEDS TO BE TESTED AND CALIBRATED *******
 

 Things to add:                         + Hardware switch and code to enable debug to LCD display
 
 */

// These constants won't change.  They're used to give names
// to the pins used:

const int analogInFuel   = A0; // Analog input pin that the Fuel Sensor is attached to
//                         A1  // Analog input pin used for LCD Serial Display output
const int analogInSmoke  = A2; // Analog input pin for Smoke Oil Tank level sensor
const int analogOutFuel  = 3;  // Analog output pin that is voltage divided before going to
float psig;                    // The VM-1000
float inputVoltage;
float temp;
float resolution=210.928;   // Fuel Pressure Input steps per Volt  1023/4.85
float scaleFactor=17.0;     // Fuel Pressure Output steps per PSIG: 255/15
float smokeGallons;
float smokeResolution = 210.928;   // Smoke Level Input steps per Volt 1023 / 4.85  (9" = Full = 5 Gallons)
float smokeScaleFactor = 1.375;    // Smoke Level Gallons per Volt (.404V/in, 3.6375V=Full, .7275V/Gallon)
float smokeVolts;
float smokeWeight;          // How much somke juice onboard in lbs.
int   garminOffset=102;     // Steps for 0.5 VDC offset to 0.0 PSIG
unsigned long currentMillis =  0;   // How long the Arduino has been running (will loo in 50 days)
unsigned long previousMillis = 0;   // The last time the Fuel Pressure was updated
int           interval = 2925;      // Loop time for reading and writing the Fuel Pressure and reading Smoke Level 

#define rxPinLCD 2    //  rxPinLCD is immaterial - not used - just make this an unused Arduino pin number
#define txPinLCD 15   //  txpinLCD for Serial 4x20 LCD Display using Analog(1)

#define rxPinAP1 5    //  rxPinAP1 is immaterial - not used - just make this an unused Arduino pin number
#define txPinAP1 4    //  txpinAP1 for output to SmartGPS using Digital(4)


SoftwareSerial lcdSerial  =  SoftwareSerial(rxPinLCD, txPinLCD);
SoftwareSerial ap1Serial  =  SoftwareSerial(rxPinAP1, txPinAP1);

int rawSensorValue = 0;     // value read from the Sensor
int sensorValue;
int outputValue = 0;        // value output to the PWM (analog out)

void setup() {
  pinMode(analogOutFuel, OUTPUT);

  pinMode(txPinLCD, OUTPUT); // Serial output to 4/20 character LCD display
  lcdSerial.begin(9600);      // 9600 baud is chip comm speed
  
  lcdSerial.print("?G420");   // set display geometry,  4 x 20 characters in this case
//  lcdSerial.print("?G216");   // set display geometry, 2 x 16 characters in this case
  delay(500);                // pause to allow LCD EEPROM to program
  lcdSerial.print("?Bff");    // set backlight to ff hex, maximum brightness
  delay(1000);               // pause to allow LCD EEPROM to program
  lcdSerial.print("?s6");     // set tabs to six spaces
  delay(1000);               // pause to allow LCD EEPROM to program
  lcdSerial.print("?c0");     // turn cursor off
  delay(300);
  lcdSerial.print("?f");      // clear the LCD
  delay(1000);
  lcdSerial.print("?x00?y0");   // cursor to first character of line 0
  lcdSerial.println("   N873PW   V 3.0   ");
  lcdSerial.print("?x00?y1");   // cursor to first character of line 1
  lcdSerial.println(" BareBones  Arduino ");
  lcdSerial.print("?x00?y2");   // cursor to first character of line 2
  lcdSerial.println("Fuel Pressure, and  ");
  lcdSerial.print("?x00?y3");   // cursor to first character of line 3
  lcdSerial.println("Pounds of Smoke Oil ");
  delay(10000);                 // Wait 10 seconds to read it.
  lcdSerial.print("?f");       // clear the LCD
  delay(1000);
}

void loop() {

  // read the analog fuel pressure in value:

  currentMillis = millis();                     // How long has it been since last booted?

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;               // Reset the timer
    rawSensorValue = analogRead(analogInFuel);    // Read voltage from Garmin fuel pressure sensor
    sensorValue = rawSensorValue - garminOffset;  // Remove the 0.5V offset
    if (sensorValue <= 0) sensorValue = 0;        // No negative Fuel Pressure
  
    outputValue = map(sensorValue, 0, 818, 0, 255);  // 
    if (outputValue >= 255) outputValue = 255;       // Set max at full scale or else it wraps
    analogWrite(analogOutFuel, outputValue);         // Set the analog out value:

    // Now read the Smoke Oil Tank Level
     rawSensorValue = analogRead(analogInSmoke);      // Read voltage for level 0-5VDC
     smokeVolts = rawSensorValue / smokeResolution;   // steps / steps per volt
     smokeGallons = smokeVolts * smokeScaleFactor;    // Level in Gallons
     smokeWeight = smokeGallons * 7.1;                // Pounds of smoke oil onboard
  }

// Uncomment this last block to enable Fuel Pressure debug messaged to the LCD and monitor.
// +++++++++++++ BEGINING OF DEBUG CODE ++++++++++++++++++++++++++++++++++++
//  /*
  temp = outputValue;        // prepare for float math
  psig = temp/scaleFactor;   // Convert to PSIG
  temp = rawSensorValue;     // prepare for float math
  inputVoltage = temp/resolution;    // assuming 4.96vdc supply/reference

  // print the results to the serial monitor:
  Serial.print("RawSensor = ");
  Serial.print(rawSensorValue);
  Serial.print("\t AdjSensor = ");
  Serial.print(sensorValue);
  Serial.print("\t Output = ");
  Serial.print(outputValue);
  Serial.print("\t Input Voltage = ");
  Serial.print(inputVoltage);
  Serial.print(" VDC");
  Serial.print("\t Fuel Presure = ");
  Serial.print(psig);
  Serial.println(" PSIG");
  Serial.print("\t Smoke Volts = ");
  Serial.print(smokeVolts);
  Serial.print("Smoke Gallons = ");
  Serial.println(smokeGallons);
  Serial.print("Smoke Weight = ");
  Serial.println(smokeWeight);
  Serial.println();
  
  // print the results to the LCD display:
  lcdSerial.print("?x00?y0");   // cursor to first character of line 0
  lcdSerial.println("Tank Voltage:       "); 
  lcdSerial.print("?x16?y0");   // cursor to 16th character of line 0
  lcdSerial.print(smokeVolts, 2);
  lcdSerial.print("?x00?y1");   // cursor to first character of line 1
  lcdSerial.println("Pounds of Oil:       "); 
  lcdSerial.print("?x15?y1");   // cursor to 15th character of line 1
  lcdSerial.print(smokeWeight, 2);
  lcdSerial.print("?x00?y2");   // cursor to first character of line 2
  lcdSerial.println("Fuel Pressure:      "); 
  lcdSerial.print("?x16?y2");   // cursor to 16th character of line 2
  lcdSerial.print(psig, 1);
  lcdSerial.print("?x00?y3");   // cursor to first character of line 3
  lcdSerial.println("Voltage In:         "); 
  lcdSerial.print("?x16?y3");   // cursor to 16th character of line 3
  lcdSerial.print(inputVoltage, 2);
  
  delay(2000); // Wait 2 seconds while in debug.

// +++++++++++++++++++++++++ END OF DEBUG CODE +++++++++++++++++++++++++
//   */

}
