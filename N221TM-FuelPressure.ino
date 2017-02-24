#include <SoftwareSerial.h>

/*
 * 
 *                      N221TM Arduino
 * 
 *          **********  Version 2.6 **********
 *          
 Reads an analog input pin, maps the result to a range from 0 to 255
 and uses the result to set the pulsewidth modulation (PWM) of an output pin.
 Also prints the results to the serial monitor.

 The first circuit:
   Garmin Fuel Pressure sensor connected to analog pin 0.
   Sensor ouputs 0.5 volts at 0.0 PSIG and 4.5 volts at 15.0 PSIG.
   Output is divided using 2K fixed resistor and a 200K 20-turn pot.

 The second circuit:
   9600 BAUD serial data from Garmin GPSMAP-496 is read into HW serial port,
   parsed and simple GPS message is sent at 4800 BAUD to the Trio AP-1 autopilot.
   
 The third circuit:
   eTape Smoke Oil Tank Level sensor connected to analog pin 1.


   
 
 Original AI and maping created 29 Dec. 2008
 modified 9 Apr 2012 by Tom Igoe
 Addapted Jan 2016 by Mike Rehberg for N221TM Fuel Pressure transducer 
                                        ouputs 0.5Vdc to 4.5Vdc while VM1000 is expecting 0 to 50mv.
                                        
 V2.0 March 2016 by Mike Rehberg:       Added Serial BAUD rate converter.  Garmin G496 outputs NMEA
                                        and Garmin COM data at 9600 BAUD.  Trio AP-1 autopilot can
                                        only read NMEA-0183 data at 4800 BAUD.

                                        Moved 9600 receive port to hardware serial port (D0).  It dropped
                                        bytes on the Software Serial port.

                                        Note that these are RS-232 level signels and as such need a
                                        TTL to RS-232 level shifter to interface with the Arduino.
                                        
 V2.2 May 9, 2016 by Mike Rehberg:      Added NMEA string code to read from G-496 at 9600 into the HW
                                        serial port and write out only NMEA code at 4800 to the Autopilot.
                                        Moed Serial output to A/P from Analog-2 to Digital-4 to match HW in plane.
                                        removed SoftwareSerial code for reading from G496.  HW reciever should
                                        be better about not dropping characters
                                        
   $GPRMB,A,0.66,R,KIKW,MTW,4407.71,N,08740.80,W,150.834,282.1,,V,D*2C
   $GPRMC,211512,A,4339.15,N,08415.97,W,0.0,9.0,110516,6.8,W,D*10
                                        
 V 2.5 May 11 2016 by Mike Rehberg:     Reading G496 message stream at 9600 and only sending NMEA
                                        $GPRMB and $GPRMC messeges to autopilot at 4800.  
                                        will only send one message out of 2 or 3 which is about 1 per two sec.

                                        Added a timer on the analog read/write of the fuel pressure sensor.
                                        The VM1000 is slow anyway so no need to waste Arduino cycles.  Currently
                                        set to update once every 2.925 seconds.

 V 2.6  Feb 24, 2017 by Mike Rehberg:   Added Smoke oil tank level sensor to Analog Input pin A2.
                                        Level is read by eTape sensor which outputs 0-5 volts based on 
                                        0 to 12" of level in tank.
                                        *****  NEEDS TO BE TESTED AND CALIBRATED *******
 

 Things to add:                         + Hardware switch and code to enable debug to LCD display
 
 */

// These constants won't change.  They're used to give names
// to the pins used:
// Note Pin-0 is hardware RX and Pin-1 is hardware TX.  The RX Pin-0 will be connected to the G496 TX

const int analogInFuel   = A0; // Analog input pin that the Fuel Sensor is attached to
const int analogInSmoke  = A2; // Analog input pin for Smoke Oil Tank level sensor
const int analogOutFuel  = 3;  // Analog output pin that is voltage divided before going to
float psig;                    // The VM-1000
float inputVoltage;
float temp;
float resolution=206.25;    // Input steps per Volt  1023/4.96
float scaleFactor=17.0;     // Output steps per PSIG: 255/15
float smokeLevel;
float smokeResolution = 204.6;      // Input steps per Volt 1023 / 5.00
float smokeScaleFactor = 0.91667;   // Volts per Gallon (.4167V/in, 4.583V=Full, .9167V/Gallon)
float smokeVolts;
float smokeWeight;          // How much somke juice onboard in lbs.
int   garminOffset=102;     // Steps for 0.5 VDC offset to 0.0 PSIG
char  incomingByte;         // BAUD conversion buffer
String g496String ="                                      ";      // String read in from G496
int    countB =0;
int    countC =0;
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

    
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);        // Harware Serail port to read NMEA and COM data from G-496

  pinMode(txPinLCD, OUTPUT); // Serial output to 4/20 character LCD display
  lcdSerial.begin(9600);      // 9600 baud is chip comm speed

  pinMode(txPinAP1, OUTPUT);  // Serial output to NavAid AP-1 Autopilot
  ap1Serial.begin(4800);      // 4800 baud is chip comm speed
  
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
  lcdSerial.println("   N221TM   v 2.5   ");
  lcdSerial.print("?x00?y1");   // cursor to first character of line 1
  lcdSerial.println(" BareBones  Arduino  ");
  lcdSerial.print("?x00?y2");   // cursor to first character of line 2
  lcdSerial.println(" Fuel Pressure and  ");
  lcdSerial.print("?x00?y3");   // cursor to first character of line 3
  lcdSerial.println(" G496 to A/P Buffer ");
  delay(2000);                 // Wait to read it.
  lcdSerial.print("?f");       // clear the LCD
  delay(1000);
}

void loop() {

  if(Serial.available() > 0) {               // Read GPS data at 9600 BAUD into the Hardware Serial Port
    incomingByte = Serial.read();            // Move byte to temp storage to be ready to write to A/P
    if (incomingByte != '\n') {              // Look for NewLine terminator
      if (incomingByte == '$') {             // Begining of NMEA string
        g496String = "";                     // Clear string buffer
      }
      g496String += incomingByte;            // Move character into next open space in string
    }
    else {
      if (g496String.substring(1, 6) == "GPRMB") {
        if (countB == 2) {
          ap1Serial.println(g496String);      // Write NMEA string to the AutoPilot at 4800
          countB = 0;
        }
        else countB = countB + 1;
      }
      else if (g496String.substring(1, 6) == "GPRMC") {
        if (countC == 2) {
          ap1Serial.println(g496String);      // Write NMEA string to the AutoPilot at 4800
          countC = 0;
        }
        else countC = countC +1;       
      }
      g496String = "";
    }
  }

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
     smokeVolts = rawSensorValue / smokeResolution;
     smokeLevel = smokeVolts * smokeScaleFactor;      // Level in Gallons
     smokeWeight = smokeLevel * 8.0;                  // Pounds of smoke oil onboard
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
  Serial.println(smokeLevel);
  Serial.println();
  
  // print the results to the LCD display:
  lcdSerial.print("?x00?y2");   // cursor to first character of line 2
  lcdSerial.println("Fuel Pressure:      "); 
  lcdSerial.print("?x15?y2");   // cursor to 15th character of line 2
  lcdSerial.print(psig, 1);
  lcdSerial.print("?x00?y3");   // cursor to first character of line 3
  lcdSerial.println("Voltage In:      "); 
  lcdSerial.print("?x15?y3");   // cursor to 15th character of line 3
  lcdSerial.print(inputVoltage, 2);
  
  delay(2000); // Wait 2 seconds while in debug.

// +++++++++++++++++++++++++ END OF DEBUG CODE +++++++++++++++++++++++++
//   */

}
