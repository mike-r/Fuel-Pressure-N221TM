#include <SoftwareSerial.h>

/*
 * 
 *          **********  Version 2.3 **********
 *          
 Reads an analog input pin, maps the result to a range from 0 to 255
 and uses the result to set the pulsewidth modulation (PWM) of an output pin.
 Also prints the results to the serial monitor.

 The circuit:
   Garmin Fuel Pressure sensor connected to analog pin 0.
   Sensor ouputs 0.5 volts at 0.0 PSIG and 4.5 volts at 15.0 PSIG.
   Output is divided using 2K fixed resistor and a 200K 20-turn pot.
 
 created 29 Dec. 2008
 modified 9 Apr 2012 by Tom Igoe
 modified Januauary 2016 by Mike Rehberg: For N221TM Fuel Pressure transducer ouputs 0.5Vdc to 4.5Vdc
                                        while VM1000 is expecting 0 to 50mv.
                                        
 modiified March 2016 by Mike Rehberg:  Added Serial BAUD rate converter.  Garmin G496 outputs NMEA
                                        and Garmin COM data at 9600 BAUD.  Trio AP-1 autopilot can
                                        only read NMEA-0183 data at 4800 BAUD.

                                        Moved 9600 receive port to hardware serial port (D0).  It dropped
                                        bytes on the Software Serial port.

                                        Note that these are RS-232 level signels and as such need a
                                        TTL to RS-232 level shifter to interface with the Arduino.
                                        
 modified May 2016 by Mike Rehberg:     Added NMEA string code to read from G-496 at 9600 into the HW
                                        serial port and write out only NMEA code at 4800 to the Autopilot.
                                        Works fine in "Heading" mode ($GPRMC) but not at all in "Intercept"
                                        or "Cource" mode ($GPRMB).

                                        $GPRMB message not getting through.
                                                                               
 */

// These constants won't change.  They're used to give names
// to the pins used:
// Note Pin-0 is hardware RX and Pin-1 is hardware TX.  The RX Pin-0 will be connected to the G496 TX

const int analogInPin = A0;  // Analog input pin that the Fuel Sensor is attached to
const int analogOutPin = 3;  // Analog output pin that is voltage divided before going to
float psig;                  // The VM-1000
float inputVoltage;
float temp;
float resolution=206.25;    // Input steps per Volt  1023/4.96
float scaleFactor=17.0;     // Output steps per PSIG: 255/15
int   garminOffset=102;     // Steps for 0.5 VDC offset to 0.0 PSIG
char  incomingByte;         // BAUD conversion buffer
String g496String ="                                        "; // String read in from G496
boolean line0 = true;       // Keep track which line of the LCD display we are on

unsigned long currentMillis  = 0;     // How long the Arduino has been running (will loop in 50 days)
unsigned long previousMillis = 0;     // will store last time the Fuel Pressure was updated
const long interval = 5;              // Interval at which to read & write Fuel Pressure (milliseconds)


#define rxPinLCD 2    //  rxPinLCD is immaterial - not used - just make this an unused Arduino pin number
#define txPinLCD 15   //  txpinLCD for Serial 4x20 LCD Display using Analog(1)

#define rxPinAP1 5    //  rxPinAP1 is immaterial - not used - just make this an unused Arduino pin number
#define txPinAP1 4    //  txpinAP1 for output to SmartGPS using Digital(4)


SoftwareSerial lcdSerial  =  SoftwareSerial(rxPinLCD, txPinLCD);
SoftwareSerial ap1Serial  =  SoftwareSerial(rxPinAP1, txPinAP1);

int rawSensorValue = 0;     // value read from the Garmin Sensor
int sensorValue;
int outputValue = 0;        // value output to the PWM (analog out)

void setup() {
  pinMode(analogOutPin, OUTPUT);

    
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);        // Harware Serail port to read NMEA and COM data from G-496

  pinMode(txPinLCD, OUTPUT); // Serial output to 4/20 character LCD display
  lcdSerial.begin(9600);      // 9600 baud is chip comm speed

  pinMode(txPinAP1, OUTPUT);  // Serial output to NavAid AP-1 Autopilot
  ap1Serial.begin(4800);      // 4800 baud is chip comm speed
  
  lcdSerial.print("?G420");   // set display geometry,  4 x 20 characters in this case
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
  lcdSerial.println("       N221TM       ");
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
        g496String ="";                      // Clear string buffer
      }
      g496String += incomingByte;            // Move character into next open space in string
    }
    else {                                  // Full message in string buffer
      if (g496String.substring(0, 2) == "$GP") {     // Check to see if this is a NMEA string
        ap1Serial.println(g496String);      // Write NMEA string to the AutoPilot at 4800
        if (line0) {
          lcdSerial.print("?x00?y0");         // cursor to first character of line 0
          lcdSerial.print(g496String);        // write string to the LCD
          line0 = false;                      // Prepare to write the next string on the next line
        }
        else {
          lcdSerial.print("?x00?y2");         // cursor to first character of line 0
          lcdSerial.print(g496String);        // write string to the LCD
          line0 = true;    
        }
      }
      g496String = "";                        // Clear string buffer
    }
  }

  currentMillis = millis();                          // How long has it been since booted?
  if (currentMillis - previousMillis <= interval) {  // Time to read & write the FuelPressure
    previousMillis = currentMillis;                  // Reset the timer
    
    rawSensorValue = analogRead(analogInPin);        // read the analog fuel pressure in value
    sensorValue = rawSensorValue - garminOffset;     // Remove the 0.5V offset
    if (sensorValue <= 0) sensorValue = 0;           // No negative Fuel Pressure
    outputValue = map(sensorValue, 0, 818, 0, 255);  // map it to the range of the analog out
    if (outputValue >= 255) outputValue = 255;       // Set max at full scale or else it wraps
    analogWrite(analogOutPin, outputValue);          // Set the analog out value
  }

// Uncomment this last block to enable Fuel Pressure debug messaged to the LCD and monitor.
// +++++++++++++ BEGINING OF DEBUG CODE ++++++++++++++++++++++++++++++++++++
/*
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
  
  // print the results to the LCD display:
  lcdSerial.print("?x00?y2");   // cursor to first character of line 2
  lcdSerial.println("Fuel Pressure:      "); 
  lcdSerial.print("?x15?y2");   // cursor to 15th character of line 2
  lcdSerial.print(psig, 1);
  lcdSerial.print("?x00?y3");   // cursor to first character of line 3
  lcdSerial.println("Voltage In:      "); 
  lcdSerial.print("?x15?y3");   // cursor to 15th character of line 3
  lcdSerial.print(inputVoltage, 2);
  
  delay(2000); // Wait 2 seconds.  Going faster dosn't matter to the VM1000.

// +++++++++++++++++++++++++ END OF DEBUG CODE +++++++++++++++++++++++++
*/

  // wait 2 milliseconds before the next loop
  // for the analog-to-digital converter to settle
  // after the last reading:

// Try to speed things up://  delay(2);

}
