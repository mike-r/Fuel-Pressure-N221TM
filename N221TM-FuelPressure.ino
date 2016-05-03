#include <MicroNMEA.h>
#include <SoftwareSerial.h>

/*
 * 
 *          **********  Version 2.0 **********
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

#define rxPinLCD 2    //  rxPinLCD is immaterial - not used - just make this an unused Arduino pin number
#define txPinLCD 15   //  txpinLCD for Serial 4x20 LCD Display using Analog(1)

#define rxPinAP1 5    //  rxPinAP1 is immaterial - not used - just make this an unused Arduino pin number
#define txPinAP1 16   //  txpinAP1 for output to SmartGPS using Analog(2)

#define rxPinG496 4   //  rxPinG496 is the 9600 BAUD serial output from the G496
#define txPinG496 17  //  txPinG496 is not used but would be Analog(3)

SoftwareSerial lcdSerial  =  SoftwareSerial(rxPinLCD, txPinLCD);
SoftwareSerial ap1Serial  =  SoftwareSerial(rxPinAP1, txPinAP1);
SoftwareSerial g496Serial =  SoftwareSerial(rxPinG496, txPinG496);

int rawSensorValue = 0;     // value read from the Garmin Sensor
int sensorValue;
int outputValue = 0;        // value output to the PWM (analog out)

void setup() {
  pinMode(analogOutPin, OUTPUT);

    
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);        // Diagnostics serial to PC running Arduino application and G496 NMEA/COM

  pinMode(txPinLCD, OUTPUT); // Serial output to 4/20 character LCD display
  lcdSerial.begin(9600);      // 9600 baud is chip comm speed

  pinMode(txPinAP1, OUTPUT);  // Serial output to NavAid AP-1 Autopilot
  ap1Serial.begin(4800);      // 4800 baud is chip comm speed

  pinMode(rxPinG496, INPUT);  // Serial input from G496
  g496Serial.begin(9600);     // 9600 BAUD is the com speed of the G496
  
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
}

void loop() {

  if(Serial.available() > 0) {               // Read GPS data at 9600 BAUD into the Hardware Serial Port
    incomingByte = Serial.read();            // Move byte to temp storage to be ready to write to A/P
//    if(g496Serial.available() >0) {        // Same as above but using Software Serial port
//      incomingByte = g496Serial.read();    // Didn't work so this code and init for it should be cutout   
    ap1Serial.print(incomingByte);           // Write to the A/P at 4800 BAUD
  }

  // read the analog fuel pressure in value:
  rawSensorValue = analogRead(analogInPin);
  sensorValue = rawSensorValue - garminOffset;  // Remove the 0.5V offset
  if (sensorValue <= 0) sensorValue = 0;        // No negative Fuel Pressure
  
  // map it to the range of the analog out:
  outputValue = map(sensorValue, 0, 818, 0, 255);  // 
  if (outputValue >= 255) outputValue = 255;       // Set max at full scale or else it wraps
  analogWrite(analogOutPin, outputValue);          // Set the analog out value:

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
  
  // wait 2 milliseconds before the next loop
  // for the analog-to-digital converter to settle
  // after the last reading:
  delay(2);
  */
  delay(2000); // Wait 2 seconds.  Going faster dosn't matter to the VM1000.

// +++++++++++++++++++++++++ END OF DEBUG CODE +++++++++++++++++++++++++

}
