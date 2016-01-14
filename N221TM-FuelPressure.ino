#include <SoftwareSerial.h>

/*
  Analog input, analog output, serial output

 Reads an analog input pin, maps the result to a range from 0 to 255
 and uses the result to set the pulsewidth modulation (PWM) of an output pin.
 Also prints the results to the serial monitor.

 The circuit:
 * potentiometer connected to analog pin 0.
   Center pin of the potentiometer goes to the analog pin.
   side pins of the potentiometer go to +5V and ground
 * LED connected from digital pin 9 to ground

 created 29 Dec. 2008
 modified 9 Apr 2012
 by Tom Igoe

 This example code is in the public domain.

 */

// These constants won't change.  They're used to give names
// to the pins used:
const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to
const int analogOutPin = 3;  // Analog output pin that the LED is attached to
float psig;
float inputVoltage;
float temp;
float resolution=206.25;    // Steps per Volt  1023/4.96
float scaleFactor=17.0;     // Steps per PSIG: 255/15
int   garminOffset=102;     // Steps for 0.5 VDC

#define rxPin 2   //  rxPin is immaterial - not used - just make this an unused Arduino pin number
#define txPin 15   //  txpin using Analog(1)

SoftwareSerial mySerial =  SoftwareSerial(rxPin, txPin);

int rawSensorValue = 0;        // value read from the pot
int sensorValue;
int outputValue = 0;        // value output to the PWM (analog out)

void setup() {
  pinMode(analogOutPin, OUTPUT);
  
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);

  pinMode(txPin, OUTPUT);
  mySerial.begin(9600);      // 9600 baud is chip comm speed
  mySerial.print("?G420");   // set display geometry,  4 x 20 characters in this case
  delay(500);                // pause to allow LCD EEPROM to program
  mySerial.print("?Bff");    // set backlight to ff hex, maximum brightness
  delay(1000);               // pause to allow LCD EEPROM to program
  mySerial.print("?s6");     // set tabs to six spaces
  delay(1000);               // pause to allow LCD EEPROM to program
  mySerial.print("?c0");     // turn cursor off
  delay(300);
  mySerial.print("?f");      // clear the LCD
  delay(1000);
  mySerial.print("?x00?y0");   // cursor to first character of line 0
  mySerial.println("       N221TM       ");
  mySerial.print("?x00?y2");   // cursor to first character of line 2
  mySerial.println("    Fuel Pressure   ");
  delay(2000);                 // Wait to read it.
}

void loop() {
  // read the analog in value:
  rawSensorValue = analogRead(analogInPin);
  sensorValue = rawSensorValue - garminOffset;  // Remove the 0.5V offset
  if (sensorValue <= 0) sensorValue = 0;     // No negative Fuel Pressure
  
  // map it to the range of the analog out:
  outputValue = map(sensorValue, 0, 818, 0, 255);  // 
  if (outputValue >= 255) outputValue = 255;        // Set max at full scale or else it wraps
  analogWrite(analogOutPin, outputValue);            // Set the analog out value:
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
  

  
  mySerial.print("?x00?y2");   // cursor to first character of line 2
  mySerial.println("Fuel Pressure:      "); 
  mySerial.print("?x15?y2");   // cursor to 15th character of line 2
  mySerial.print(psig, 1);
  mySerial.print("?x00?y3");   // cursor to first character of line 3
  mySerial.println("Voltage In:      "); 
  mySerial.print("?x15?y3");   // cursor to 15th character of line 3
  mySerial.print(inputVoltage, 2);
  
  // wait 2 milliseconds before the next loop
  // for the analog-to-digital converter to settle
  // after the last reading:
  delay(2);
  delay(2000); // Wait 2 seconds;
}
