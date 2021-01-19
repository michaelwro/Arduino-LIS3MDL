/**
 * LIS3MDL I2C MAGNETOMETER TESTING SCRIPT
 * 
 * This code runs on an Arduino Uno to test the LIS3MDL code
 * library I developed.
 * 
 * Code By: Michael Wrona
 * Created: 18 Jan 2021
 * 
 * Resources:
 * 
 */


#include "LIS3MDL.h"
#include <Wire.h>


float mx, my, mz;  // Measurements in [uT]

LIS3MDL_Mag Magnetometer(LIS3MDL_RANGE_4G);


void setup()
{
    Serial.begin(9600);
    Wire.begin();

    delay(250);
    if (Magnetometer.Begin() == false)
        Serial.println("ERROR INITIALIZING SENSOR.");
    
    if (Magnetometer.ConfigureSensor() == false)
        Serial.println("ERROR CONFIGURING SENSOR.");
    
    Serial.println("GOOD!");
}



void loop()
{
    // Read data
    Magnetometer.Read();
    
    // Print data
    Serial.print("mx: "); Serial.print(Magnetometer.mx, 4);
    Serial.print("  my: "); Serial.print(Magnetometer.my, 4);
    Serial.print("  mz: "); Serial.println(Magnetometer.mz, 4);

    delay(250);  // 4 Hz
}