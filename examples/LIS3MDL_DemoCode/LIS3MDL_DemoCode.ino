/**
 * ARDUINO LIS3MDL EXAMPLE/DEMO CODE
 * 
 * A working example for the LIS3MDL magnetometer library. You can find the 
 * source code at: https://github.com/michaelwro/Arduino-LIS3MDL
 * 
 * Code By: Michael Wrona
 * Created: 18 Jan 2021
 * 
 * Arduino Uno Pin Layout:
 * (LIS3MDL) -> (Arduino)
 *     VDD/VCC -> 3.3V
 *     SDA -> A4
 *     SCL -> A5
 *     GND -> GND
 */


/**
 * You can comment/uncomment this to enable/disable simple debugging messages.
 * They will print to the default serial console. You can also change this at
 * the top of 'LIS3MDL.h'
 */
#define LIS3MDL_DEBUG  // For printing debug messages


#include <Wire.h>
#include "LIS3MDL.h"

/**
 * Create Magnetometer object. You can specify a measurement range, if 
 * desired. The default is 4 gauss.
 * 
 * LIS3MDL_RANGE_4G (4 gauss = 400 microtesla)
 * LIS3MDL_RANGE_8G (8 gauss = 800 microtesla)
 * LIS3MDL_RANGE_12G (12 gauss = 1,200 microtesla)
 * LIS3MDL_RANGE_16G (16 gauss = 1,600 microtesla)
 */
LIS3MDL_Mag Magnetometer(LIS3MDL_RANGE_4G);


void setup()
{
    Serial.begin(9600);
    Wire.begin();

    /** 
     * 'Begin' function checks the ID byte of the sensor and also
     * the measurement range.
     */
    if (Magnetometer.Begin() == false)
        Serial.println("ERROR INITIALIZING SENSOR.");
    
    /**
     * 'Configure' function sends command to the sensor's I2C registers
     * to set parameters such as measurement range, sample rate, etc.
     */
    if (Magnetometer.ConfigureSensor() == false)
        Serial.println("ERROR CONFIGURING SENSOR.");
}


void loop()
{
    // Read data
    Magnetometer.Read();
    
    // Print data
    // Outputs in microtesla [uT]
    Serial.print("mx: "); Serial.print(Magnetometer.mx, 4); Serial.print("uT");
    Serial.print("  my: "); Serial.print(Magnetometer.my, 4); Serial.print("uT");
    Serial.print("  mz: "); Serial.println(Magnetometer.mz, 4); Serial.print("uT");

    delay(250);  // 4 Hz
}
