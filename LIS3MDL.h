/**
 * LIS3MDL 3-AXIS MAGNETOMETER SENSOR LIBRARY
 * 
 * Used to communicate with the STMicroelectronic's LIS3MDL magnetometer
 * sensor over I2C.
 * 
 * Code By: Michael Wrona
 * Created: 17 Jan 2021
 * 
 * Resources:
 * GitHub Code Repository: https://github.com/michaelwro/Arduino-LIS3MDL
 * LIS3MDL Datasheet: https://www.st.com/resource/en/datasheet/lis3mdl.pdf
 */


#ifndef __LIS3MDL_MAGNETOMETER__
#define __LIS3MDL_MAGNETOMETER__

#if ARDUINO >= 100
    #include "Arduino.h"
#else
    #include "WProgram.h"
#endif

#include <Wire.h>


// #define LIS3MDL_DEBUG  // For printing debug messages


// ------------------------------------
// Control Registers
// ------------------------------------

// Default I2C default slave address (0x1C). SDO/SA1 pin connected to GND.
// If you have an issue connecting to the sensor, try changing
// LIS3MDL_ADDR to 0x1C
#define LIS3MDL_ADDR 0x1E
#define LIS3MDL_WHOAMI 0x0F  // ID byte
#define LIS3MDL_CTRL_REG1 0x20  // Control register 1
#define LIS3MDL_CTRL_REG2 0x21  // Control register 2
#define LIS3MDL_CTRL_REG3 0x22  // Control register 3
#define LIS3MDL_CTRL_REG4 0x23  // Control register 4
#define LIS3MDL_CTRL_REG5 0x24  // Control register 5


// ------------------------------------
// Data Registers
// ------------------------------------
typedef enum {
    LIS3MDL_OUT_X_L = 0x28,
    LIS3MDL_OUT_X_H = 0x29,
    LIS3MDL_OUT_Y_L = 0x2A,
    LIS3MDL_OUT_Y_H = 0X2B,
    LIS3MDL_OUT_Z_L = 0x2C,
    LIS3MDL_OUT_Z_H = 0x2D
} LIS3MDL_DataReg_t;


// ------------------------------------
// Data Ranges
// ------------------------------------
typedef enum {
    LIS3MDL_RANGE_4G = 4,  // +/- 4 G range
    LIS3MDL_RANGE_8G = 8,  // +/- 8 G range
    LIS3MDL_RANGE_12G = 12,  // +/- 12G range
    LIS3MDL_RANGE_16G = 16  // +/-16G range
} LIS3MDL_MeasRange_t;


// ------------------------------------
// LIS3MDL Magnetometer Sensor Object
// ------------------------------------
class LIS3MDL_Mag
{
    public:
        LIS3MDL_Mag(LIS3MDL_MeasRange_t measRange = LIS3MDL_RANGE_4G);
        bool Begin(uint8_t i2cAddress = LIS3MDL_ADDR);
        bool ConfigureSensor();
        bool Read();

        float mx;  // X magnetometer reading [uT]
        float my;  // Y magnetometer reading [uT]
        float mz;  // Z magnetometer reading [uT]
    protected:
    private:
        LIS3MDL_MeasRange_t _range;  // Set meas. range
        void I2Cwrite8(uint8_t regOfInterest, uint8_t valToWrite);
        uint8_t I2Cread8(uint8_t regOfInterest);

};


#endif  // __LIS3MDL_MAGNETOMETER__
