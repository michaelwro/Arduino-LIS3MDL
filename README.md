# Arduino-LIS3MDL

An easy to understand Arduino sensor library for the LIS3MDL 3-axis magnetometer.

Code By: Michael Wrona, *B.S. Aerospace Engineering*

Check out my blog: [mwrona.com](https://mwrona.com/)

## How To Use

Simply download this repository into you Arduino `libraries/` folder and put `#include "LIS3MDL.h"` at the top of your sketch. That easy!

The library will output magnetometer measurements in microtesla ($\mu T$), but the sensor natively outputs in Gauss (G). You can change the code to output in Gauss in `LIS3MDL.cpp`. Just get rid of the multiplication by 100, i.e. `this->mx = ((float)mxRaw / 6842.0f) * 100.0f`.

The default configuration I programmed for the LIS3MDL is **Ultra-high performance, FAST_ODR=true, ODR=155Hz, continuous output.**

## Example

An example of how to use the code can be found in the `examples/LIS3MDL_DemoCode` directory.


## Arduino Uno Wiring

| LIS3MDL | Arduino Uno |
| --------|--------------|
|VCC/VDD | 3.3V |
| SDA | A4 |
| SCL | A5 |
| GND | GND|


## Troubleshooting

If you have an issue connecting to your LIS3MDL sensor, try changing `LIS3MDL_ADDR` to `0x1C` in `LIS3MDL.h`. The value coded in my software is `0x1E`. This worked for my sensor, but yours might be different. I've seen `0x1E` in others' code, so mine might be special. Try it out before submitting an issue.

## Testing Notes

I tested this code with the LIS3MDL sensor onboard my [mRobotics NEO-M8N Dual Compass GPS Receiver](https://store.mrobotics.io/mRo-GPS-u-Blox-Neo-M8N-Dual-Compass-LIS3MDL-IST831-p/mro-gps003-mr.htm) hooked up to an Arduino Uno.

## Resources

* [STMicroelectronic's LIS3MDL Datasheet](https://www.st.com/resource/en/datasheet/lis3mdl.pdf)
* [Magnetometer Calibration Procedure](https://youtu.be/MinV5V1ioWg)
