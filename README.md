# Adafruit LSM6DS ![Build Status](https://github.com/adafruit/Adafruit_LSM6DS/workflows/Arduino%20Library%20CI/badge.svg)

This is the Adafruit LSM6DS 6-DoF Accelerometer and Gyroscope Sensor Library for Arduino

Tested and works great with the Adafruit LSM6DSOX Breakout Board 
[<img src="assets/board.jpg?raw=true" width="500px">](https://www.adafruit.com/products/4438)

Updated for LSM6DSV320 and all other LSM6DSV... variants. These chips have a different pinout to the others
and either a "dual channel" or a complete second "high G" accelerometer in the package.
They also have "sensor fusion low power" (SFLP) to combine accelerometer and gyro into a single orientation solution.

Updated for FIFO on all chips with "standard FIFO". Seems like a few (maybe older) part numbers use a different kind of FIFO. 

Updated for Sensor Hub on all chips which have this function. Only two part numbers in the LSM6 family don't have sensor hub.

This chip uses I2C to communicate, 2 pins are required to interface

Adafruit invests time and resources providing this open source code, please support Adafruit and open-source hardware by purchasing products from Adafruit!

# Dependencies
* [Adafruit_BusIO](https://github.com/adafruit/Adafruit_BusIO)
* [Adafruit_Sensor](https://github.com/adafruit/Adafruit_Sensor)

# Contributing

Contributions are welcome! Please read our [Code of Conduct](https://github.com/adafruit/Adafruit_LSM6DSOX/blob/master/CODE_OF_CONDUCT.md>)
before contributing to help this project stay welcoming.

## Documentation and doxygen
Documentation is produced by doxygen. Contributions should include documentation for any new code added.

Some examples of how to use doxygen can be found in these guide pages:

https://learn.adafruit.com/the-well-automated-arduino-library/doxygen

https://learn.adafruit.com/the-well-automated-arduino-library/doxygen-tips

Written by Bryan Siepert for Adafruit Industries.  
BSD license, check license.txt for more information
All text above must be included in any redistribution

To install, use the Arduino Library Manager and search for "Adafruit LSM6DS" and install the library.
