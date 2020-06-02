# libapds9960
A Linux based user space driver for APDS9960 RGB and Gesture Sensor

## Building
Just add the .c and .h files to your project.

## Usage
To start, call apds9960_new() with a pointer to an apds9960_ctxp pointer and the device ID number for the i2c bus that the sensor is connected to. The library will open the device node at /dev/i2c-<dev_id>.
Once the context is initialized, call apds9960_set_chip_enabled() to bring up the chip with some default settings. Then call functions to set any settings you want modified from the defaults. After that, call apds9960_set_*_enabled for the features you would like to use.

## Example
See test/main.cpp for example code.
