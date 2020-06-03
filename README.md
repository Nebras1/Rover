# Rover
Python RM3100
=============

This project contains a python module for interfacing with SPI RM3100 integrated with MPU6050 from user space via the spidev linux kernel driver.

All code is MIT licensed unless explicitly stated otherwise.


Hints
-----
This project is depends on spidev library to make the SPI connection and 
you have to check the SPI connection as mentioned here before make the communication
https://www.raspberrypi.org/documentation/hardware/raspberrypi/spi/README.md

Usage
-----


```Connection
The MPU6050 to RPI
SDA --> SDA
SCL --> SCL
VCC --> 3.3V
GND --> GND

The RM3100 to RPI
MOSI --> MOSI
MISO --> MISO
SSN --> GPIO PIN(17 for example)
DRDY --> GPIO PIN(27 for example)
VCC --> 3.3V
GND --> GND
SCK --> SCLK

The Teensy to RPI
18(A4) --> SDA
19(A5) --> SCL
GND --> GND

first make sure for the directions to be as shown


```

![alt text](https://raw.githubusercontent.com/Ahmed-Dakrory/RM3100_With_MPU6050/master/Directions.jpg)
