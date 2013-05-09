Overview
========

This repository contains code projects with the Max32 microcontroller. The code projects are presented below.


ADC 12 bits code
----------------

This code project is about making the Max32 communicate via an SPI channel with a 12 bits ADC.


LSM330 accelerometer code
-------------------------

This code project is about making the Max32 communicate via an SPI channel with the LSM330 accelerometer.


LSM330 gyroscope code
---------------------

This code project is about making the Max32 communicate via an SPI channel with the LSM330 gyroscope.


L3G4200D gyroscope code
-----------------------

This code project is about making the Max32 communicate via an SPI channel with the L3G4200D gyroscope.


HMC5883L magnetometer code
--------------------------

This code project is about making the Max32 communicate in I2C with the HMC5883L magnetometer.


HMC5883L magnetometer code with LSM330 accelerometer code
----------------------------------------------------------

This code project is about making the Max32 communicate in I2C with the HMC5883L magnetometer and communication in SPI with the LSM330 accelerometer.

The accelerometer is used to compensate the tilting of the compass. The accelerometer can compensate up to 44 degrees of compass tilting. Also, there must be no other acceleration than earth's gravitational acceleration.



Prerequisites
=============
* [Download and install MPIDE](http://chipkit.org/wiki/?title=MPIDE_Installation)
* [Buy a Max32](http://www.digilentinc.com/Products/Detail.cfm?Prod=CHIPKIT-MAX32)
* [Buy a 12 bits ADC](http://www.digikey.ca/product-detail/en/MCP3208-CI%2FP/MCP3208-CI%2FP-ND/305928?cur=USD)
* [Buy an LSM330](http://ca.mouser.com/ProductDetail/STMicroelectronics/LSM330DL/?qs=sGAEpiMZZMvhQj7WZhFIABvlHH1EqRb7AHxxrBGGw5U%3d)
* [Buy a L3G4200D](https://www.sparkfun.com/products/10612?)
* [Buy an HMC5883L](https://www.sparkfun.com/products/10530)



Program execution setup
=======================

* Execute MPIDE on your pc
* Open a code project on your MPIDE (/File/Open.../)
* Choose the right board for the program execution (/Tools/Board/chipKIT MAX32)
* Choose the right port for the program execution on the board (/Tools/Serial Port/)
* Upload the code on your board
* Open the serial monitor and make sure you are listening on the right port at the right baud rate
* You should read the output values on the monitor



Data interpretation
===================

ADC 12 bits code
----------------

The ADC input value is calculated : AdcOutputValue/4096*5 = AdcInputValueInVolts

* For a 5V ADC input value, you should read 4096 on the monitor.
* For a 3.3V ADC input value, you should read a value around 2700 on the monitor.
* For a 0V ADC input value, you should read 0 on the monitor.


LSM330 accelerometer code
-------------------------

The LSM330 accelerometer output value is a linear acceleration measured in g.


LSM330 gyroscope code
---------------------

The LSM330 gyro output value is an angular rate measured in degrees per second (dps).


L3G4200D gyroscope code
-----------------------

The L3G4200D output value is an angular rate measured in degrees per second (dps).


HMC5883L magnetometer code
--------------------------

The HMC5883L output value is scaled in degrees. From 0 to 359.9 degrees.

* If it points to the North, the heading value should be 0.
* If it points to the East, the heading value should be 90.
* If it points to the South, the heading value should be 180.
* If it points to the West, the heading value should be 270.


HMC5883L magnetometer code with LSM330 accelerometer code
---------------------------------------------------------
The HMC5883L output value is scaled in degrees. From 0 to 359.9 degrees.

* If it points to the North, the heading value should be 0.
* If it points to the East, the heading value should be 90.
* If it points to the South, the heading value should be 180.
* If it points to the West, the heading value should be 270.

The LSM330 accelerometer output value is a linear acceleration measured in g. The output data is properly scaled.



Authors
=======
* Michaël Fayad
* Raphaël Guimond