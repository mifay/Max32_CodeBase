// Copyright 2013 Michael Fayad
//
// This file is part of Max32_CodeBase.
//
// HMC5883L.pde is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// HMC5883L.pde is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with HMC5883L.pde.  If not, see <http://www.gnu.org/licenses/>.

/*
 * source: - Love Electronics (loveelectronics.com)
 *         - Jordan McConnell, SparkFun Electronics
 *
 * Pin table
 * ==========
 * HMC5883L ->                   Max32
 * 3v3      ->                   3v3
 * GND      ->                   GND
 * SDA      ->(4.7k pull-up)  -> SDA1 (20)
 * SCL      ->(4.7k pull-up)  -> SCL1 (21)
 */

//I2C Library
#include <Wire.h>

#define HMC_ADDRESS 0x1E

// Registers
#define CONFIG_REG_A 0x00
#define OUTPUT_DATA_RATE_75_HZ 0x18
#define CONFIG_REG_B 0x01
#define GAIN_VALUE_8_GAUSS 0xE0
#define GAIN_VALUE_SCALE 4.35
#define MODE_REG 0x02
#define CONTINUOUS_MEASUREMENT 0x00
#define SINGLESHOT_MEASUREMENT 0x01
#define IDLE_MEASUREMENT 0x03
#define DATA_REG_BEGIN 0x03
#define ID_REG 0x0A
#define ID_REG_VALUE 0x48

/*
 * Max32 setup function. Executed at program beginning
 *
 * in  :
 * out :
 */
void setup()
{
    // Initialize Serial and I2C communications
    Serial.begin(115200);
    Wire.begin();

    // Put the HMC5883 IC into the correct operating mode
    Wire.beginTransmission(HMC_ADDRESS); // open communication with HMC5883
    Wire.send(MODE_REG);                 // select mode register
    Wire.send(CONTINUOUS_MEASUREMENT);   // continuous measurement mode
    Wire.endTransmission();
    
    // Improve HMC datarate to 75Hz
    Wire.beginTransmission(HMC_ADDRESS);
    Wire.send(CONFIG_REG_A);
    Wire.send(OUTPUT_DATA_RATE_75_HZ);
    Wire.endTransmission();
    
    // Set to best magnetism perceptibility, but lowest accuracy.
    Wire.beginTransmission(HMC_ADDRESS);
    Wire.send(CONFIG_REG_B);
    Wire.send(GAIN_VALUE_8_GAUSS);
    Wire.endTransmission(); 
}

/*
 * Max32 loop function.
 *
 * in  :
 * out :
 */
void loop()
{
    int x,y,z; //triple axis data

    //Tell the HMC5883 where to begin reading data
    Wire.beginTransmission(HMC_ADDRESS);
    Wire.send(DATA_REG_BEGIN); //select register 3, X MSB register
    Wire.endTransmission();


    //Read data from each axis, 2 registers per axis
    Wire.requestFrom(HMC_ADDRESS, 6);
    if(6<=Wire.available())
    {
        x = Wire.receive()<<8;  //X msb
        x |= Wire.receive();    //X lsb
        z = Wire.receive()<<8;  //Z msb
        z |= Wire.receive();    //Z lsb
        y = Wire.receive()<<8;  //Y msb
        y |= Wire.receive();    //Y lsb
        
        // Calculate heading when the magnetometer is level, then correct for signs of axis.
        float scaledY = GAIN_VALUE_SCALE * y;
        float scaledX = GAIN_VALUE_SCALE * x;
        float heading = atan2(scaledY, scaledX);
        
        // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
        // Find yours here: http://www.magnetic-declination.com/
        // Mine is: 2� 37' W, which is 2.617 Degrees, or (which we need) 0.0456752665 radians, I will use 0.0457
        // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
        float declinationAngle = -0.2578;
        heading += declinationAngle;
        
        // Correct for when signs are reversed.
        if(heading < 0)
        heading += 2*PI;

        // Check for wrap due to addition of declination.
        if(heading > 2*PI)
        heading -= 2*PI;

        // Convert radians to degrees for readability.
        float headingDegrees = heading * 180/M_PI;         
        
        //Print out values of each axis.
        Serial.print("x: ");
        Serial.print(x);
        Serial.print("  y: ");
        Serial.print(y);
        Serial.print("  z: ");
        Serial.print(z);
        Serial.print("  heading: ");
        Serial.println(headingDegrees);
    }
}
