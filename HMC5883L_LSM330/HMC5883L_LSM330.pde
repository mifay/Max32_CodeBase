// Copyright 2013 Michael Fayad
//
// This file is part of Max32_CodeBase.
//
// HMC5883L_LSM330.pde is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// HMC5883L_LSM330.pde is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with HMC5883L_LSM330.pde.  If not, see <http://www.gnu.org/licenses/>.

/*
 * source: - Tom Pycke (http://code.google.com/p/gluonpilot)
 *         - Love Electronics (loveelectronics.com)
 *         - Jordan McConnell, SparkFun Electronics
 *
 * Pin table
 * ==========
 * HMC5883L ->                   Max32
 * 3v3      ->                   3v3
 * GND      ->                   GND
 * SDA      ->(4.7k pull-up)  -> SDA1 (20)
 * SCL      ->(4.7k pull-up)  -> SCL1 (21)
 *
 * LSM330   ->                   Max32
 * 3v3      ->                   3V3
 * GND      ->                   GND
 * SDO      ->                   SDI(J13-1)(first pin)
 * SCL      ->                   SCL(J13-3)
 * CSG      ->                   (6)
 * CS A     ->                   (10)
 * SDI      ->                   SDO(J13-4)
 */

#include <Wire.h> // For i2c communication
#include <SPI.h>
#include "lsm330.h"
#include "hmc5883l.h"
#include "attitude.h"

// pin definitions
const int chipSelectGyro = 6;
const int chipSelectAccel = 10;

unsigned long tBegin = 0; unsigned long tEnd = 0;

HMC5883L compass;
LSM330 lsm; // Gyroscope and accelerometer
Attitude attitude; //quaternions

bool skipHeading = true;
bool skipCompass = true;

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

    // Start the SPI library:
    SPI.begin();
    SPI.setDataMode(SPI_MODE3);
    SPI.setClockDivider(SPI_CLOCK_DIV2);

    // Set gyro pin values
    pinMode(chipSelectGyro, OUTPUT);
    digitalWrite(chipSelectGyro, HIGH);

    // Set accelerometer pin values
    pinMode(chipSelectAccel, OUTPUT);
    digitalWrite(chipSelectAccel, HIGH);

    // Don't move the device for 5 seconds
    lsm.setup(chipSelectGyro, 250, chipSelectAccel, 16);

    compass.setup(1.3);

    attitude.init(0,0,0);
}

/*
 * Max32 loop function.
 *
 * in  :
 * out :
 */
void loop()
{
    tBegin = micros();
    
    // Infinite loop
    do
    {  
        delay(50); // for monitor lisibility. else mpide crashes
        
        lsm.getGyroValues(chipSelectGyro);
        
        lsm.getAccelValues(chipSelectAccel);
    
        // reset skip values for up to date data printing
        skipCompass = true;
        skipHeading = true;        
    
        if(compass.readMagnetism())
        {
            skipCompass = false;
            
            float rollRadians = asin(lsm.scaledAccelY_);
            float pitchRadians = asin(lsm.scaledAccelX_);
    
            if(rollRadians > 0.78 || rollRadians < -0.78 || pitchRadians > 0.78 || pitchRadians < -0.78)
            {
                // We cannot correct for tilt over 44 degrees with this algorithm, if the board is tilted as such, do nothing...
            }
            else
            {
                skipHeading = false;
                compass.calcHeading(rollRadians, pitchRadians/*, yawRadians*/);
            }
        }
        // else carry on but we won't have magnetism data
    
        tEnd = micros();
        float deltaT = tEnd - tBegin;
        attitude.update(lsm.scaledGyroX_, lsm.scaledGyroY_, lsm.scaledGyroZ_, deltaT);
        printAttitude();
        
        tBegin = tEnd;
        
        printAll();
    }while(true);
}

/*
 * Print lsm330 gyroscope data
 *
 * in  :
 * out :
 */
void printGyro()
{
    // Print gyro values
    Serial.print(lsm.scaledGyroX_);
    Serial.print("\t");
    Serial.print(lsm.scaledGyroY_);
    Serial.print("\t");
    Serial.print(lsm.scaledGyroZ_);
    Serial.println("\t dps");
    
    /*Serial.print(lsm.gyroX_);
    Serial.print("\t");
    Serial.print(lsm.gyroY_);
    Serial.print("\t");
    Serial.print(lsm.gyroZ_);
    Serial.println("\t counts");*/
}

/*
 * Print lsm330 accelerometer data
 *
 * in  :
 * out :
 */
void printAccelero()
{
    Serial.print(lsm.scaledAccelX_);
    Serial.print("\t");
    Serial.print(lsm.scaledAccelY_);
    Serial.print("\t");
    Serial.print(lsm.scaledAccelZ_);
    Serial.println("\t g");
    
    /*Serial.print(lsm.accelX_);
    Serial.print("\t");
    Serial.print(lsm.accelY_);
    Serial.print("\t");
    Serial.print(lsm.accelZ_);
    Serial.println("\t counts");*/
}

/*
 * Print hmc compass data
 *
 * in  :
 * out :
 */
void printMagneto()
{
    if(!skipCompass)
    {
      /*Serial.print(compass.scaledMagnX_);
        Serial.print("\t");
        Serial.print(compass.scaledMagnY_);
        Serial.print("\t");
        Serial.print(compass.scaledMagnZ_);
        Serial.println("\t Gauss");*/
    
        if(!skipHeading)
        {
            Serial.print(compass.heading_);
            Serial.println("\t deg");
        }
        // else no new data to print
    }
    // else no new data to print
}

/*
 * Print everything
 *
 * in  :
 * out :
 */
void printAll()
{
    //printGyro();
    //printAccelero();
    //printMagneto();
    Serial.println();
}

/*
 * Print attitude
 *
 * in  :
 * out :
 */
void printAttitude()
{
    /*float q0Deg = attitude.q0_ * 180.0/PI;
    float q1Deg = attitude.q1_ * 180.0/PI;
    float q2Deg = attitude.q2_ * 180.0/PI;
    float q3Deg = attitude.q3_ * 180.0/PI;

    Serial.print(q0Deg);
    Serial.print("\t");
    Serial.print(q1Deg);
    Serial.print("\t");
    Serial.print(q2Deg);
    Serial.print("\t");
    Serial.print(q3Deg);
    Serial.println("\t deg");*/

    float roll  = attitude.getRoll()  * 180.0/PI;
    float pitch = attitude.getPitch() * 180.0/PI;
    float yaw   = attitude.getYaw()   * 180.0/PI;

    Serial.print(roll);
    Serial.print("\t");
    Serial.print(pitch);
    Serial.print("\t");
    Serial.print(yaw);
    Serial.println("\t deg");
}
