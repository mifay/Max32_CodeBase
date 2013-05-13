// Copyright 2013 Michael Fayad
//
// This file is part of Max32_CodeBase.
//
// lsm330.pde is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// lsm330.pde is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with lsm330.pde.  If not, see <http://www.gnu.org/licenses/>.

/* 
    LSM330 3-axis gyro and accelerometer SPI with Max32 code


    source : SparkFun Electronics

    Circuit:
    LSM330 Breakout-----------------Max32
    GND-----------------------------J13-GND
    VCC-----------------------------3.3V
    SCL-----------------------------J13-3
    SDA-----------------------------J13-4
    SDO-----------------------------J13-1 (first pin)
    CSGyro--------------------------6
    CSAccel-------------------------10

    This code sets up the LSM330's 5 control registers, and then 
    streams the data from all three axes over the Serial Monitor at 115200bps.
*/
// Includes
#include <SPI.h>
#include "lsm330.h"

// Accelerometer scale values
#define SCALE_FOR_2G 0.0000637 // in g/counts (calibrated)
#define SCALE_FOR_4G 0.0002 // in g/counts (not calibrated)
#define SCALE_FOR_8G 0.0004 // in g/counts (not calibrated)
#define SCALE_FOR_16G 0.0012 // in g/counts (not calibrated)

// Gyro scale values
#define SCALE_FOR_250_DPS 0.000875
#define SCALE_FOR_500_DPS 0.00175
#define SCALE_FOR_2000_DPS 0.0070

// pin definitions
const int chipSelectGyro = 6;
const int chipSelectAccel = 10;

// gyro readings
int16_t gx, gy, gz;

// accelerometer readings
int16_t ax, ay, az;

///////////////////////////////////////////////////////////
// Setup function. Launched initially by the Max32
///////////////////////////////////////////////////////////
void setup()
{
    Serial.begin(115200);

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

    // Configure LSM330 accelerometer and gyroscope
    setupLSM330a();
    setupLSM330g();
}

///////////////////////////////////////////////////////////
// Max32 loop function
///////////////////////////////////////////////////////////
void loop()
{
    // This will update gx, gy, and gz with new values
    getGyroValues();
    
    // scale gyro values
    float scaledGx = gx * SCALE_FOR_2000_DPS;
    float scaledGy = gy * SCALE_FOR_2000_DPS;
    float scaledGz = gz * SCALE_FOR_2000_DPS;

    // Print gyro values
    Serial.print("g : ");
    Serial.print(scaledGx);
    Serial.print("\t");
    Serial.print(scaledGy);
    Serial.print("\t");
    Serial.print(scaledGz);
    Serial.print("\t");
    Serial.println();

    // This will update ax, ay, and az with new values
    getAccelValues();
    
    // scale accelerometer values
    float scaledAx = ax * SCALE_FOR_16G;
    float scaledAy = ay * SCALE_FOR_16G;
    float scaledAz = az * SCALE_FOR_16G;    

    // Print accel values
    Serial.print("a : ");
    Serial.print(scaledAx);
    Serial.print("\t");
    Serial.print(scaledAy);
    Serial.print("\t");
    Serial.print(scaledAz);
    Serial.print("\t");
    Serial.println();  
    
    // TBR : added for debug purpose
    delay(750);
}

///////////////////////////////////////////////////////////
// register reading function
//
// In   : address - The register address to read
//
// Out  : toRead - The value read at the register address
///////////////////////////////////////////////////////////
int readRegister(int chipSelect, byte address)
{
    int toRead;

    // This address is to tell the LSM330 that we're reading
    address |= 0x80;

    digitalWrite(chipSelect, LOW);
    SPI.transfer(address);
    toRead = SPI.transfer(0x00);
    digitalWrite(chipSelect, HIGH);

    return toRead;
}

///////////////////////////////////////////////////////////
// Register writing function
//
// In   : address - The register address to write
//        data    - The data to write at the register address
///////////////////////////////////////////////////////////
void writeRegister(int chipSelect, byte address, byte data)
{
    // This address is to tell the LSM330 that we're writing
    address &= 0x7F;

    digitalWrite(chipSelect, LOW);
    SPI.transfer(address);
    SPI.transfer(data);
    digitalWrite(chipSelect, HIGH);
}


///////////////////////////////////////////////////////////
// LSM330 accelerometer setup function
///////////////////////////////////////////////////////////
void setupLSM330a()
{    
    // Check the datasheet (p29)
    
    // Normal (1.344 kHz) / low-power mode (5.376 kHz) data rate
    // Normal mode selected (default)
    // x,y,z axis enabled (default)
    writeRegister(chipSelectAccel, CTRL_REG1, 0b10010111);

    // High-pass filter mode selection : Normal mode (reset reading HP_RESET_FILTER) (default)
    // High-pass filter cutoff frequency selection
    // Filtered data selection : internal filter bypassed (default)
    // High-pass filter enabled for CLICK function : filter bypassed
    // High-pass filter enabled for AOI function on interrupt 2 : filter bypassed
    // High-pass filter enabled for AOI function on interrupt 1 : filter bypassed
    writeRegister(chipSelectAccel, CTRL_REG2, 0b00000000);

    // CLICK interrupt on INT1_A disabled (default)
    // AOI1 interrupt on INT1_A disabled (default)
    // DRDY1 interrupt on INT1_A disabled (default)
    // DRDY2 interrupt on INT1_A disabled (default)
    // FIFO watermark interrupt on INT1_A enabled
    // FIFO overrun interrupt on INT1_A disabled (default)
    writeRegister(chipSelectAccel, CTRL_REG3, 0b00001000);

    // Continuous block data update (default)
    // Big/little endian data selection : Data LSB at lower address (default)
    // Full-scale selection  +/- 16G
    // Normal mode enable
    // SPI serial interface mode selection : 4 wire interface (default)
    writeRegister(chipSelectAccel, CTRL_REG4, 0b00111000);

    // normal mode, no reboot memory content (default)
    // FIFO enabled
    // interrupt request not latched (default)
    // 4D detection disabled
    writeRegister(chipSelectAccel, CTRL_REG5, 0b01000000);
}

///////////////////////////////////////////////////////////
// LSM330 gyro setup function
///////////////////////////////////////////////////////////
void setupLSM330g()
{    
    //////////////////////////////////
    // Check the LSM330 datasheet to setup the registers properly
    //////////////////////////////////
    
    // Enable x, y, z and turn off power down:
    writeRegister(chipSelectGyro, CTRL_REG1, 0b00001111);

    // If you'd like to adjust/use the HPF, you can edit the line below to configure CTRL_REG2:
    writeRegister(chipSelectGyro, CTRL_REG2, 0b00000000);

    // Configure CTRL_REG3 to generate data ready interrupt on INT2
    // No interrupts used on INT1, if you'd like to configure INT1
    // or INT2 otherwise, consult the datasheet:
    writeRegister(chipSelectGyro, CTRL_REG3, 0b00001000);

    // CTRL_REG4 controls the full-scale range, among other things:
    int fullScale = 2; // 2000dps
    fullScale &= 0x03;
    writeRegister(chipSelectGyro, CTRL_REG4, fullScale<<4);

    // CTRL_REG5 controls high-pass filtering of outputs, use it
    // if you'd like:
    writeRegister(chipSelectGyro, CTRL_REG5, 0b00000000);
}

///////////////////////////////////////////////////////////
// Get gyroscope values function
///////////////////////////////////////////////////////////
void getGyroValues()
{
    gx = (readRegister(chipSelectGyro, 0x29)&0xFF)<<8;
    gx |= (readRegister(chipSelectGyro, 0x28)&0xFF);

    gy = (readRegister(chipSelectGyro, 0x2B)&0xFF)<<8;
    gy |= (readRegister(chipSelectGyro, 0x2A)&0xFF);

    gz = (readRegister(chipSelectGyro, 0x2D)&0xFF)<<8;
    gz |= (readRegister(chipSelectGyro, 0x2C)&0xFF);
}

///////////////////////////////////////////////////////////
// Get accelerometer values function
///////////////////////////////////////////////////////////
void getAccelValues()
{
    ax = (readRegister(chipSelectAccel, 0x29)&0xFF)<<8;
    ax |= (readRegister(chipSelectAccel, 0x28)&0xFF);

    ay = (readRegister(chipSelectAccel, 0x2B)&0xFF)<<8;
    ay |= (readRegister(chipSelectAccel, 0x2A)&0xFF);

    az = (readRegister(chipSelectAccel, 0x2D)&0xFF)<<8;
    az |= (readRegister(chipSelectAccel, 0x2C)&0xFF);
}