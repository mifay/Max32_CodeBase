// Copyright 2013 Michael Fayad
//
// This file is part of Max32_CodeBase.
//
// lsm330a.pde is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// lsm330a.pde is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with lsm330a.pde.  If not, see <http://www.gnu.org/licenses/>.

/*
    LSM330 3-axis accelerator SPI with Max32 code


    source : SparkFun Electronics

    Circuit:
    LSM330 Breakout-----------------Max32
    GND-----------------------------J13-GND
    VCC-----------------------------3.3V
    SCL-----------------------------J13-3
    SDA-----------------------------J13-4
    SDO-----------------------------J13-1 (first pin)
    CS------------------------------10
    INT2----------------------------6

    This code sets up the LSM330's 5 control registers, and then
    streams the data from all three axes over the Serial Monitor at 115200bps.
*/
// Includes
#include <SPI.h>
#include "lsm330a.h"

// Accelerometer scale values
#define SCALE_FOR_2G 0.0001
#define SCALE_FOR_4G 0.0002
#define SCALE_FOR_8G 0.0004
#define SCALE_FOR_16G 0.0012

// pin definitions
const int int2pin = 6;
const int chipSelect = 10;

// accel readings
int16_t x, y, z;

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

    // Set pin values
    pinMode(int2pin, INPUT);
    pinMode(chipSelect, OUTPUT);
    digitalWrite(chipSelect, HIGH);

    //delay(100); //TBR: seems useless

    // Configure LSM330
    setupLSM330();
}

///////////////////////////////////////////////////////////
// Max32 loop function
///////////////////////////////////////////////////////////
void loop()
{
    // Don't read accel values until the accel says it's ready
    while(!digitalRead(int2pin));

    getAccValues();  // This will update x, y, and z with new values

    // scale accelerometer values
    x *= SCALE_FOR_16G;
    y *= SCALE_FOR_16G;
    z *= SCALE_FOR_16G;
    
    // Display raw accelaration values
    Serial.print(x, DEC);
    Serial.print("\t");
    Serial.print(y, DEC);
    Serial.print("\t");
    Serial.print(z, DEC);
    Serial.print("\t");
    Serial.println();
}

///////////////////////////////////////////////////////////
// Register reading function
//
// In   : address - The register address to read
//
// Out  : toRead - The value read at the register address
///////////////////////////////////////////////////////////
int readRegister(byte address)
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
void writeRegister(byte address, byte data)
{
    // This address is to tell the LSM330 that we're writing
    address &= 0x7F;

    digitalWrite(chipSelect, LOW);
    SPI.transfer(address);
    SPI.transfer(data);
    digitalWrite(chipSelect, HIGH);
}


///////////////////////////////////////////////////////////
// LSM330 setup function
///////////////////////////////////////////////////////////
void setupLSM330()
{
    // Check the datasheet (p29)
    
    // Normal (1.344 kHz) / low-power mode (5.376 kHz) data rate
    // Normal mode selected (default)
    // x,y,z axis enabled (default)
    writeRegister(CTRL_REG1, 0b10010111);
    
    // High-pass filter mode selection : Normal mode (reset reading HP_RESET_FILTER) (default)
    // High-pass filter cutoff frequency selection
    // Filtered data selection : internal filter bypassed (default)
    // High-pass filter enabled for CLICK function : filter bypassed
    // High-pass filter enabled for AOI function on interrupt 2 : filter bypassed
    // High-pass filter enabled for AOI function on interrupt 1 : filter bypassed
    writeRegister(CTRL_REG2, 0b00000000);
    
    // CLICK interrupt on INT1_A disabled (default)
    // AOI1 interrupt on INT1_A disabled (default)
    // DRDY1 interrupt on INT1_A disabled (default)
    // DRDY2 interrupt on INT1_A disabled (default)
    // FIFO watermark interrupt on INT1_A enabled
    // FIFO overrun interrupt on INT1_A disabled (default)
    writeRegister(CTRL_REG3, 0b00001000);
    
    // Continuous block data update (default)
    // Big/little endian data selection : Data LSB at lower address (default)
    // Full-scale selection  +/- 16G
    // Normal mode enable
    // SPI serial interface mode selection : 4 wire interface (default)
    writeRegister(CTRL_REG4, 0b00111000);
    
    // normal mode, no reboot memory content (default)
    // FIFO enabled
    // interrupt request not latched (default)
    // 4D detection disabled
    writeRegister(CTRL_REG5, 0b01000000);
}

///////////////////////////////////////////////////////////
// Get accelerometer values function
///////////////////////////////////////////////////////////
void getAccValues()
{
    x = (readRegister(0x29)&0xFF)<<8;
    x |= (readRegister(0x28)&0xFF);

    y = (readRegister(0x2B)&0xFF)<<8;
    y |= (readRegister(0x2A)&0xFF);

    z = (readRegister(0x2D)&0xFF)<<8;
    z |= (readRegister(0x2C)&0xFF);
}