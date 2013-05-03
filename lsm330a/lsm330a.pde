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
  // Check the datasheet (p29) - default register values
  writeRegister(CTRL_REG1, 0b10010111);
  writeRegister(CTRL_REG2, 0b00000000);
  writeRegister(CTRL_REG3, 0b00001000);
  writeRegister(CTRL_REG4, 0b00111000);
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

