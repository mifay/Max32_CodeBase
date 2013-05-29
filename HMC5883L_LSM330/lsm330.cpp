// Copyright 2013 Michael Fayad
//
// This file is part of Max32_CodeBase.
//
// lsm330.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// lsm330.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with lsm330.cpp.  If not, see <http://www.gnu.org/licenses/>.
//
//
// source :      - Jordan McConnell, SparkFun Electronics

#include "lsm330.h"
#include <SPI.h>

/*
 * Setup lsm330 spi device
 *
 * in  :
 * out :
 */
void LSM330::setup(int chipSelectGyro, int gyroSensitivity, int chipSelectAccel, int accelSensitivity)
{   
    this->setupAccelerometer(chipSelectAccel, accelSensitivity);
    this->setupGyroscope(chipSelectGyro, gyroSensitivity);
}

/*
 * Setup lsm330 accelerometer (spi communication)
 *
 * in  :
 * out :
 */
void LSM330::setupAccelerometer(int chipSelectAccel, int accelSensitivity)
{   
    this->setAccelSensitivity(accelSensitivity);
    
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
    // Normal mode enable
    // SPI serial interface mode selection : 4 wire interface (default)
    writeRegister(chipSelectAccel, CTRL_REG4, this->accelCtrlReg4Val_);

    // normal mode, no reboot memory content (default)
    // FIFO enabled
    // interrupt request not latched (default)
    // 4D detection disabled
    writeRegister(chipSelectAccel, CTRL_REG5, 0b01000000);
}

/*
 * Setup lsm330 gyroscope (spi communication)
 *
 * in  :
 * out :
 */
void LSM330::setupGyroscope(int chipSelectGyro, int gyroSensitivity)
{   
    this->setGyroSensitivity(gyroSensitivity);
    
    //////////////////////////////////////////////////////////////////
    // Check the LSM330 datasheet to understand more clearly my setup
    //////////////////////////////////////////////////////////////////

    // Enable x, y, z and turn off power down:
    writeRegister(chipSelectGyro, CTRL_REG1, 0b00001111);

    // If you'd like to adjust/use the HPF, you can edit the line below to configure CTRL_REG2:
    writeRegister(chipSelectGyro, CTRL_REG2, 0b00000000);

    // Configure CTRL_REG3 to generate data ready interrupt on INT2
    // No interrupts used on INT1, if you'd like to configure INT1
    // or INT2 otherwise, consult the datasheet:
    writeRegister(chipSelectGyro, CTRL_REG3, 0b00001000);

    // CTRL_REG4 controls the full-scale range, among other things:
    writeRegister(chipSelectGyro, CTRL_REG4, this->gyroCtrlReg4Val_);

    // CTRL_REG5 controls high-pass filtering of outputs, use it
    // if you'd like:
    writeRegister(chipSelectGyro, CTRL_REG5, 0b00000000); 
        
    this->calculateGyroOffsets(chipSelectGyro);
    this->calculateGyroThresholds(chipSelectGyro);    
}

/*
 * Set lsm330 gyroscope sensitivity
 *
 * in  : sensitivity (in dps)
 *
 * out :
 *
 * N.B.: Must only be called in gyro setup function
 */
void LSM330::setGyroSensitivity(int sensitivity)
{
    if(sensitivity == 500)
    {
        this->gyroSensitivity_ = sensitivity;
        this->gyroScale_ = 0.0175; // from the datasheet
        this->gyroCtrlReg4Val_ = 0x10;
    }
    else if(sensitivity == 2000)
    {
        this->gyroSensitivity_ = sensitivity;
        this->gyroScale_ = 0.070; // from the datasheet
        this->gyroCtrlReg4Val_ = 0x20;      
    }
    else
    {
        // default value : 250 dps
        this->gyroSensitivity_ = 250;
        this->gyroScale_ = 0.00875; // from the datasheet
        this->gyroCtrlReg4Val_ = 0x00;        
    }
}

/*
 * Set lsm330 accelerometer sensitivity
 *
 * in  : sensitivity (in g)
 *
 * out :
 *
 * N.B.: Must only be called in accelerometer setup function
 */
void LSM330::setAccelSensitivity(int sensitivity)
{
    if(sensitivity == 4)
    {
        this->accelSensitivity_ = sensitivity;
        this->accelScale_ = 0.002; // from the datasheet
        this->accelCtrlReg4Val_ = 0b00011000;
    }
    else if(sensitivity == 8)
    {
        this->accelSensitivity_ = sensitivity;
        this->accelScale_ = 0.004; // from the datasheet
        this->accelCtrlReg4Val_ = 0b00101000;      
    }
    else if(sensitivity == 16)
    {
        this->accelSensitivity_ = sensitivity;
        this->accelScale_ = 0.00078; // manually calibrated
        this->accelCtrlReg4Val_ = 0b00111000;      
    }    
    else
    {
        // default value : 2 g
        this->accelSensitivity_ = 2;
        this->accelScale_ = 0.0000637; // manually calibrated
        this->accelCtrlReg4Val_ = 0b00001000;        
    }
}

///////////////////////////////////////////////////////////
// Register writing function (spi communication)
//
// in   : chipSelect - pin used to select the chip
//        address    - The register address to write
//        data       - The data to write at the register address
///////////////////////////////////////////////////////////
void LSM330::writeRegister(int chipSelect, int address, int data)
{
    // This address is to tell the LSM330 that we're writing
    address &= 0x7F;

    digitalWrite(chipSelect, LOW);
    SPI.transfer(address);
    SPI.transfer(data);
    digitalWrite(chipSelect, HIGH);
}

///////////////////////////////////////////////////////////
// register reading function
//
// In   : address - The register address to read
//
// Out  : toRead - The value read at the register address
///////////////////////////////////////////////////////////
int LSM330::readRegister(int chipSelect, int address)
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

/*
 * Calculate Gyro offsets for better data reading later
 *
 * in  :
 * out :
 *
 * WARNING: The gyro must not be in movement when calculating the offsets
 */
void LSM330::calculateGyroOffsets(int chipSelectGyro)
{
    int16_t totalGx = 0;
    int16_t totalGy = 0;
    int16_t totalGz = 0;

    int nbIterations = 100;

    for(int i = 0; i < nbIterations; ++i)
    {
        getGyroValues(chipSelectGyro);

        totalGx += gyroX_;
        totalGy += gyroY_;
        totalGz += gyroZ_;

        delay(10);
    }

    gyroOffsetX_ = (float)totalGx / (float)nbIterations;
    gyroOffsetY_ = (float)totalGy / (float)nbIterations;
    gyroOffsetZ_ = (float)totalGz / (float)nbIterations;
}

/*
 * Calculate Gyro thresholds for better data reading later
 *
 * in  :
 * out :
 *
 * WARNING: The gyro must not be in movement when calculating the offsets
 */
void LSM330::calculateGyroThresholds(int chipSelectGyro)
{
    int16_t totalGx = 0;
    int16_t totalGy = 0;
    int16_t totalGz = 0;

    int nbIterations = 100;

    for(int i = 0; i < nbIterations; ++i)
    {
        getGyroValues(chipSelectGyro);

        totalGx = fabs((float)gyroX_ - (float)gyroOffsetX_);
        totalGy = fabs((float)gyroY_ - (float)gyroOffsetY_);
        totalGz = fabs((float)gyroZ_ - (float)gyroOffsetZ_);

        if(totalGx > gyroThresholdX_)
        {
            gyroThresholdX_ = totalGx;
        }
        if(totalGy > gyroThresholdY_)
        {
            gyroThresholdY_ = totalGy;
        }
        if(totalGz > gyroThresholdZ_)
        {
            gyroThresholdZ_ = totalGz;
        }

        delay(10);
    }
}

///////////////////////////////////////////////////////////
// Get gyroscope values function
///////////////////////////////////////////////////////////
void LSM330::getGyroValues(int chipSelectGyro)
{
    gyroX_ = (readRegister(chipSelectGyro, 0x29)&0xFF)<<8;
    gyroX_ |= (readRegister(chipSelectGyro, 0x28)&0xFF);

    gyroY_ = (readRegister(chipSelectGyro, 0x2B)&0xFF)<<8;
    gyroY_ |= (readRegister(chipSelectGyro, 0x2A)&0xFF);

    gyroZ_ = (readRegister(chipSelectGyro, 0x2D)&0xFF)<<8;
    gyroZ_ |= (readRegister(chipSelectGyro, 0x2C)&0xFF);
    
    float deltaGx = (float)gyroX_ - (float)gyroOffsetX_;
    float deltaGy = (float)gyroY_ - (float)gyroOffsetY_;
    float deltaGz = (float)gyroZ_ - (float)gyroOffsetZ_;

    // scale gyro values
    if(fabs(deltaGx) < gyroThresholdX_)
    {
        scaledGyroX_ = 0;
    }
    else
    {
        scaledGyroY_ = deltaGx * this->gyroScale_;
    }
    if(fabs(deltaGy) < gyroThresholdY_)
    {
        scaledGyroY_ = 0;
    }
    else
    {
        scaledGyroY_ = deltaGy * this->gyroScale_;
    }
    if(fabs(deltaGz) < gyroThresholdZ_)
    {
        scaledGyroZ_ = 0;
    }
    else
    {
        scaledGyroZ_ = deltaGz * this->gyroScale_;
    }    
}

///////////////////////////////////////////////////////////
// Get accelerometer values function
///////////////////////////////////////////////////////////
void LSM330::getAccelValues(int chipSelectAccel)
{
    this->accelX_ = (readRegister(chipSelectAccel, 0x29)&0xFF)<<8;
    this->accelX_ |= (readRegister(chipSelectAccel, 0x28)&0xFF);

    this->accelY_ = (readRegister(chipSelectAccel, 0x2B)&0xFF)<<8;
    this->accelY_ |= (readRegister(chipSelectAccel, 0x2A)&0xFF);

    this->accelZ_ = (readRegister(chipSelectAccel, 0x2D)&0xFF)<<8;
    this->accelZ_ |= (readRegister(chipSelectAccel, 0x2C)&0xFF);
    
    this->scaledAccelX_ = (float)accelX_ * this->accelScale_;
    this->scaledAccelY_ = (float)accelY_ * this->accelScale_;
    this->scaledAccelZ_ = (float)accelZ_ * this->accelScale_;    
}
