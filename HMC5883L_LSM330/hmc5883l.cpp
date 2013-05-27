// Copyright 2013 Michael Fayad
//
// This file is part of Max32_CodeBase.
//
// hmc5883l.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// hmc5883l.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with hmc5883l.cpp.  If not, see <http://www.gnu.org/licenses/>.

#include "hmc5883l.h"
#include <Wire.h> // For i2c communication
#include <math.h> // To use M_PI constant
#include <errno.h> // To get errno after atan2 operations

/*
 * Setup HMC5883L i2c compass
 *
 * in  :
 * out :
 */
void HMC5883L::setup(float gain)
{   
    this->setGain(gain);
  
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

    // Set magnetism perceptibility and accuracy.
    Wire.beginTransmission(HMC_ADDRESS);
    Wire.send(CONFIG_REG_B);
    Wire.send(this->gainRegVal_);
    Wire.endTransmission();
}

void HMC5883L::setGain(float gain)
{
    if(gain == 8.1)
    {
        this->gain_ = gain;
        this->scale_ = 4.347826; // from the datasheet (in mG/counts)
        this->gainRegVal_ = 0xE0;
    }
    else
    {
        // Set to device's default according to the datasheet
        this->gain_ = 1.3;
        this->scale_ = 0.92; // from the datasheet (in mG/counts)
        this->gainRegVal_ = 0x20;
    }
}

bool HMC5883L::readMagnetism()
{
    //Tell the HMC5883 where to begin reading data
    Wire.beginTransmission(HMC_ADDRESS);
    Wire.send(DATA_REG_BEGIN); //select register 3, X MSB register
    Wire.endTransmission();
    
    //Read data from each axis, 2 registers per axis
    Wire.requestFrom(HMC_ADDRESS, 6);
    if(6<=Wire.available())
    {
        int16_t mx; int16_t my; int16_t mz;
        
        mx = Wire.receive()<<8;  //X msb
        mx |= Wire.receive();    //X lsb
        mz = Wire.receive()<<8;  //Z msb
        mz |= Wire.receive();    //Z lsb
        my = Wire.receive()<<8;  //Y msb
        my |= Wire.receive();    //Y lsb
        
        this->magnX_ = mx;
        this->magnY_ = my;
        this->magnZ_ = mz;

        this->scaledMagnX_ = this->scale_ * (float)mx;
        this->scaledMagnY_ = this->scale_ * (float)my;
        this->scaledMagnZ_ = this->scale_ * (float)mz;    
        
        return true;
    }
    else
    {
        // No data available
        return false;
    }
}

float HMC5883L::calcHeading(float rollRadians, float pitchRadians/*, float yawRadians*/)
{
    float cosRoll = cos(rollRadians);
    float sinRoll = sin(rollRadians);
    float cosPitch = cos(pitchRadians);
    float sinPitch = sin(pitchRadians);

    float Xh = this->scaledMagnX_ * cosPitch + this->scaledMagnZ_ * sinPitch;
    float Yh = this->scaledMagnX_ * sinRoll * sinPitch + this->scaledMagnY_ * cosRoll - this->scaledMagnZ_ * sinRoll * cosPitch;

    errno = 0;
    this->heading_ = atan2(Yh, Xh);
    if (errno)
    {
        return  0.0;
    }
    else
    {
        // Correct for when signs are reversed.
        if(this->heading_ < 0)
        {
            this->heading_ += 2.0 * M_PI;
        }
    
        // Check for wrap due to addition of declination.
        if(this->heading_ > 2.0 * M_PI)
        {
            this->heading_ -= 2.0 * M_PI;
        }
    
        // Radians to degrees
        this->heading_ = this->heading_ * 180.0 / M_PI;
        
        return this->heading_;
    }
}
