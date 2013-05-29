/* 
// Copyright 2013 Michael Fayad
//
// This file is part of Max32_CodeBase.
//
// lsm330.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// lsm330.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with lsm330.h.  If not, see <http://www.gnu.org/licenses/>.

// source :      - Jordan McConnell, SparkFun Electronics
*/

#include <stdint.h> // to use int16_t type to avoid sign extension problems with read output data

/***************************************************
    LSM330 Registers for gyro and accelerometer
****************************************************/
#define WHO_AM_I 0x0F
#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24
#define CTRL_REG6 0x25
#define REFERENCE_A 0x26
#define REFERENCE 0x25
#define OUT_TEMP 0x26
#define STATUS_REG 0x27
#define OUT_X_L 0x28
#define OUT_X_H 0x29
#define OUT_Y_L 0x2A
#define OUT_Y_H 0x2B
#define OUT_Z_L 0x2C
#define OUT_Z_H 0x2D
#define FIFO_CTRL_REG 0x2E
#define FIFO_SRC_REG 0x2F
#define INT1_CFG 0x30
#define INT1_SRC 0x31
#define INT1_TSH_XH 0x32
#define INT1_TSH_XL 0x33
#define INT1_TSH_YH 0x34
#define INT1_TSH_YL 0x35
#define INT1_TSH_ZH 0x36
#define INT1_TSH_ZL 0x37
#define INT1_DURATION 0x38

class LSM330
{
    public:
        // setup methods
        void setup(int chipSelectGyro, int gyroSensitivity, int chipSelectAccel, int accelSensitivity);
        void setupAccelerometer(int chipSelectAccel, int accelSensitivity);
        void setupGyroscope(int chipSelectGyro, int gyroSensitivity);
        
        void getGyroValues(int chipSelectGyro);
        
        void getAccelValues(int chipSelectAccel);
        
        // Public gyro values
        int16_t gyroX_; // in counts
        int16_t gyroY_; // in counts
        int16_t gyroZ_; // in counts
        float scaledGyroX_; // in dps
        float scaledGyroY_; // in dps
        float scaledGyroZ_; // in dps
        
        // Public accelerometer values
        int16_t accelX_; // in counts
        int16_t accelY_; // in counts
        int16_t accelZ_; // in counts
        float scaledAccelX_; // in g
        float scaledAccelY_; // in g
        float scaledAccelZ_; // in g        

    private:
        void writeRegister(int chipSelect, int address, int data);
        int readRegister(int chipSelect, int address);
        
        // Accelerometer private methods
        void setAccelSensitivity(int sensitivity); 
        
        // Gyro private methods
        void setGyroSensitivity(int sensitivity);        
        void calculateGyroOffsets(int chipSelectGyro);
        void calculateGyroThresholds(int chipSelectGyro);
        
        // Gyro attributes
        int gyroSensitivity_; // Gyro sensitivity and accuracy (in dps)
        float gyroScale_; // Gyro multiplicator scale factor (in dps/count)
        int gyroCtrlReg4Val_; // Gyro control register 4 value according to set sensitivity
        float gyroOffsetX_; // in counts
        float gyroOffsetY_; // in counts
        float gyroOffsetZ_; // in counts    
        float gyroThresholdX_; // in counts    
        float gyroThresholdY_; // in counts
        float gyroThresholdZ_; // in counts   
   
        // Accel attributes
        int accelSensitivity_; // Accelerometer sensitivity and accuracy (in g)
        float accelScale_; // Accelerometer multiplicator scale factor (in g/count)
        int accelCtrlReg4Val_; // Accelerometer control register 4 value according to set sensitivity        
};
