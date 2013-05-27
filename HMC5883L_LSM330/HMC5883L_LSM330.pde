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

#include <Wire.h>
#include <SPI.h>
#include "lsm330.h"
#include "lsm330a_config.h"
#include "lsm330g_config.h"
#include "hmc5883l.h"

// pin definitions
const int chipSelectGyro = 6;
const int chipSelectAccel = 10;

// attitude (quaternion)
float q0 = 0.45; float q1 = 0.0; float q2 = 0.89; float q3 = 0.0; // initialized at raw = pitch = yaw = 0
unsigned long tBegin = 0; unsigned long tEnd = 0;

// gyro readings
int16_t gx = 0; int16_t gy = 0; int16_t gz = 0;
float scaledGx = 0.0; float scaledGy = 0.0; float scaledGz = 0.0;
float gOffsetX = 0.0; float gOffsetY = 0.0; float gOffsetZ = 0.0;
float gThresholdX = 0.0; float gThresholdY = 0.0; float gThresholdZ = 0.0;

// accelerometer readings
int16_t ax = 0; int16_t ay = 0; int16_t az = 0;
float scaledAx = 0.0; float scaledAy = 0.0; float scaledAz = 0.0;

HMC5883L compass;

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

    // Configure LSM330 accelerometer and gyroscope
    setupLSM330a();
    setupLSM330g();

    compass.setup(1.3);

    // Don't move the device for 5 seconds
    calculateGyroOffsets();
    calculateGyroThresholds();

    qInit(0,0,0);
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

    // This will update gx, gy, and gz with new values
    getGyroValues();

    float deltaGx = (float)gx - gOffsetX;
    float deltaGy = (float)gy - gOffsetY;
    float deltaGz = (float)gz - gOffsetZ;

    // scale gyro values
    if(fabs(deltaGx) < gThresholdX)
    {
        scaledGx = 0;
    }
    else
    {
        scaledGx = deltaGx * GYRO_SENSITIVITY;
    }
    if(fabs(deltaGy) < gThresholdY)
    {
        scaledGy = 0;
    }
    else
    {
        scaledGy = deltaGy * GYRO_SENSITIVITY;
    }
    if(fabs(deltaGz) < gThresholdZ)
    {
        scaledGz = 0;
    }
    else
    {
        scaledGz = deltaGz * GYRO_SENSITIVITY;
    }

    if(compass.readMagnetism())
    {
        getAccelValues();  // This will update accel x, y, and z with new values

        // Configure accel axis according to your setup. My accel axis were the opposite of the magneto axis
        // My setup doesn't need axis configuration

        scaledAx = (float)ax * ACC_SENSITIVITY;
        scaledAy = (float)ay * ACC_SENSITIVITY;
        scaledAz = (float)az * ACC_SENSITIVITY;

        float rollRadians = asin(scaledAy);
        float pitchRadians = asin(scaledAx);

        if(rollRadians > 0.78 || rollRadians < -0.78 || pitchRadians > 0.78 || pitchRadians < -0.78)
        {
            // We cannot correct for tilt over 44 degrees with this algorithm, if the board is tilted as such, do nothing...
        }
        else
        {
            compass.calcHeading(rollRadians, pitchRadians/*, yawRadians*/);

            printAll();
        }
    }

    tEnd = micros();
    float deltaT = tEnd - tBegin;
    quaternionUpdateWithRates(deltaT);
    //printAttitude();
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
    // Full-scale selection  +/- 2G
    // Normal mode enable
    // SPI serial interface mode selection : 4 wire interface (default)
    writeRegister(chipSelectAccel, CTRL_REG4, ACC_CTRL_REG4_VAL);

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
    writeRegister(chipSelectGyro, CTRL_REG4, GYRO_CTRL_REG4_VAL);

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

/*
 * Print lsm330 gyroscope data
 *
 * in  :
 * out :
 */
void printGyro()
{
    // Print gyro values
    Serial.print(scaledGx);
    Serial.print("\t");
    Serial.print(scaledGy);
    Serial.print("\t");
    Serial.print(scaledGz);
    Serial.println("\t dps");
}

/*
 * Print lsm330 accelerometer data
 *
 * in  :
 * out :
 */
void printAccelero()
{
    Serial.print(scaledAx);
    Serial.print("\t");
    Serial.print(scaledAy);
    Serial.print("\t");
    Serial.print(scaledAz);
    Serial.println("\t g");
}

/*
 * Print hmc compass data
 *
 * in  :
 * out :
 */
void printMagneto()
{
/*  Serial.print(compass.scaledMagnX_);
    Serial.print("\t");
    Serial.print(compass.scaledMagnY_);
    Serial.print("\t");
    Serial.print(compass.scaledMagnZ_);
    Serial.println("\t Gauss");*/

    Serial.print(compass.heading_);
    Serial.println("\t deg");
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
    printMagneto();
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
    /*float q0Deg = q0 * 180.0/PI;
    float q1Deg = q1 * 180.0/PI;
    float q2Deg = q2 * 180.0/PI;
    float q3Deg = q3 * 180.0/PI;

    Serial.print(q0Deg);
    Serial.print("\t");
    Serial.print(q1Deg);
    Serial.print("\t");
    Serial.print(q2Deg);
    Serial.print("\t");
    Serial.print(q3Deg);
    Serial.println("\t deg");*/

    float roll = getQRoll() * 180.0/PI;
    float pitch = getQPitch() * 180.0/PI;
    float yaw = getQYaw() * 180.0/PI;

    Serial.print(roll);
    Serial.print("\t");
    Serial.print(pitch);
    Serial.print("\t");
    Serial.print(yaw);
    Serial.println("\t deg");
}

/*
 * Calculate Gyro offsets for better data reading later
 *
 * in  :
 * out :
 *
 * WARNING: The gyro must not be in movement when calculating the offsets
 */
void calculateGyroOffsets()
{
    int16_t totalGx = 0;
    int16_t totalGy = 0;
    int16_t totalGz = 0;

    int nbIterations = 100;

    for(int i = 0; i < nbIterations; ++i)
    {
        getGyroValues();

        totalGx += gx;
        totalGy += gy;
        totalGz += gz;

        delay(10);
    }

    gOffsetX = (float)totalGx / (float)nbIterations;
    gOffsetY = (float)totalGy / (float)nbIterations;
    gOffsetZ = (float)totalGz / (float)nbIterations;
}

/*
 * Calculate Gyro thresholds for better data reading later
 *
 * in  :
 * out :
 *
 * WARNING: The gyro must not be in movement when calculating the offsets
 */
void calculateGyroThresholds()
{
    int16_t totalGx = 0;
    int16_t totalGy = 0;
    int16_t totalGz = 0;

    int nbIterations = 100;

    for(int i = 0; i < nbIterations; ++i)
    {
        getGyroValues();

        totalGx = fabs((float)gx - (float)gOffsetX);
        totalGy = fabs((float)gy - (float)gOffsetY);
        totalGz = fabs((float)gz - (float)gOffsetZ);

        if(totalGx > gThresholdX)
        {
            gThresholdX = totalGx;
        }
        if(totalGy > gThresholdY)
        {
            gThresholdY = totalGy;
        }
        if(totalGz > gThresholdZ)
        {
            gThresholdZ = totalGz;
        }

        delay(10);
    }
}

/*
 *   Update quaternion with rates.
 *
 *   in:
 *      dt : deltaT
 */
void quaternionUpdateWithRates(unsigned long dt_us)
{
    float dt = (float)dt_us / 1000000.0;

    float w1 = scaledGy * PI/180.0;
    float w2 = scaledGx * PI/180.0;
    float w3 = scaledGz * PI/180.0;

    float oldQ0 = q0;
    float oldQ1 = q1;
    float oldQ2 = q2;
    float oldQ3 = q3;

    q0 += 0.5 * (         - oldQ1*w1 - oldQ2*w2 - oldQ3*w3)*dt;
    q1 += 0.5 * (oldQ0*w1 +            oldQ2*w3 - oldQ3*w2)*dt;
    q2 += 0.5 * (oldQ0*w2 - oldQ1*w3 +            oldQ3*w1)*dt;
    q3 += 0.5 * (oldQ0*w3 + oldQ1*w2 - oldQ2*w1           )*dt;

    float norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);

    q0 /= norm;
    q1 /= norm;
    q2 /= norm;
    q3 /= norm;
}

float getQRoll()
{
    float r;
    r = atan2( 2.0 * ( q2*q3 + q0*q1 ),(1.0 - 2.0 * (q1*q1 + q2*q2)) );
    return r;
}

float getQPitch()
{
    float r;
    r = asinf( -2.0 * (q1*q3 - q0*q2) );
    return r;
}

float getQYaw()
{
    float r;
    r = atan2( 2.0 * ( q0*q3 + q1*q2 ) ,(1.0 - 2.0 * (q2*q2 + q3*q3)) );
    return r;
}

void qInit(float roll,float pitch,float yaw)
{
    float cos_roll_2 = cosf(roll/2.0);
    float sin_roll_2 = sinf(roll/2.0);
    float cos_pitch_2 = cosf(pitch/2.0);
    float sin_pitch_2 = sinf(pitch/2.0);
    float cos_yaw_2 = cosf(yaw/2.0);
    float sin_yaw_2 = sinf(yaw/2.0);


    q0 = cos_roll_2 * cos_pitch_2 * cos_yaw_2 + sin_roll_2 * sin_pitch_2 * sin_yaw_2;
    q1 = sin_roll_2 * cos_pitch_2 * cos_yaw_2 - cos_roll_2 * sin_pitch_2 * sin_yaw_2;
    q2 = cos_roll_2 * sin_pitch_2 * cos_yaw_2 + sin_roll_2 * cos_pitch_2 * sin_yaw_2;
    q3 = cos_roll_2 * cos_pitch_2 * sin_yaw_2 - sin_roll_2 * sin_pitch_2 * cos_yaw_2;


    float norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);

    q0 /= norm;
    q1 /= norm;
    q2 /= norm;
    q3 /= norm;
}
