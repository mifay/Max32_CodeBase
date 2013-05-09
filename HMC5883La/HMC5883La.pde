// Copyright 2013 Michael Fayad
//
// This file is part of Max32_CodeBase.
//
// HMC5883La.pde is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// HMC5883La.pde is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with HMC5883La.pde.  If not, see <http://www.gnu.org/licenses/>.

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
 *
 * LSM330   ->                   Max32
 * 3v3      ->                   3V3
 * GND      ->                   GND
 * SDO      ->                   SDI(J13-1)(first pin)
 * SCL      ->                   SCL(J13-3)
 * INT2     ->                   (6)
 * CS       ->                   (10)
 * SDI      ->                   SDO(J13-4)
 */

#include <Wire.h>
#include <SPI.h>
#include "lsm330a.h"

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

// Accelerometer scale values
#define SCALE_FOR_2G 0.0001
#define SCALE_FOR_4G 0.0002
#define SCALE_FOR_8G 0.0004
#define SCALE_FOR_16G 0.0012

// accelerometer pin definitions
const int int2pin = 6;
const int chipSelect = 10;

// accelerometer readings
int16_t ax, ay, az;

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
    
    // Set pin values for the SPI communication with the accelerometer
    pinMode(int2pin, INPUT);
    pinMode(chipSelect, OUTPUT);
    digitalWrite(chipSelect, HIGH);
    
    // Configure LSM330
    setupLSM330();    

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
    int16_t x,y,z; //triple axis data

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
        float scaledY = GAIN_VALUE_SCALE * (float)y;
        float scaledX = GAIN_VALUE_SCALE * (float)x;
        float scaledZ = GAIN_VALUE_SCALE * (float)z;
        float heading = atan2(scaledY, scaledX);
        
        // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
        // Find yours here: http://www.magnetic-declination.com/
        // Mine is: 2� 37' W, which is 2.617 Degrees, or (which we need) 0.0456752665 radians, I will use 0.0457
        // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
        float declinationAngle = -0.2578;
        heading += declinationAngle;
        
        // Correct for when signs are reversed.
        if(heading < 0)
            heading += 2.0*PI;

        // Check for wrap due to addition of declination.
        if(heading > 2*PI)
            heading -= 2.0*PI;

        // Convert radians to degrees for readability.
        float headingDegrees = heading * 180.0/PI;     

        getAccValues();  // This will update accel x, y, and z with new values
      
        // Configure accel axis according to your setup. My accel axis were the opposite of the magneto axis
        ax = -ax;
        ay = -ay;    
        
        float scaledAx = (float)ax * SCALE_FOR_2G;
        float scaledAy = (float)ay * SCALE_FOR_2G;
        float scaledAz = (float)az * SCALE_FOR_2G;
        
        float rollRadians = asin(scaledAy);
        float pitchRadians = asin(scaledAx);       

        // We cannot correct for tilt over 44 degrees with this algorithm, if the board is tilted as such, do nothing...       
        if(rollRadians > 0.78 || rollRadians < -0.78 || pitchRadians > 0.78 || pitchRadians < -0.78)
        {
            // do nothing and print anyway
            //Print out values of each axis.
            Serial.print("mx: ");
            Serial.print(x);
            Serial.print("  my: ");
            Serial.print(y);
            Serial.print("  mz: ");
            Serial.print(z);
            Serial.print("  heading: ");
            Serial.println(headingDegrees);
            Serial.print("  ax: ");
            Serial.print(ax);
            Serial.print("  ay: ");
            Serial.print(ay);
            Serial.print("  az: ");
            Serial.print(az);
            Serial.print("  heading2: ");
            Serial.println(0);
            Serial.println();   
        }
        else
        {
            // Compensate magneto tilt with accelero values
            float cosRoll = cos(rollRadians);
            float sinRoll = sin(rollRadians);  
            float cosPitch = cos(pitchRadians);
            float sinPitch = sin(pitchRadians);
            
            float Xh = scaledX * cosPitch + scaledZ * sinPitch;
            float Yh = scaledX * sinRoll * sinPitch + scaledY * cosRoll - scaledZ * sinRoll * cosPitch;

            float heading2 = atan2(Yh, Xh);
            
            heading2 += declinationAngle;
            
            // Correct for when signs are reversed.
            if(heading2 < 0)
                heading2 += 2.0*PI;
    
            // Check for wrap due to addition of declination.
            if(heading2 > 2*PI)
                heading2 -= 2.0*PI;            
            
            heading2 = heading2 * 180.0/PI;
            
            //Print out values of each axis.
            Serial.print("mx: ");
            Serial.print(x);
            Serial.print("  my: ");
            Serial.print(y);
            Serial.print("  mz: ");
            Serial.print(z);
            Serial.print("  heading: ");
            Serial.println(headingDegrees);
            Serial.print("  ax: ");
            Serial.print(ax);
            Serial.print("  ay: ");
            Serial.print(ay);
            Serial.print("  az: ");
            Serial.print(az);
            Serial.print("  heading2: ");
            Serial.println(heading2);
            Serial.println();            
        }
        //TBR
        delay(750);
    }
}

///////////////////////////////////////////////////////////
// Register SPI reading function
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
// Register SPI writing function
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
// LSM330 SPI setup function
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
    // Full-scale selection  +/- 2G (default)
    // Normal mode enable
    // SPI serial interface mode selection : 4 wire interface (default)
    writeRegister(CTRL_REG4, 0b00001000);
    
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
    ax = (readRegister(0x29)&0xFF)<<8;
    ax |= (readRegister(0x28)&0xFF);

    ay = (readRegister(0x2B)&0xFF)<<8;
    ay |= (readRegister(0x2A)&0xFF);

    az = (readRegister(0x2D)&0xFF)<<8;
    az |= (readRegister(0x2C)&0xFF);
}
