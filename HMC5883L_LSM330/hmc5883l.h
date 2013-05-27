// Copyright 2013 Michael Fayad
//
// This file is part of Max32_CodeBase.
//
// hmc5883l.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// hmc5883l.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with hmc5883l.h.  If not, see <http://www.gnu.org/licenses/>.

#include <stdint.h> // to use int16_t type to avoid sign extension problems with read output data

// Compass values and registers
#define HMC_ADDRESS 0x1E
#define CONFIG_REG_A 0x00
#define OUTPUT_DATA_RATE_75_HZ 0x18
#define CONFIG_REG_B 0x01
#define MODE_REG 0x02
#define CONTINUOUS_MEASUREMENT 0x00
#define SINGLESHOT_MEASUREMENT 0x01
#define IDLE_MEASUREMENT 0x03
#define DATA_REG_BEGIN 0x03
#define ID_REG 0x0A
#define ID_REG_VALUE 0x48

class HMC5883L
{
    public:
        void setup(float gain);
        bool readMagnetism();
        float calcHeading(float rollRadians, float pitchRadians/*, float yawRadians*/);
        
        int16_t magnX_; // in counts
        int16_t magnY_; // in counts
        int16_t magnZ_; // in counts
        
        int16_t scaledMagnX_; // in Gauss        
        int16_t scaledMagnY_; // in Gauss        
        int16_t scaledMagnZ_; // in Gauss        
        
        float heading_; // In degrees
    private:
        void setGain(float gain);
        
        // Attributes
        float gain_;  // in Gauss
        float scale_; // multiplicator to scale compass output (in miliGaus/counts)
        int gainRegVal_; // register value to set compass gain
};


