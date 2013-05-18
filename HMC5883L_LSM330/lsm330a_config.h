// Copyright 2013 Michael Fayad
//
// This file is part of Max32_CodeBase.
//
// lsm330a_config.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// lsm330a_config.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with lsm330a_config.h.  If not, see <http://www.gnu.org/licenses/>.

// Choose only one scale value
//#define ACC_SCALE_2G
//#define ACC_SCALE_4G
//#define ACC_SCALE_8G
#define ACC_SCALE_16G

// Accelerometer sensitivity values according to the scaling
#ifdef ACC_SCALE_2G
    #define ACC_SENSITIVITY 0.0000637 // in g/counts (not calibrated)
#elif ACC_SCALE_4G
    #define ACC_SENSITIVITY 0.002  // in g/counts (not calibrated)
#elif ACC_SCALE_8G
    #define ACC_SENSITIVITY 0.004 // in g/counts (not calibrated)
#else //ACC_SCALE_16G
    #define ACC_SENSITIVITY 0.00078 // in g/counts (notcalibrated)
#endif

// Accelerometer CTRL_REG4 value
#ifdef ACC_SCALE_2G
    #define ACC_CTRL_REG4_VAL 0b00001000
#elif ACC_SCALE_4G
    #define ACC_CTRL_REG4_VAL 0b00011000
#elif ACC_SCALE_8G
    #define ACC_CTRL_REG4_VAL 0b00101000
#else //ACC_SCALE_16G
    #define ACC_CTRL_REG4_VAL 0b00111000
#endif
