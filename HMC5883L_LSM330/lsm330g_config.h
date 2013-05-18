// Copyright 2013 Michael Fayad
//
// This file is part of Max32_CodeBase.
//
// lsm330g_config.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// lsm330g_config.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with lsm330g_config.h.  If not, see <http://www.gnu.org/licenses/>.

// Choose only one scale value
#define GYRO_SCALE_250_DPS
//#define GYRO_SCALE_500_DPS
//#define GYRO_SCALE_2000_DPS

// GYRO sensitivity
#ifdef GYRO_SCALE_250_DPS
    #define GYRO_SENSITIVITY 0.00875 // in dps/count (not calibrated)
#elif GYRO_SCALE_500_DPS
    #define GYRO_SENSITIVITY 0.0175 // in dps/count (not calibrated)
#else //GYRO_SCALE_2000_DPS
    #define GYRO_SENSITIVITY 0.070 // in dps/count (not calibrated)
#endif

// gyro CTRL_REG4 value
#ifdef GYRO_SCALE_250_DPS
    #define GYRO_CTRL_REG4_VAL 0x00
#elif GYRO_SCALE_500_DPS
    #define GYRO_CTRL_REG4_VAL 0x10
#else //GYRO_SCALE_2000_DPS
    #define GYRO_CTRL_REG4_VAL 0x20
#endif
