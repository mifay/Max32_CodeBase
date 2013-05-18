// Copyright 2013 Michael Fayad
//
// This file is part of Max32_CodeBase.
//
// hmc5883l_config.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// hmc5883l_config.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with hmc5883l_config.h.  If not, see <http://www.gnu.org/licenses/>.

// Choose only one gain value
#define COMPASS_GAIN_1_3_GA // 1.3 Ga
//#define COMPASS_GAIN_8_1_GA // 8.1 Ga

// Compass scale values
#ifdef COMPASS_GAIN_1_3_GA
    #define COMPASS_SCALE 0.92 // in mG/counts (not calibrated)
#else //COMPASS_GAIN_8_1_GA
    #define COMPASS_SCALE 4.347826 // in mG/counts (not calibrated)
#endif

// Compass gain values
#ifdef COMPASS_GAIN_1_3_GA
    #define COMPASS_GAIN 0x20
#else //COMPASS_GAIN_8_1_GA
    #define COMPASS_GAIN 0xE0
#endif