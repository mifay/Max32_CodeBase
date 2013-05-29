// Copyright 2013 Michael Fayad
//
// This file is part of Max32_CodeBase.
//
// attitude.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// attitude.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with attitude.h.  If not, see <http://www.gnu.org/licenses/>.
//
//
// source :      Tom Pycke (http://code.google.com/p/gluonpilot)

#include "attitude.h"
#include <math.h> // To use sin, cos, etc functions

// Function to init quaternions from Euler angles
void Attitude::init(float roll,float pitch,float yaw)
{
    float cos_roll_2 = cosf(roll/2.0);
    float sin_roll_2 = sinf(roll/2.0);
    float cos_pitch_2 = cosf(pitch/2.0);
    float sin_pitch_2 = sinf(pitch/2.0);
    float cos_yaw_2 = cosf(yaw/2.0);
    float sin_yaw_2 = sinf(yaw/2.0);


    this->q0_ = cos_roll_2 * cos_pitch_2 * cos_yaw_2 + sin_roll_2 * sin_pitch_2 * sin_yaw_2;
    this->q1_ = sin_roll_2 * cos_pitch_2 * cos_yaw_2 - cos_roll_2 * sin_pitch_2 * sin_yaw_2;
    this->q2_ = cos_roll_2 * sin_pitch_2 * cos_yaw_2 + sin_roll_2 * cos_pitch_2 * sin_yaw_2;
    this->q3_ = cos_roll_2 * cos_pitch_2 * sin_yaw_2 - sin_roll_2 * sin_pitch_2 * cos_yaw_2;


    float norm = sqrtf(q0_*q0_ + q1_*q1_ + q2_*q2_ + q3_*q3_);

    q0_ /= norm;
    q1_ /= norm;
    q2_ /= norm;
    q3_ /= norm;
}

/*
 *   Update quaternion with rates.
 *
 *   in:
 *      Euler angles in degrees (from the gyro)
 *      dt : deltaT
 */
void Attitude::update(float scaledGx, float scaledGy, float scaledGz, unsigned long dt_us)
{
    // Convert from microseconds to seconds
    float dt = (float)dt_us / 1000000.0;

    float w1 = scaledGy * M_PI/180.0;
    float w2 = scaledGx * M_PI/180.0;
    float w3 = scaledGz * M_PI/180.0;

    float oldQ0 = this->q0_;
    float oldQ1 = this->q1_;
    float oldQ2 = this->q2_;
    float oldQ3 = this->q3_;

    this->q0_ += 0.5 * (         - oldQ1*w1 - oldQ2*w2 - oldQ3*w3)*dt;
    this->q1_ += 0.5 * (oldQ0*w1 +            oldQ2*w3 - oldQ3*w2)*dt;
    this->q2_ += 0.5 * (oldQ0*w2 - oldQ1*w3 +            oldQ3*w1)*dt;
    this->q3_ += 0.5 * (oldQ0*w3 + oldQ1*w2 - oldQ2*w1           )*dt;

    float norm = sqrtf(q0_*q0_ + q1_*q1_ + q2_*q2_ + q3_*q3_);

    this->q0_ /= norm;
    this->q1_ /= norm;
    this->q2_ /= norm;
    this->q3_ /= norm;
}

// Convert Quaternions to euler angles
float Attitude::getRoll()
{
    float r;
    r = atan2( 2.0 * ( q2_*q3_ + q0_*q1_ ),(1.0 - 2.0 * (q1_*q1_ + q2_*q2_)) );
    return r;
}

// Convert Quaternions to euler angles
float Attitude::getPitch()
{
    float r;
    r = asinf( -2.0 * (q1_*q3_ - q0_*q2_) );
    return r;
}

// Convert Quaternions to euler angles
float Attitude::getYaw()
{
    float r;
    r = atan2( 2.0 * ( q0_*q3_ + q1_*q2_ ) ,(1.0 - 2.0 * (q2_*q2_ + q3_*q3_)) );
    return r;
}
