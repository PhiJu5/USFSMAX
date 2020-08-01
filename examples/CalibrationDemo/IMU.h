/*
 * Copyright (c) 2019,2020 Gregory Tomasch and Simon D. Levy.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal with the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimers.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimers in the
 *     documentation and/or other materials provided with the distribution.
 *  3. The names of Gregory Tomasch, Simon D. Levy, and their successors
 *     may not be used to endorse or promote products derived from this Software
 *     without specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * WITH THE SOFTWARE.
 */

#pragma once

class IMU
{
    public:

        void compute(float quat[4], float angle[2], float & heading); // returns time taken in usec

        void computeAlternate(uint32_t Start_time, 
                float accData[3], float gyroData[3], float magData[3],
                float quat[4], float angle[2], float & heading,
                const float magDeclination, const float rpsPerDps);

    private:

        // Constants for alternate quaternion
        const float KP_DEF         = 0.5f;    // Mahony filter proportional gain
        const float KI_DEF         = 0.0f;    // Mahony filter integral gain
        const float GYRO_MEAS_ERR  = 1.0f;    // Madgwick gyro measurement error. Start at 1 deg/s; original: 2.5deg/s)
        const float GyroMeasError  = GYRO_MEAS_ERR*0.01745329252f; // Madgwick fliter (RPS)
        const float GyroMeasDrift  = GYRO_MEAS_ERR*0.01745329252f; // Madgwick filter (RPS/s)
        const float beta           = sqrt(3.0f/4.0f)*GyroMeasError;// Madgwick fliter compute beta
        const float zeta           = sqrt(3.0f/4.0f)*GyroMeasDrift;// Madgwick fliter Compute zeta (Small or zero)
        const float twoKp          = 2.0f*KP_DEF;                  // Mahony filter 2X proportional gain (Kp)
        const float twoKi          = 2.0f*KI_DEF;                  // Mahony filter 2X integral gain (Ki)

        // Mahony filter integral error terms
        float integralFBx    = 0.0f;                         
        float integralFBy    = 0.0f;
        float integralFBz    = 0.0f;        

        void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz,
                float mx, float my, float mz, float deltat, float* quat);
        void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz,
                float mx, float my, float mz, float deltat, float* quat);
};
