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

#include "USFSMAX_Basic.h"

USFSMAX_Basic::USFSMAX_Basic(
        const USFSMAX::AccelGyroODR_t accelODR, 
        const USFSMAX::AccelGyroODR_t gyroODR,
        const USFSMAX::MagODR_t magODR,
        const USFSMAX::BaroODR_t baroODR,
        const USFSMAX::QuatDiv_t quatDiv,
        const USFSMAX::LSM6DSMGyroLPF_t lsm6dsmGyroLPF,
        const USFSMAX::LSM6DSMAccLpfODR_t lsm6dsmGyroLpfODR,
        const USFSMAX::AccScale_t accScale,
        const USFSMAX::GyroScale_t gyroScale,
        const USFSMAX::LIS2MDLMagLpfODR_t lis2mdlMagLpfODR,
        const USFSMAX::LPS22HBBaroLpfODR_t lps22hbBaroLpfODR,
        float magV,
        const float magH,
        const float magDec)
{
    _usfsmax.init(
        accelODR,
        gyroODR,
        magODR,
        baroODR,
        quatDiv,
        lsm6dsmGyroLPF,
        lsm6dsmGyroLpfODR,
        accScale,
        gyroScale,
        lis2mdlMagLpfODR,
        lps22hbBaroLpfODR,
        magV,
        magH,
        magDec,
        true,  // enableDHICorrector
        false, // use2DDHICorrector
        false, // eulerQuatFlag
        true); // scaledSensorDataFlag
}

uint8_t USFSMAX_Basic::begin(uint8_t bus)
{
    return _usfsmax.begin(bus);
}

USFSMAX::DataReady_t USFSMAX_Basic::dataReady(void)
{
    return _usfsmax.dataReady();
}

bool USFSMAX_Basic::quaternionReady(void)
{
    return _usfsmax.quaternionReady();
}

void USFSMAX_Basic::readGyroAcc(float gyro[3], float acc[3])
{
    int16_t  gyroADC[3] = {};
    int16_t  accADC[3] = {};

    _usfsmax.getGyroAccelADC(gyroADC, accADC);

    for(uint8_t i=0; i<3; i++) {
        gyro[i] = gyroADC[i] * _usfsmax.dpsPerCount;
        acc[i]  = accADC[i]  * _usfsmax.gPerCount;
    }
}

void USFSMAX_Basic::readMag(float mag[3])
{
    int16_t magADC[3] = {};

    _usfsmax.getMagADC(magADC);

    for(uint8_t i=0; i<3; i++) {
        mag[i] = magADC[i] * USFSMAX::MAG_UT_PER_COUNT;
    }
}

void USFSMAX_Basic::readBaro(float & baro)
{
    int32_t baroADC = 0;
    _usfsmax.getBaroADC(baroADC);
    baro = baroADC / 4096.0f;
}

void USFSMAX_Basic::readQuat(float quat[4])
{
    _usfsmax.getQuatLin(quat);
}
