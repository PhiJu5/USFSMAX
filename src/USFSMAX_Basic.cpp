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

#include "Arduino.h"
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
        const float magV, 
        const float magH, 
        const float magDec)
{
}

uint8_t USFSMAX_Basic::begin(uint8_t bus)
{
}

USFSMAX::DataReady_t USFSMAX_Basic::dataReadyFlags(void)
{
}

bool USFSMAX_Basic::quaternionReady(void) {
}

void USFSMAX_Basic::readGyroAcc(float gyro[3], float acc[3])
{
}

void USFSMAX_Basic::readGyroAccMagBaro(float gyro[3], float acc[3], float mag[3], float & baro)
{
}

void USFSMAX_Basic::readMagBaro(float mag[3], float & baro)
{
}

void USFSMAX_Basic::readMag(float mag[3])
{
}

void USFSMAX_Basic::readBaro(float & baro)
{
}
