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
#include <stdio.h>

#if defined(RASPBERRY_PI)
extern "C" {
#include <wiringPi.h>
}
#else
#endif

// Magnetic constants for Kelseyville, CA
// For your location, use https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfwmm
static const float MAG_V           = 42.9631f; // angle, degrees
static const float MAG_H           = 22.7568f; // angle, degrees
static const float MAG_DECLINATION = 13.7433f; // uT

// Support for interrupt handler
static volatile bool dataReady;

// Serial update period (ms)
static const uint32_t UPDATE_PERIOD  = 100;

// I2C Clock Speed
static const uint32_t I2C_CLOCK = 1000000;    // 1MHz

// Output Data Rates (ODRs)
static const USFSMAX::AccelGyroODR_t ACCEL_ODR = USFSMAX::ACCEL_GYRO_ODR_834;
static const USFSMAX::AccelGyroODR_t GYRO_ODR  = USFSMAX::ACCEL_GYRO_ODR_834;
static const USFSMAX::MagODR_t       MAG_ODR   = USFSMAX::MAG_ODR_100;
static const USFSMAX::BaroODR_t      BARO_ODR  = USFSMAX::BARO_ODR_50;
static const USFSMAX::QuatDiv_t      QUAT_DIV  = USFSMAX::QUAT_DIV_8;

// LSM6DSM filter settings
static const USFSMAX::LSM6DSMGyroLPF_t   LSM6DSM_GYRO_LPF    = USFSMAX::LSM6DSM_GYRO_LPF_167;
static const USFSMAX::LSM6DSMAccLpfODR_t LSM6DSM_ACC_LPF_ODR = USFSMAX::LSM6DSM_ACC_LPF_ODR_DIV400;

// LIS2MDL filter setting
static const USFSMAX::LIS2MDLMagLpfODR_t LIS2MDL_MAG_LPF_ODR = USFSMAX::LIS2MDL_MAG_LPF_ODR_4;

// LPS22HB baro filter setting
static const USFSMAX::LPS22HBBaroLpfODR_t LPS22HB_BARO_LPF = USFSMAX::LPS22HB_BARO_LPF_ODR_20;

// IMU scaling
static const USFSMAX::AccScale_t  ACC_SCALE  = USFSMAX::ACC_SCALE_16;
static const USFSMAX::GyroScale_t GYRO_SCALE = USFSMAX::GYRO_SCALE_2000;


static USFSMAX_Basic
usfsmax(
        ACCEL_ODR,
        GYRO_ODR,
        MAG_ODR,
        BARO_ODR,
        QUAT_DIV,
        LSM6DSM_GYRO_LPF,
        LSM6DSM_ACC_LPF_ODR,
        ACC_SCALE,
        GYRO_SCALE,
        LIS2MDL_MAG_LPF_ODR,
        LPS22HB_BARO_LPF,
        MAG_V,
        MAG_H,
        MAG_DECLINATION);

static void printVal(float val)
{
    printf("%+4.4f", val);
}

static void printSensor(float vals[3], const char * label, const char * units, uint8_t n=3)
{
    printf("%s: ", label);

    for (uint8_t k=0; k<n; ++k) {
        printVal(vals[k]);
        printf("%s", " ");
    }

    printf("%s ", units);
}

static void printDelimiter(void)
{
    printf("%s", " | ");
}

static void printAccGyro()
{
    float gyro[3] = {};
    float acc[3] = {};

    usfsmax.readGyroAcc(gyro, acc);

    printSensor(gyro, "g", "deg/s");
    printDelimiter();
    printSensor(acc, "a", "g");
}

static void printMag()
{
    float mag[3] = {};
    usfsmax.readMag(mag);
    printSensor(mag, "m", "uT");
}

static void printBaro()
{
    float baro = 0;
    usfsmax.readBaro(baro);
    printSensor(&baro, "b", "hPa", 1);
}

static void fetchUsfsmaxData(void)
{
    // Optimize the I2C read function with respect to whatever sensor data is ready
    switch (usfsmax.dataReady()) {
        case USFSMAX::DATA_READY_GYRO_ACC:
            printAccGyro();
            printf("%s", "\n");
            break;
        case USFSMAX::DATA_READY_GYRO_ACC_MAG_BARO:
            printAccGyro();
            printDelimiter();
            printMag();
            printDelimiter();
            printBaro();
            printf("%s", "\n");
            break;
        case USFSMAX::DATA_READY_MAG_BARO:
            printMag();
            printDelimiter();
            printBaro();
            printf("%s", "\n");
            break;
        case USFSMAX::DATA_READY_MAG:
            printMag();
            printf("%s", "\n");
            break;
        case USFSMAX::DATA_READY_BARO:
            printBaro();
            printf("%s", "\n");
            break;
        default:
            break;
    };

    if (usfsmax.quaternionReady()) {
        float quat[4] = {};
        usfsmax.readQuat(quat);
        printSensor(quat, "q", "", 4);
        printf("%s", "\n");
    }

} // fetchUsfsmaxData


static void error(uint8_t status)
{
    while (true) {
        printf("Got error %d\n", status);
        delay(500);
    }
}

void setup()
{
    uint8_t status = usfsmax.begin(); // Start USFSMAX

    printf("%s", "Configuring the coprocessor...\n");

    if (status) {
        error(status);
    }

    delay(100);

} // setup

void loop()
{
    static uint32_t lastRefresh;

    if (dataReady) {

        dataReady = false;

        // Get the new data from the USFSMAX, and run our alternate quaternion IMU if we have new gyro data
        fetchUsfsmaxData();
    }

    // Update serial output
    if ((millis() - lastRefresh) > UPDATE_PERIOD)  {   

        lastRefresh = millis();

        dataReady = false;
    }

    delay(5);

} // loop
