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

#ifdef __MK20DX256__
#include <i2c_t3.h>
#else
#include <Wire.h>
#endif

#include "USFSMAX_Basic.h"

// Un-comment one
//static uint32_t INTERRUPT_PIN = 23; // Teensy4.0
//static uint32_t INTERRUPT_PIN = 32; // TinyPICO
static uint32_t INTERRUPT_PIN = 2; // Butterfly STM32L433

// Magnetic constants for Kelseyville, CA
// For your location, use https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfwmm
static const float MAG_V           = 42.9631f; // angle, degrees
static const float MAG_H           = 22.7568f; // angle, degrees
static const float MAG_DECLINATION = 13.7433f; // uT

// Support for interrupt handler
static volatile bool dataReady;

// Serial update period (ms)
static uint32_t UPDATE_PERIOD  = 100;

// I2C Clock Speed
static uint32_t I2C_CLOCK = 1000000;    // 1MHz

// Output Data Rates (ODRs)
static USFSMAX::AccelGyroODR_t ACCEL_ODR = USFSMAX::ACCEL_GYRO_ODR_834;
static USFSMAX::AccelGyroODR_t GYRO_ODR  = USFSMAX::ACCEL_GYRO_ODR_834;
static USFSMAX::MagODR_t       MAG_ODR   = USFSMAX::MAG_ODR_100;
static USFSMAX::BaroODR_t      BARO_ODR  = USFSMAX::BARO_ODR_50;
static USFSMAX::QuatDiv_t      QUAT_DIV  = USFSMAX::QUAT_DIV_8;

// LSM6DSM filter settings
static USFSMAX::LSM6DSMGyroLPF_t   LSM6DSM_GYRO_LPF    = USFSMAX::LSM6DSM_GYRO_LPF_167;
static USFSMAX::LSM6DSMAccLpfODR_t LSM6DSM_ACC_LPF_ODR = USFSMAX::LSM6DSM_ACC_LPF_ODR_DIV400;

// LIS2MDL filter setting
static USFSMAX::LIS2MDLMagLpfODR_t LIS2MDL_MAG_LPF_ODR = USFSMAX::LIS2MDL_MAG_LPF_ODR_4;

// LPS22HB baro filter setting
static USFSMAX::LPS22HBBaroLpfODR_t LPS22HB_BARO_LPF = USFSMAX::LPS22HB_BARO_LPF_ODR_20;

// IMU scaling
USFSMAX::AccScale_t  ACC_SCALE  = USFSMAX::ACC_SCALE_16;
USFSMAX::GyroScale_t GYRO_SCALE = USFSMAX::GYRO_SCALE_2000;

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

static void dumpVal(float val)
{
    Serial.print(val < 0 ? "" : "+");
    Serial.print(val, 4);
}

static void dumpSensor(float vals[3], const char * label, const char * units, uint8_t n=3)
{
    Serial.print(label);
    Serial.print(": ");

    for (uint8_t k=0; k<n; ++k) {
        dumpVal(vals[k]);
        Serial.print(" ");
    }

    Serial.print(units);
    Serial.print(" ");
}

static void dumpDelimiter(void)
{
    Serial.print(" | ");
}

static void dumpAccGyro()
{
    float gyro[3] = {};
    float acc[3] = {};

    usfsmax.readGyroAcc(gyro, acc);

    dumpSensor(gyro, "g", "deg/s");
    dumpDelimiter();
    dumpSensor(acc, "a", "g");
}

static void dumpMag()
{
    float mag[3] = {};
    usfsmax.readMag(mag);
    dumpSensor(mag, "m", "uT");
}

static void dumpBaro()
{
    float baro = 0;
    usfsmax.readBaro(baro);
    dumpSensor(&baro, "b", "hPa", 1);
}

static void fetchUsfsmaxData(void)
{
    float mag[3] = {};
    float baro = 0;

    // Optimize the I2C read function with respect to whatever sensor data is ready
    switch (usfsmax.dataReady()) {
        case USFSMAX::DATA_READY_GYRO_ACC:
            dumpAccGyro();
            Serial.println();
            break;
        case USFSMAX::DATA_READY_GYRO_ACC_MAG_BARO:
            dumpAccGyro();
            dumpDelimiter();
            dumpMag();
            dumpDelimiter();
            dumpBaro();
            Serial.println();
            break;
        case USFSMAX::DATA_READY_MAG_BARO:
            dumpMag();
            dumpDelimiter();
            dumpBaro();
            Serial.println();
            break;
        case USFSMAX::DATA_READY_MAG:
            dumpMag();
            Serial.println();
            break;
        case USFSMAX::DATA_READY_BARO:
            dumpBaro();
            Serial.println();
            break;
        default:
            break;
    };

    if (usfsmax.quaternionReady()) {
        float quat[4] = {};
        usfsmax.readQuat(quat);
        dumpSensor(quat, "q", "", 4);
        Serial.println();
    }

} // fetchUsfsmaxData

// Host DRDY interrupt handler
static void DRDY_handler()
{
    dataReady = true;
}

static void error(uint8_t status)
{
    while (true) {
        Serial.print("Got error ");
        Serial.println(status);
        delay(500);
    }
}

void setup()
{
    // Open serial port
    Serial.begin(115200);

    // Set up DRDY interrupt pin
    pinMode(INTERRUPT_PIN, INPUT);

    // Initialize I^2C bus, setting I2C clock speed to 100kHz
#ifdef __MK20DX256__
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_INT, I2C_RATE_100);
#else
    Wire.begin();
    delay(100);
    Wire.setClock(100000); 
#endif

    delay(1000);

    uint8_t status = usfsmax.begin(); // Start USFSMAX

    Serial.println("Configuring the coprocessor...");

    if (status) {
        error(status);
    }

    Wire.setClock(I2C_CLOCK);// Set the I2C clock to high speed for run-mode data collection
    delay(100);

    // Attach interrupts
    attachInterrupt(INTERRUPT_PIN, DRDY_handler, RISING);           // Attach DRDY interrupt

} // setup

void loop()
{
    int32_t  baroADC;

    static uint32_t lastRefresh;

    float quat[4];

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
