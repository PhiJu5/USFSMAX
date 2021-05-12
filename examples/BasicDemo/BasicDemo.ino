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

#ifdef __MK20DX256__
#include <i2c_t3.h>
#elif defined(ARDUINO)
#include <Wire.h>
#endif

// Un-comment one
//static uint32_t INTERRUPT_PIN = 23; // Teensy4.0
static uint32_t INTERRUPT_PIN = 32; // TinyPICO
//static uint32_t INTERRUPT_PIN = 2; // Butterfly STM32L433

// Magnetic constants for Lexington, VA, USA
// For your location, use https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfwmm
static const float MAG_V           = 45.821;  // vertical intensity (uT)
static const float MAG_H           = 21.521;  // horizontal intensity (uT)
static const float MAG_DECLINATION = -9.1145; // angle, degrees

// Support for interrupt handler
static volatile bool dataReady;

// Serial update period (ms)
static const uint32_t UPDATE_PERIOD  = 100;

// I2C Clock Speed
static const uint32_t I2C_CLOCK =  400000L;    // 400kHz

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
    Serial.print(val < 0 ? "" : "+");
    Serial.print(val);
}

static void printSensor(float vals[3], const char * label, const char * units, uint8_t n=3)
{
    Serial.print(label);
    Serial.print(": ");

    for (uint8_t k=0; k<n; ++k) {
        printVal(vals[k]);
        Serial.print(" ");
    }

    Serial.print(units);
    Serial.print(" ");
}

static void printDelimiter(void)
{
    Serial.print(" | ");
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
            break;
            printAccGyro();
            Serial.print("\n");
            break;
        case USFSMAX::DATA_READY_GYRO_ACC_MAG_BARO:
            break;
            printAccGyro();
            printDelimiter();
            printMag();
            printDelimiter();
            printBaro();
            Serial.print("\n");
            break;
        case USFSMAX::DATA_READY_MAG_BARO:
            break;
            printMag();
            printDelimiter();
            printBaro();
            Serial.print("\n");
            break;
        case USFSMAX::DATA_READY_MAG:
            break;
            printMag();
            Serial.print("\n");
            break;
        case USFSMAX::DATA_READY_BARO:
            break;
            printBaro();
            Serial.print("\n");
            break;
        default:
            break;
    };

    if (usfsmax.quaternionReady()) {
        float quat[4] = {};
        usfsmax.readQuat(quat);
        reportEulerAngles(quat);
        //printSensor(quat, "q", "", 4);
        //Serial.print("\n");
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
        Serial.print(status);
        Serial.print("\n");
        delay(500);
    }
}

static float rad2deg(float rad)
{
    return 180 * rad / M_PI;
}

static void reportEulerAngles(float q[4])
{
    float qw = q[0];
    float qx = q[1];
    float qy = q[2];
    float qz = q[3];

    float ex = rad2deg(atan2(2.0f*(qw*qx+qy*qz),qw*qw-qx*qx-qy*qy+qz*qz));
    float ey = rad2deg(asin(2.0f*(qx*qz-qw*qy)));
    float ez = rad2deg(atan2(2.0f*(qx*qy+qw*qz),qw*qw+qx*qx-qy*qy-qz*qz));

    Serial.printf("roll: %+3.3f    pitch: %+3.3f    yaw: %+3.3f\n", ex, ey, ez);
}

void setup()
{
    Serial.begin(115200);

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

    Serial.print("Configuring the coprocessor...\n");

    if (status) {
        error(status);
    }

    Wire.setClock(I2C_CLOCK);// Set the I2C clock to high speed for run-mode data collection
    delay(100);

    // Attach interrupt (implemented only for Arduino)
    pinMode(INTERRUPT_PIN, INPUT);
    attachInterrupt(INTERRUPT_PIN, DRDY_handler, RISING);

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
