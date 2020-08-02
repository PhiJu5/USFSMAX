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

#include "Alarms.h"
#include "USFSMAX.h"
#include "SensorCal.h"

// Un-comment one
//static uint32_t INTERRUPT_PIN = 23; // Teensy4.0
//static uint32_t INTERRUPT_PIN = 32; // TinyPICO
static uint32_t INTERRUPT_PIN = 2; // Butterfly STM32L433

// Magnetic constants for Kelseyville, CA
// For your location, use https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfwmm
static const float MAG_V           = 42.9631f; // angle, degrees
static const float MAG_H           = 22.7568f; // angle, degrees
static const float MAG_DECLINATION = 13.7433f; // uT

// Calibration-related constants
static const float LIS2MDL_UT_PER_COUNT =  0.15f;
static const float  UT_PER_COUNT  = LIS2MDL_UT_PER_COUNT;

// For alternative IMU fusion filter
static const float RPS_PER_DPS  = 0.01745329;

// USFSMAX-related flags
static const bool EULER_QUAT_FLAG         = false;
static const bool SCALED_SENSOR_DATA_FLAG = true;

// Support for interrupt handler
static volatile bool dataReady;

// Serial update period (ms)
static uint32_t UPDATE_PERIOD  = 100;

// Visualize the magnetometer response surface on the "MotionCal" GUI (https://www.pjrc.com/store/prop_shield.html)
static bool MOTION_CAL_GUI_ENABLED  = false;

// I2C Clock Speed
static uint32_t I2C_CLOCK = 1000000;    // 1MHz

// Dynamic Hard Iron corrector (Uncomment one only)
static bool ENABLE_DHI_CORRECTOR  = true;
static bool USE_2D_DHI_CORRECTOR  = false;

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

// Pin definitions
static uint32_t LED_PIN       = 13;

// Instantiate class objects

static USFSMAX    
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
        MAG_DECLINATION,
        ENABLE_DHI_CORRECTOR,
        USE_2D_DHI_CORRECTOR,
        EULER_QUAT_FLAG,
        SCALED_SENSOR_DATA_FLAG);

static SensorCal sensorCal(&usfsmax);

static void fetchUsfsmaxData(void)
{
    // Optimize the I2C read function with respect to whatever sensor data is ready
    switch (usfsmax.dataReadyFlags()) {
        case USFSMAX::DATA_READY_GYRO_ACC:
            Serial.println("gyro acc");
            break;
        case USFSMAX::DATA_READY_GYRO_ACC_MAG_BARO:
            Serial.println("gyro acc mag baro");
            break;
        case USFSMAX::DATA_READY_MAG_BARO:
            Serial.println("mag baro");
            break;
        case USFSMAX::DATA_READY_MAG:
            Serial.println("mag");
            break;
        case USFSMAX::DATA_READY_BARO:
            Serial.println("baro");
            break;
        default:
            break;
    };

    if (usfsmax.quaternionReady()) {
        Serial.println("quat acc");
    }

} // fetchUsfsmaxData

// Host DRDY interrupt handler
static void DRDY_handler()
{
    dataReady = true;
}

// Serial interface handler
static void serialInterfaceHandler()
{
    uint8_t serial_input = 0;
    if (Serial.available()) serial_input = Serial.read();
    if (serial_input == 49) {sensorCal.GyroCal();} // Type "1" to initiate usfsmax Gyro Cal
    if (serial_input == 50)                         // Type "2" to list current sensor calibration data
    {
        Wire.setClock(100000);     // Set I2C clock to 100kHz to read the calibration data from the MAX32660
        delay(100);
        usfsmax.retrieveFullGyroCal();
        delay(100);
        usfsmax.retrieveFullAccelCal();
        delay(100);
        usfsmax.retrieveEllipMagCal();
        delay(100);
        usfsmax.retrieveFinalMagCal();
        delay(100);
        Wire.setClock(I2C_CLOCK);  // Resume high-speed I2C operation
        delay(100);

        // Print the calibration results
        sensorCal.sendOneToProceed();     // Halt the serial monitor to let the user read the calibration data
    }
    if (serial_input == 51) {usfsmax.resetDHI();} // Type "3" to reset the DHI corrector
    serial_input = 0;
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
    delay(2000);

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

    delay(2000);

    uint8_t status = usfsmax.begin(); // Start USFSMAX

    Serial.println("Configuring the coprocessor...");

    if (status) {
        error(status);
    }

    Wire.setClock(I2C_CLOCK);// Set the I2C clock to high speed for run-mode data collection
    delay(100);

    // Attach interrupts
    attachInterrupt(INTERRUPT_PIN, DRDY_handler, RISING);           // Attach DRDY interrupt

    Serial.println("USFXMAX successfully initialized!\n");
    sensorCal.sendOneToProceed();                            // Halt the serial monitor to let the user read the results

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
    uint32_t deltaT = millis() - lastRefresh;
    if (deltaT > UPDATE_PERIOD)  {                               // Update the serial monitor every "UPDATE_PERIOD" ms

        lastRefresh = millis();
        usfsmax.getMxMy(UT_PER_COUNT);                        // Get Horizontal magnetic components

        serialInterfaceHandler();

        if (ENABLE_DHI_CORRECTOR) {
            uint8_t calStatus = usfsmax.getCalibrationStatus();// Poll calibration status byte
            usfsmax.getDHIRsq();                                               // Get DHI R-square
        }

        // Toggle LED if not calibrating gyroscopes
        if (sensorCal.gyroCalActive) {
            if ((usfsmax.getCalibrationStatus() & 0x01) == 0) {
                sensorCal.gyroCalActive = false;
            }
        } 
        dataReady = false;
    }

    delay(5);

} // loop
