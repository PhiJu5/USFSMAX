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
#include "IMU.h"
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

// See the verbose screen update; set to false for spreadsheet or "MotionCal" GUI output
static bool SERIAL_DEBUG = true;

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

// Helpful constants

enum axes
{
    EAST = 0,
    NORTH,
    UP
};

enum attitudes
{
    PITCH = 0,
    ROLL,
    YAW
};


// Support timing test for alternate quaternion fusion
static uint32_t startTime;

// Instantiate class objects

static Alarms alarms(LED_PIN);

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

static IMU imu;

static SensorCal sensorCal(&usfsmax, 0, &alarms);

static void accOrientation(float accData[3], float sensorPoint[3])
{
    accData[EAST]  = +sensorPoint[0];     
    accData[NORTH] = +sensorPoint[1];     
    accData[UP]    = +sensorPoint[2];
}

// ENU rotation axes
static void gyroOrientation(float gyroData[3], float sensorPoint[3])
{
    gyroData[PITCH] = +sensorPoint[0]; // PITCH (EAST) axis gyro (nose-up +)   
    gyroData[ROLL]  = +sensorPoint[1]; // ROLL (NORTH) axis gyro (CW +)
    gyroData[YAW]   = -sensorPoint[2]; // YAW (UP) axis gyro (E of N +)
}

// Mag follows NED convention FWD RIGHT DOWN                             
// Check component sign with NOAA calculator
static void magOrientation(float magData[3], float sensorPoint[3])
{
    magData[0] = -sensorPoint[1]; 
    magData[1] = +sensorPoint[0]; 
    magData[2] = -sensorPoint[2];  // sensor is left-handed
}

static void convertMagData(int16_t magADC[3], float magData[3])
{
    if (SCALED_SENSOR_DATA_FLAG) {                 // Calibration data is applied in the coprocessor; just scale
        for(uint8_t i=0; i<3; i++) {
            magData[i] = ((float)magADC[i])*UT_PER_COUNT;
        }
    } else {                                   // Calibration data applied locally
        float magCalData[3];
        float sensorPoint[3];
        sensorCal.apply_adv_calibration(usfsmax.ellipsoidMagCal, magADC, UT_PER_COUNT, magCalData);
        sensorCal.apply_adv_calibration(usfsmax.finalMagCal, magCalData, 1.0f, sensorPoint);
        magOrientation(magData, sensorPoint);
    }
}

static void convertAccData(int16_t accADC[3], float accData[3])
{
    float g_per_count = usfsmax.gPerCount;
    if (SCALED_SENSOR_DATA_FLAG) {                 // Calibration data is applied in the coprocessor; just scale
        for(uint8_t i=0; i<3; i++) {
            accData[i] = ((float)accADC[i])*g_per_count;
        } 
    } else {                                   // Calibration data applied locally
        float sensorPoint[3];
        sensorCal.apply_adv_calibration(usfsmax.accelCal, accADC, g_per_count, sensorPoint);
        accOrientation(accData, sensorPoint);
    }
}

static void convertGyroData(int16_t gyroADC[3], float gyroData[3])
{
    float  dps_per_count = usfsmax.dpsPerCount;
    if (SCALED_SENSOR_DATA_FLAG) {                 // Calibration data is applied in the coprocessor; just scale
        for(uint8_t i=0; i<3; i++) {
            gyroData[i] = ((float)gyroADC[i])*dps_per_count;
        }
    } else {                                   // Calibration data applied locally
        float sensorPoint[3];
        sensorCal.apply_adv_calibration(usfsmax.gyroCal, gyroADC, dps_per_count, sensorPoint);
        gyroOrientation(gyroData, sensorPoint);
    }
}

static void computeAnglesFromQuat(float quat[4], float angle[2], float & heading)

{
    usfsmax.getQuatLin(quat);
    imu.compute(quat, angle, heading);
}

static void fetchUsfsmaxData(float gyroData[3], float accData[3], float magData[3], 
        float quat[4], float angle[2], float & heading, int32_t & baroADC)
{
    int16_t  gyroADC[3] = {};
    int16_t  accADC[3] = {};
    int16_t  magADC[3] = {};

    // Optimize the I2C read function with respect to whatever sensor data is ready
    switch (usfsmax.dataReadyFlags()) {
        case USFSMAX::DATA_READY_GYRO_ACC:
            usfsmax.getGyroAccelADC(gyroADC, accADC);
            convertGyroData(gyroADC, gyroData);
            convertAccData(accADC, accData);
            break;
        case USFSMAX::DATA_READY_GYRO_ACC_MAG_BARO:
            usfsmax.getGyroAccelMagBaroADC(gyroADC, accADC, magADC, baroADC);
            convertGyroData(gyroADC, gyroData);
            convertAccData(accADC, accData);
            convertMagData(magADC, magData);
            break;
        case USFSMAX::DATA_READY_MAG_BARO:
            usfsmax.getMagBaroADC(magADC, baroADC);
            convertMagData(magADC, magData);
            break;
        case USFSMAX::DATA_READY_MAG:
            usfsmax.getMagADC(magADC);
            convertMagData(magADC, magData);
            break;
        case USFSMAX::DATA_READY_BARO:
            usfsmax.getBaroADC(baroADC);
            break;
        default:
            break;
    };

    if (usfsmax.quaternionReady()) {
        computeAnglesFromQuat(quat, angle, heading);
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
        printCalibrationResults();
        sensorCal.sendOneToProceed();     // Halt the serial monitor to let the user read the calibration data
    }
    if (serial_input == 51) {usfsmax.resetDHI();} // Type "3" to reset the DHI corrector
    serial_input = 0;

    // Hotkey messaging
    Serial.println("'1' Gyro Cal");
    Serial.println("'2' List Cal Data");
    Serial.println("'3' Reset DHI Corrector\n");
}

static void printCalibrationResults(void)
{
    Serial.println("Gyroscope Sensor Offsets (dps)");
    Serial.println(usfsmax.gyroCal.V[0], 4);
    Serial.println(usfsmax.gyroCal.V[1], 4);
    Serial.println(usfsmax.gyroCal.V[2], 4);
    Serial.println("");
    Serial.println("Gyroscope Calibration Tensor");
    Serial.print(usfsmax.gyroCal.invW[0][0], 4);
    Serial.print(",");
    Serial.print(usfsmax.gyroCal.invW[0][1], 4);
    Serial.print(",");
    Serial.println(usfsmax.gyroCal.invW[0][2], 4);
    Serial.print(usfsmax.gyroCal.invW[1][0], 4);
    Serial.print(",");
    Serial.print(usfsmax.gyroCal.invW[1][1], 4);
    Serial.print(",");
    Serial.println(usfsmax.gyroCal.invW[1][2], 4);
    Serial.print(usfsmax.gyroCal.invW[2][0], 4);
    Serial.print(",");
    Serial.print(usfsmax.gyroCal.invW[2][1], 4);
    Serial.print(",");
    Serial.println(usfsmax.gyroCal.invW[2][2], 4);
    Serial.println("\n");
    Serial.println("Accelerometer Sensor Offsets (g)");
    Serial.println(usfsmax.accelCal.V[0], 4);
    Serial.println(usfsmax.accelCal.V[1], 4);
    Serial.println(usfsmax.accelCal.V[2], 4);
    Serial.println("");
    Serial.println("Accelerometer Calibration Tensor");
    Serial.print(usfsmax.accelCal.invW[0][0], 4);
    Serial.print(",");
    Serial.print(usfsmax.accelCal.invW[0][1], 4);
    Serial.print(",");
    Serial.println(usfsmax.accelCal.invW[0][2], 4);
    Serial.print(usfsmax.accelCal.invW[1][0], 4);
    Serial.print(",");
    Serial.print(usfsmax.accelCal.invW[1][1], 4);
    Serial.print(",");
    Serial.println(usfsmax.accelCal.invW[1][2], 4);
    Serial.print(usfsmax.accelCal.invW[2][0], 4);
    Serial.print(",");
    Serial.print(usfsmax.accelCal.invW[2][1], 4);
    Serial.print(",");
    Serial.println(usfsmax.accelCal.invW[2][2], 4);
    Serial.println("\n");
    Serial.println("Magnetometer Sensor Offsets (uT)");
    Serial.println(usfsmax.ellipsoidMagCal.V[0], 4);
    Serial.println(usfsmax.ellipsoidMagCal.V[1], 4);
    Serial.println(usfsmax.ellipsoidMagCal.V[2], 4); 
    Serial.println("");
    Serial.println("Magnetometer Soft Iron Correction Tensor");
    Serial.print(usfsmax.ellipsoidMagCal.invW[0][0], 4);
    Serial.print(",");
    Serial.print(usfsmax.ellipsoidMagCal.invW[0][1], 4);
    Serial.print(",");
    Serial.println(usfsmax.ellipsoidMagCal.invW[0][2], 4);
    Serial.print(usfsmax.ellipsoidMagCal.invW[1][0], 4);
    Serial.print(",");
    Serial.print(usfsmax.ellipsoidMagCal.invW[1][1], 4);
    Serial.print(",");
    Serial.println(usfsmax.ellipsoidMagCal.invW[1][2], 4);
    Serial.print(usfsmax.ellipsoidMagCal.invW[2][0], 4);
    Serial.print(",");
    Serial.print(usfsmax.ellipsoidMagCal.invW[2][1], 4);
    Serial.print(",");
    Serial.println(usfsmax.ellipsoidMagCal.invW[2][2], 4);
    Serial.println("\n");
    Serial.println("Magnetometer Residual Hard Iron Offsets (uT)");
    Serial.println(usfsmax.finalMagCal.V[0], 4);
    Serial.println(usfsmax.finalMagCal.V[1], 4);
    Serial.println(usfsmax.finalMagCal.V[2], 4);
    Serial.println("");
    Serial.println("Magnetometer Fine Calibration/Alignment Tensor");
    Serial.print(usfsmax.finalMagCal.invW[0][0], 4);
    Serial.print(",");
    Serial.print(usfsmax.finalMagCal.invW[0][1], 4);
    Serial.print(",");
    Serial.println(usfsmax.finalMagCal.invW[0][2], 4);
    Serial.print(usfsmax.finalMagCal.invW[1][0], 4);
    Serial.print(",");
    Serial.print(usfsmax.finalMagCal.invW[1][1], 4);
    Serial.print(",");
    Serial.println(usfsmax.finalMagCal.invW[1][2], 4);
    Serial.print(usfsmax.finalMagCal.invW[2][0], 4);
    Serial.print(",");
    Serial.print(usfsmax.finalMagCal.invW[2][1], 4);
    Serial.print(",");
    Serial.println(usfsmax.finalMagCal.invW[2][2], 4);
    Serial.println("\n");

} // printCalibrationResults

static void printSpreadsheetHeader(void)
{
    Serial.print("Time");        
    Serial.print(","); 
    Serial.print("Heading (deg)"); 
    Serial.print(",");
    Serial.print("Pitch (deg)"); 
    Serial.print(","); 
    Serial.print("Roll (deg)");   
    Serial.print(",");
    Serial.print("Cal Status\n");  
}

static void error(uint8_t status)
{
    while (true) {
        Serial.print("Got error ");
        Serial.println(status);
        delay(500);
    }
}

static void reportCurrentData(float accData[3], float gyroData[3], float magData[3], 
        int32_t baroADC, float quat[4], float angle[2], float heading)
{
    Serial.print("ax = ");
    Serial.print((int)(1000.0f*accData[0]));
    Serial.print(" ay = ");
    Serial.print((int)(1000.0f*accData[1]));
    Serial.print(" az = ");
    Serial.print((int)(1000.0f*accData[2]));
    Serial.println(" mg");
    Serial.print("gx = ");
    Serial.print(gyroData[0], 1);
    Serial.print(" gy = ");
    Serial.print(gyroData[1], 1); 
    Serial.print(" gz = ");
    Serial.print(gyroData[2], 1);
    Serial.println(" deg/s");
    Serial.print("mx = ");
    Serial.print(magData[0], 1);
    Serial.print(" my = ");
    Serial.print(magData[1], 1);
    Serial.print(" mz = ");
    Serial.print(magData[2], 1);
    Serial.println(" uT");
    Serial.print("Tomasch Xh, Yh: ");
    Serial.print(usfsmax.Mx, 2);
    Serial.print(", ");
    Serial.print(usfsmax.My, 2);
    Serial.println(" uT");
    Serial.print("Baro pressure = ");
    Serial.print(((float)baroADC)/4096.0f);
    Serial.println(" hPa");
    Serial.println("");
    Serial.print("USFSMAX Quat: ");
    Serial.print("q0 = ");
    Serial.print(quat[0], 4);
    Serial.print(" qx = ");
    Serial.print(quat[1], 4);
    Serial.print(" qy = ");
    Serial.print(quat[2], 4); 
    Serial.print(" qz = ");
    Serial.print(quat[3], 4);
    Serial.println("");

    // Euler angles
    Serial.print("USFSMAX Yaw, Pitch, Roll: ");
    Serial.print(heading, 2);
    Serial.print(", "); 
    Serial.print(angle[1], 2); 
    Serial.print(", "); 
    Serial.println(angle[0], 2);

} // reportCurrentData

static void dumpMagData(float magData[3])
{
    Serial.print("Raw:");
    Serial.print(0);                                   // MotionCal GUI doesn't act upon accel/gyro input; send null data
    Serial.print(',');
    Serial.print(0);
    Serial.print(',');
    Serial.print(0);
    Serial.print(',');
    Serial.print(0);
    Serial.print(',');
    Serial.print(0);
    Serial.print(',');
    Serial.print(0);
    Serial.print(',');
    Serial.print((int16_t)(magData[0]*10.0f));      // The MotionCal GUI is expecting 0.1uT/LSB
    Serial.print(',');
    Serial.print((int16_t)(magData[1]*10.0f));
    Serial.print(',');
    Serial.print((int16_t)(magData[2]*10.0f));
    Serial.println();
}

void setup()
{
    // Open serial port
    Serial.begin(115200);
    delay(2000);

    // Set up DRDY interrupt pin
    pinMode(INTERRUPT_PIN, INPUT);

    // Assign Indicator LED
    alarms.begin();
    alarms.blueLEDoff();

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

    if (SERIAL_DEBUG) {

        Serial.println("Configuring the coprocessor...");

        if (status) {
            error(status);
        }

        else {
            Serial.print("Coprocessor configured! Reading sensor calibrations...\n");
        }
    }

    usfsmax.readSensorCalibrations();

    if (SERIAL_DEBUG) {
        printCalibrationResults();
    }

    Wire.setClock(I2C_CLOCK);// Set the I2C clock to high speed for run-mode data collection
    delay(100);

    // Attach interrupts
    attachInterrupt(INTERRUPT_PIN, DRDY_handler, RISING);           // Attach DRDY interrupt

    if (SERIAL_DEBUG) {
        Serial.println("USFXMAX successfully initialized!\n");
        sensorCal.sendOneToProceed();                            // Halt the serial monitor to let the user read the results
    }

    else if (!MOTION_CAL_GUI_ENABLED) { // Print header for spreadsheet data collection
        printSpreadsheetHeader();
    }

    startTime = micros();                                     // Set sketch start time

} // setup

void loop()
{
    int32_t  baroADC;

    static uint32_t lastRefresh;

    float gyroData[3] = {};
    float accData[3] = {};
    float magData[3] = {};

    float quat[4];

    static float angle[2];
    static float heading;

    if (dataReady) {

        dataReady = false;

        // Get the new data from the USFSMAX, and run our alternate quaternion IMU if we have new gyro data
        fetchUsfsmaxData(gyroData, accData, magData, quat, angle, heading, baroADC);
    }

    // Update serial output
    uint32_t deltaT = millis() - lastRefresh;
    if (deltaT > UPDATE_PERIOD)  {                               // Update the serial monitor every "UPDATE_PERIOD" ms

        lastRefresh = millis();
        usfsmax.getMxMy(UT_PER_COUNT);                        // Get Horizontal magnetic components

        if (SERIAL_DEBUG) {
            serialInterfaceHandler();
            if (ENABLE_DHI_CORRECTOR) {
                uint8_t calStatus = usfsmax.getCalibrationStatus();// Poll calibration status byte
                usfsmax.getDHIRsq();                                               // Get DHI R-square
                Serial.print("Dynamic Hard Iron Correction Valid = ");
                Serial.println(calStatus & 0x80);                                 // DHI correction status
                Serial.print("Dynamic Hard Iron Fit R-square = ");
                Serial.println(usfsmax.Rsq, 4);
                Serial.println(USE_2D_DHI_CORRECTOR ? "Using the 2D Corrector\n" : "Using the 3D Corrector");
            } else {
                Serial.print("Dynamic Hard Iron Correction Disabled!\n\n");
            }

            // usfsmax sensor and raw quaternion outout
            reportCurrentData(accData, gyroData, magData, baroADC, quat, angle, heading);
        }

        // Output formatted MotionCal GUI magnetometer data message when
        // "MOTION_CAL_GUI_ENABLED" is defined and "SERIAL_DEBUG" is not defined 
        // https://www.pjrc.com/store/prop_shield.html
        else if (MOTION_CAL_GUI_ENABLED) {
            dumpMagData(magData);
        }

        // Toggle LED if not calibrating gyroscopes
        if (sensorCal.gyroCalActive[0]) {
            alarms.blueLEDoff();
            if ((usfsmax.getCalibrationStatus() & 0x01) == 0) {
                sensorCal.gyroCalActive[0] = false;
            }
        } else {
            alarms.toggle_blueLED();
        }
        dataReady = false;
    }

    delay(5);

} // loop
