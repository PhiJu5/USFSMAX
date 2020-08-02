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

class USFSMAX
{
    public:

        typedef enum {
            ACCEL_GYRO_ODR_12_5 = 1,
            ACCEL_GYRO_ODR_26,
            ACCEL_GYRO_ODR_52,
            ACCEL_GYRO_ODR_104,
            ACCEL_GYRO_ODR_208,
            ACCEL_GYRO_ODR_416,
            ACCEL_GYRO_ODR_834,
            ACCEL_GYRO_ODR_1660,
            ACCEL_GYRO_ODR_3330,
            ACCEL_GYRO_ODR_6660
        } AccelGyroODR_t;

        typedef enum {
            MAG_ODR_10 = 0,
            MAG_ODR_20,
            MAG_ODR_50,
            MAG_ODR_100
        } MagODR_t;

        typedef enum {
            BARO_ODR_1 = 1,
            BARO_ODR_10,
            BARO_ODR_25,
            BARO_ODR_50,
            BARO_ODR_75
        } BaroODR_t;

        typedef enum {
            QUAT_DIV_1 = 0,
            QUAT_DIV_2,
            QUAT_DIV_3,
            QUAT_DIV_4,
            QUAT_DIV_5,
            QUAT_DIV_6,
            QUAT_DIV_8,
            QUAT_DIV_10,
            QUAT_DIV_16
        } QuatDiv_t;

        typedef enum {

            LSM6DSM_GYRO_LPF_314 = 0,
            LSM6DSM_GYRO_LPF_223, 
            LSM6DSM_GYRO_LPF_167, 
            LSM6DSM_GYRO_LPF_655 

        } LSM6DSMGyroLPF_t;

        typedef enum {

            LSM6DSM_ACC_LPF_ODR_DIV2 = 0,
            LSM6DSM_ACC_LPF_ODR_DIV4,
            LSM6DSM_ACC_LPF_ODR_DIV9,
            LSM6DSM_ACC_LPF_ODR_DIV50,
            LSM6DSM_ACC_LPF_ODR_DIV100,
            LSM6DSM_ACC_LPF_ODR_DIV400

        } LSM6DSMAccLpfODR_t;

        typedef enum {

            LIS2MDL_MAG_LPF_ODR_2 = 0,
            LIS2MDL_MAG_LPF_ODR_4

        } LIS2MDLMagLpfODR_t;

        typedef enum {

            LPS22HB_BARO_LPF_ODR_2  = 0,
            LPS22HB_BARO_LPF_ODR_9  = 8,
            LPS22HB_BARO_LPF_ODR_20 = 12

        } LPS22HBBaroLpfODR_t;

        typedef enum {

            ACC_SCALE_2 = 0,
            ACC_SCALE_16,
            ACC_SCALE_4,
            ACC_SCALE_8

        } AccScale_t;

        typedef enum {

            GYRO_SCALE_125  = 2,
            GYRO_SCALE_250  = 0,
            GYRO_SCALE_500  = 4,
            GYRO_SCALE_1000 = 8,
            GYRO_SCALE_2000 = 12

        } GyroScale_t;

        typedef enum {
            DATA_READY_NONE = 0,
            DATA_READY_GYRO_ACC,
            DATA_READY_GYRO_ACC_MAG_BARO,
            DATA_READY_MAG_BARO,
            DATA_READY_MAG,
            DATA_READY_BARO

        }  DataReady_t;

        USFSMAX(const AccelGyroODR_t accelODR, 
                const AccelGyroODR_t gyroODR,
                const MagODR_t magODR,
                const BaroODR_t baroODR,
                const QuatDiv_t quatDiv,
                const LSM6DSMGyroLPF_t lsm6dsmGyroLPF,
                const LSM6DSMAccLpfODR_t lsm6dsmGyroLpfODR,
                const AccScale_t accScale,
                const GyroScale_t gyroScale,
                const LIS2MDLMagLpfODR_t lis2mdlMagLpfODR,
                const LPS22HBBaroLpfODR_t lps22hbBaroLpfODR,
                const float magV,
                const float magH,
                const float magDec,
                const bool enableDHICorrector=true,
                const bool use2DDHICorrector=false,
                const bool eulerQuatFlag=false,
                const bool scaledSensorDataFlag=true);

        uint8_t begin(uint8_t bus=1);

        DataReady_t dataReadyFlags(void);

        bool quaternionReady(void);

        void readGyroAcc(float gyro[3], float acc[3]);

        void readGyroAccMagBaro(float gyro[3], float acc[3], float mag[3], float & baro);

        void readMagBaro(float mag[3], float & baro);

        void readMag(float mag[3]);

        void readBaro(float & baro);

        void readQuat(float quat[4]);

        // -------------------------------------------------------------------------

        typedef struct
        {
            float V[3];          // Offset vector components in physical units
            float invW[3][3];    // Inverse calibration matrix
            uint8_t calGood;     // Byte to verify valid cal is in EEPROM
        } fullAdvCal_t;          // IMU conversion factors

        float gPerCount;
        float dpsPerCount;

        fullAdvCal_t gyroCal;
        fullAdvCal_t ellipsoidMagCal;
        fullAdvCal_t accelCal;
        fullAdvCal_t finalMagCal;

        float Rsq = 0.0f;

        float Mx, My;  // Tilt-corrected horizontal magnetic components

        uint8_t getCalibrationStatus(void);

        void    getBaroADC(int32_t & baroADC);
        void    getGyroAccelADC(int16_t  gyroADC[3], int16_t accADC[3]);
        void    getGyroAccelMagBaroADC(int16_t  gyroADC[3], int16_t accADC[3], int16_t magADC[3], int32_t & baroADC);
        void    getGyroADC(int16_t  gyroADC[3]);
        void    getAccADC(int16_t accADC[3]);
        void    getMagADC(int16_t magADC[3]);
        void    getMagBaroADC(int16_t magADC[3], int32_t & baroADC);
        void    getLinAccADC();

        void    getMxMy(const float utPerCount);

        void    getQuat(float quat[4]);
        void    getQuatLin(float quat[4]);

        void    getDHIRsq();

        void    readSensorCalibrations(void);

        void    resetDHI();

        void    retrieveFullAccelCal();
        void    retrieveEllipMagCal();
        void    retrieveFinalMagCal();
        void    retrieveFullGyroCal();

        void    sendGyroCalibrationRequest(void);

    private:

        static const uint8_t MAX32660_ADDR = 0x57;

        // Registers
        static const uint8_t SENS_ERR_STAT        = 0x00;
        static const uint8_t CALIBRATION_STATUS   = 0x01;
        static const uint8_t ACCEL_CAL_POS        = 0x02;
        static const uint8_t FUSION_STATUS        = 0x03;
        static const uint8_t COMBO_DRDY_STAT      = 0x04;
        static const uint8_t G_X_L                = 0x05;
        static const uint8_t G_X_H                = 0x06;
        static const uint8_t G_Y_L                = 0x07;
        static const uint8_t G_Y_H                = 0x08;
        static const uint8_t G_Z_L                = 0x09;
        static const uint8_t G_Z_H                = 0x0A;
        static const uint8_t A_X_L                = 0x0B;
        static const uint8_t A_X_H                = 0x0C;
        static const uint8_t A_Y_L                = 0x0D;
        static const uint8_t A_Y_H                = 0x0E;
        static const uint8_t A_Z_L                = 0x0F;
        static const uint8_t A_Z_H                = 0x10;
        static const uint8_t M_X_L                = 0x11;
        static const uint8_t M_X_H                = 0x12;
        static const uint8_t M_Y_L                = 0x13;
        static const uint8_t M_Y_H                = 0x14;
        static const uint8_t M_Z_L                = 0x15;
        static const uint8_t M_Z_H                = 0x16;
        static const uint8_t BARO_XL              = 0x17;
        static const uint8_t BARO_L               = 0x18;
        static const uint8_t BARO_H               = 0x19;
        static const uint8_t Q0_BYTE0             = 0x1A;
        static const uint8_t Q0_BYTE1             = 0x1B;
        static const uint8_t Q0_BYTE2             = 0x1C;
        static const uint8_t Q0_BYTE3             = 0x1D;
        static const uint8_t Q1_BYTE0             = 0x1E;
        static const uint8_t Q1_BYTE1             = 0x1F;
        static const uint8_t Q1_BYTE2             = 0x20;
        static const uint8_t Q1_BYTE3             = 0x21;
        static const uint8_t Q2_BYTE0             = 0x22;
        static const uint8_t Q2_BYTE1             = 0x23;
        static const uint8_t Q2_BYTE2             = 0x24;
        static const uint8_t Q2_BYTE3             = 0x25;
        static const uint8_t Q3_BYTE0             = 0x26;
        static const uint8_t Q3_BYTE1             = 0x27;
        static const uint8_t Q3_BYTE2             = 0x28;
        static const uint8_t Q3_BYTE3             = 0x29;
        static const uint8_t LIN_X_L              = 0x2A;
        static const uint8_t LIN_X_H              = 0x2B;
        static const uint8_t LIN_Y_L              = 0x2C;
        static const uint8_t LIN_Y_H              = 0x2D;
        static const uint8_t LIN_Z_L              = 0x2E;
        static const uint8_t LIN_Z_H              = 0x2F;
        static const uint8_t GRAV_X_L             = 0x30;
        static const uint8_t GRAV_X_H             = 0x31;
        static const uint8_t GRAV_Y_L             = 0x32;
        static const uint8_t GRAV_Y_H             = 0x33;
        static const uint8_t GRAV_Z_L             = 0x34;
        static const uint8_t GRAV_Z_H             = 0x35;
        static const uint8_t YAW_BYTE0            = 0x36;
        static const uint8_t YAW_BYTE1            = 0x37;
        static const uint8_t YAW_BYTE2            = 0x38;
        static const uint8_t YAW_BYTE3            = 0x39;
        static const uint8_t PITCH_BYTE0          = 0x3A;
        static const uint8_t PITCH_BYTE1          = 0x3B;
        static const uint8_t PITCH_BYTE2          = 0x3C;
        static const uint8_t PITCH_BYTE3          = 0x3D;
        static const uint8_t ROLL_BYTE0           = 0x3E;
        static const uint8_t ROLL_BYTE1           = 0x3F;
        static const uint8_t ROLL_BYTE2           = 0x40;
        static const uint8_t ROLL_BYTE3           = 0x41;
        static const uint8_t AG_TEMP_L            = 0x42;
        static const uint8_t AG_TEMP_H            = 0x43;
        static const uint8_t M_TEMP_L             = 0x44;
        static const uint8_t M_TEMP_H             = 0x45;
        static const uint8_t B_TEMP_L             = 0x46;
        static const uint8_t B_TEMP_H             = 0x47;
        static const uint8_t AUX_1_X_L            = 0x48;
        static const uint8_t AUX_1_X_H            = 0x49;
        static const uint8_t AUX_1_Y_L            = 0x4A;
        static const uint8_t AUX_1_Y_H            = 0x4B;
        static const uint8_t AUX_1_Z_L            = 0x4C;
        static const uint8_t AUX_1_Z_H            = 0x4D;
        static const uint8_t AUX_2_X_L            = 0x4E;
        static const uint8_t AUX_2_X_H            = 0x4F;
        static const uint8_t AUX_2_Y_L            = 0x50;
        static const uint8_t AUX_2_Y_H            = 0x51;
        static const uint8_t AUX_2_Z_L            = 0x52;
        static const uint8_t AUX_2_Z_H            = 0x53;
        static const uint8_t AUX_3_X_L            = 0x54;
        static const uint8_t AUX_3_X_H            = 0x55;
        static const uint8_t AUX_3_Y_L            = 0x56;
        static const uint8_t AUX_3_Y_H            = 0x57;
        static const uint8_t AUX_3_Z_L            = 0x58;
        static const uint8_t AUX_3_Z_H            = 0x59;
        static const uint8_t MX_L                 = 0x5A;
        static const uint8_t MX_H                 = 0x5B;
        static const uint8_t MY_L                 = 0x5C;
        static const uint8_t MY_H                 = 0x5D;
        static const uint8_t DHI_RSQ_L            = 0x5E;
        static const uint8_t DHI_RSQ_H            = 0x5F;
        static const uint8_t FUSION_START_STOP    = 0x60;
        static const uint8_t CALIBRATION_REQUEST  = 0x61;
        static const uint8_t COPRO_CFG_DATA0      = 0x62;
        static const uint8_t COPRO_CFG_DATA1      = 0x63;
        static const uint8_t GYRO_CAL_DATA0       = 0x64;
        static const uint8_t GYRO_CAL_DATA1       = 0x65;
        static const uint8_t ACCEL_CAL_DATA0      = 0x66;
        static const uint8_t ACCEL_CAL_DATA1      = 0x67;
        static const uint8_t ELLIP_MAG_CAL_DATA0  = 0x68;
        static const uint8_t ELLIP_MAG_CAL_DATA1  = 0x69;
        static const uint8_t FINE_MAG_CAL_DATA0   = 0x6A;
        static const uint8_t FINE_MAG_CAL_DATA1   = 0x6B;

        static const uint8_t FUSION_RUNNING_MASK  = 0x01;
        static const uint8_t HI_CORRECTOR_MASK    = 0x10;

        typedef struct
        {
            uint16_t cal_points;
            uint8_t  Ascale;
            uint8_t  AODR;
            uint8_t  Alpf;
            uint8_t  Ahpf;
            uint8_t  Gscale;
            uint8_t  GODR;
            uint8_t  Glpf;
            uint8_t  Ghpf;
            uint8_t  Mscale;
            uint8_t  MODR;
            uint8_t  Mlpf;
            uint8_t  Mhpf;
            uint8_t  Pscale;
            uint8_t  PODR;
            uint8_t  Plpf;
            uint8_t  Phpf;
            uint8_t  AUX1scale;
            uint8_t  AUX1ODR;
            uint8_t  AUX1lpf;
            uint8_t  AUX1hpf;
            uint8_t  AUX2scale;
            uint8_t  AUX2ODR;
            uint8_t  AUX2lpf;
            uint8_t  AUX2hpf;
            uint8_t  AUX3scale;
            uint8_t  AUX3ODR;
            uint8_t  AUX3lpf;
            uint8_t  AUX3hpf;
            float    m_v;
            float    m_h;
            float    m_dec;
            uint8_t  quat_div;
        } CoProcessorConfig_t;

        // Number or data points collected for gyro and accel/fine mag calibrations
        static const uint16_t CAL_POINTS = 2048;     

        // Number of screen updates to be averaged for AHRS summary data
        static const uint16_t AHRS_AVERAGING_POINTS = 300;      

        // Cross-platform I^2C support
        uint8_t  _i2c;

        // Passed to constructor
        bool     _enableDHICorrector;
        bool     _use2DDHICorrector;
        bool     _serialDebug;

        AccelGyroODR_t _accelODR;
        AccelGyroODR_t _gyroODR;
        MagODR_t       _magODR;
        BaroODR_t      _baroODR;
        QuatDiv_t      _quatDiv;

        bool _eulerQuatFlag; 
        bool _scaledSensorDataFlag;

        LSM6DSMGyroLPF_t   _lsm6dsmGyroLPF;
        LSM6DSMAccLpfODR_t _lsm6dsmGyroLpfODR;

        LIS2MDLMagLpfODR_t _lis2mdlMagLpfODR;

        LPS22HBBaroLpfODR_t _lps22hbBaroLpfODR;

        AccScale_t  _accScale;
        GyroScale_t _gyroScale;

        float _magV;
        float _magH;
        float _magDec;

        int16_t _accLin[3];  // linear acceleration
        int16_t _grav[3];

        CoProcessorConfig_t _Cfg[2];
        uint8_t _cfg_buff[sizeof(CoProcessorConfig_t)];

        float uint32_reg_to_float (uint8_t *buf);

        uint8_t getDataReadyStatus(void);

        void getIMU_ADC(uint8_t bytes[], int16_t imuADC[3], uint8_t start);
        void getBaro_ADC(uint8_t bytes[], int32_t & baroADC, uint8_t start);

        void uploadConfig(CoProcessorConfig_t Cfg);
        void retrieveConfig();

        uint8_t readRegister(uint8_t address);
        void    writeRegister(uint8_t address, uint8_t data);
        void    readRegisters(uint8_t address, uint8_t count, uint8_t * dst);
        void    writeRegisters(uint8_t address, uint8_t count, uint8_t * src);
};
