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

#include "USFSMAX.h"
#include <CrossPlatformI2C_Core.h>
#include <string.h>

#if defined(ARDUINO)
#include <Arduino.h> // support for delay()
#endif

USFSMAX::USFSMAX(
        const AccelGyroODR_t accelODR,
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
        const bool enableDHICorrector,
        const bool use2DDHICorrector,
        const bool eulerQuatFlag,
        const bool scaledSensorDataFlag)

{
    init(
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
        enableDHICorrector,
        use2DDHICorrector,
        eulerQuatFlag,
        scaledSensorDataFlag);
}

void USFSMAX::init(
        const AccelGyroODR_t accelODR,
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
        const bool enableDHICorrector,
        const bool use2DDHICorrector,
        const bool eulerQuatFlag,
        const bool scaledSensorDataFlag)
{
    _accelODR = accelODR;
    _gyroODR  = gyroODR;
    _magODR   = magODR;
    _baroODR  = baroODR;
    _quatDiv  = quatDiv;

    _enableDHICorrector = enableDHICorrector;
    _use2DDHICorrector = use2DDHICorrector;

    _magV   = magV; 
    _magH   = magH;
    _magDec = magDec;

    _lsm6dsmGyroLPF    = lsm6dsmGyroLPF;
    _lsm6dsmGyroLpfODR = lsm6dsmGyroLpfODR;

    _lis2mdlMagLpfODR = lis2mdlMagLpfODR;

    _lps22hbBaroLpfODR = lps22hbBaroLpfODR;   

    _accScale = accScale;

    _gyroScale = gyroScale;

    constexpr float ACC_SCALE_TO_G_PER_COUNT[4] = {0.0000610f, 0.0004880f, 0.0001220f, 0.0002440f};
    gPerCount = ACC_SCALE_TO_G_PER_COUNT[accScale];

    switch (gyroScale) {
        case GYRO_SCALE_125:
            dpsPerCount = 0.004375f;
            break;
        case GYRO_SCALE_250:
            dpsPerCount = 0.00875f;
            break;
        case GYRO_SCALE_500:
            dpsPerCount = 0.0175f;
            break;
        case GYRO_SCALE_1000:
            dpsPerCount = 0.035f;
            break;
        case GYRO_SCALE_2000:
            dpsPerCount = 0.070f;
            break;
    }

    _eulerQuatFlag = eulerQuatFlag; 
    _scaledSensorDataFlag = scaledSensorDataFlag;
}

USFSMAX::USFSMAX(void)
{
}

uint8_t USFSMAX::begin(uint8_t bus)
{
    _i2c = cpi2c_open(MAX32660_ADDR, bus);

    uint8_t status = readRegister(FUSION_STATUS);     // Read the coprocessor's current fusion status

    delay(100);

    if (status == 0) {

        writeRegister(FUSION_START_STOP, 0x00); // Stop sensor fusion

        delay(100);

        // Upload configuration structure variable
        USFSMAX::uploadConfig(_Cfg[0]);

        // Re-start sensor fusion: set bit0 to re-start fusion; adjust bit1, bit2 for desired output options
        writeRegister(FUSION_START_STOP, 
                ((0x01 | ((uint8_t)_eulerQuatFlag) << 1) | ((uint8_t)_scaledSensorDataFlag) << 2));

        delay(100);

        // Poll the FUSION_STATUS register to see if fusion has resumed
        while (true) {
            delay(10);
            status = readRegister(FUSION_STATUS);
            if((status & FUSION_RUNNING_MASK)) {
                break;
            }
        }
    }

    // Check for sensor errors
    status = readRegister(SENS_ERR_STAT);

    if (status !=0) {
        return status;
    }

    if (_enableDHICorrector) {
        if(_use2DDHICorrector) {
            writeRegister(CALIBRATION_REQUEST, 0x50); // Enable DHI corrector, 2D (0x10|0x50)
        } else {
            writeRegister(CALIBRATION_REQUEST, 0x10); // Enable DHI corrector, 3D (0x10)
        }
    }

    delay(100);

    return status;
}

USFSMAX::DataReady_t USFSMAX::dataReady(void)
{
    // Optimize the I2C read function with respect to whatever sensor data is ready
    switch(getDataReadyStatus() & 0x0F) {
        case 0x01:
        case 0x02:
        case 0x03:
            return DATA_READY_GYRO_ACC;
        case 0x07:
        case 0x0B:
        case 0x0F:
            return DATA_READY_GYRO_ACC_MAG_BARO;
        case 0x0C:
            return DATA_READY_MAG_BARO;
        case 0x04:
            return DATA_READY_MAG;
            break;
        case 0x08:
            return DATA_READY_BARO;
    }

    return DATA_READY_NONE;
}

bool USFSMAX::quaternionReady(void)
{
    return (bool)(getDataReadyStatus() & 0x10);
}

// ------------------------------------------------------------------------------------------

void USFSMAX::readSensorCalibrations(void)
{
    retrieveFullGyroCal();
    delay(100);
    USFSMAX::retrieveFullAccelCal();
    delay(100);
    USFSMAX::retrieveEllipMagCal();
    delay(100);
    USFSMAX::retrieveFinalMagCal();
    delay(500);
}

void USFSMAX::uploadConfig(CoProcessorConfig_t Config)
{
    uint8_t CmdByte;

    CmdByte = 0x08;  // Clears bit0 to stop fusion an sets bit3 to specify configuration uplaod
    writeRegister(FUSION_START_STOP, CmdByte);
    delay(1000);

    // Assign configuration values
    Config.cal_points        = CAL_POINTS;
    Config.Ascale            = _accScale;
    Config.AODR              = _accelODR;
    Config.Alpf              = _lsm6dsmGyroLpfODR;
    Config.Ahpf              = 0; // future option
    Config.Gscale            = _gyroScale;
    Config.GODR              = _gyroODR;
    Config.Glpf              = _lsm6dsmGyroLPF;
    Config.Ghpf              = 0; // future option
    Config.Mscale            = 0; // not adjustable
    Config.MODR              = _magODR;
    Config.Mlpf              = _lis2mdlMagLpfODR;
    Config.Mhpf              = 0; // future option
    Config.Pscale            = 0; // not adjustable
    Config.PODR              = _baroODR;
    Config.Plpf              = _lps22hbBaroLpfODR;
    Config.Phpf              = 0; // future option
    Config.AUX1scale         = 0;
    Config.AUX1ODR           = 0;
    Config.AUX1lpf           = 0;
    Config.AUX1hpf           = 0;
    Config.AUX2scale         = 0;
    Config.AUX2ODR           = 0;
    Config.AUX2lpf           = 0;
    Config.AUX2hpf           = 0;
    Config.AUX3scale         = 0;
    Config.AUX3ODR           = 0;
    Config.AUX3lpf           = 0;
    Config.AUX3hpf           = 0;
    Config.m_v               = _magV;
    Config.m_h               = _magH;
    Config.m_dec             = _magDec;
    Config.quat_div          = _quatDiv;

    // Assign config structure to byte array upload
    memcpy(_cfg_buff, &Config, sizeof(CoProcessorConfig_t));

    // Upload configuration bytes
    writeRegisters(COPRO_CFG_DATA0, 30, &_cfg_buff[0]);
    delay(100);
    writeRegisters(COPRO_CFG_DATA1, (sizeof(CoProcessorConfig_t) - 30), &_cfg_buff[30]);
    delay(100);
}

void USFSMAX::getGyroAccelMagBaroADC(int16_t  gyroADC[3], int16_t accADC[3], int16_t magADC[3], int32_t & baroADC)
{
    uint8_t bytes[21];

    readRegisters(G_X_L, 21, bytes);

    getIMU_ADC(bytes, gyroADC, 0);
    getIMU_ADC(bytes, accADC,  6);
    getIMU_ADC(bytes, magADC,  12);

    getBaro_ADC(bytes, baroADC, 18);
}

void USFSMAX::getGyroAccelADC(int16_t  gyroADC[3], int16_t accADC[3])
{
    uint8_t bytes[12];

    readRegisters(G_X_L, 12, bytes);

    getIMU_ADC(bytes, gyroADC, 0);
    getIMU_ADC(bytes, accADC,  6);
}

void USFSMAX::getMagBaroADC(int16_t magADC[3], int32_t & baroADC)
{
    uint8_t bytes[9];

    readRegisters(M_X_L, 9, bytes);

    getIMU_ADC(bytes, magADC, 0);

    getBaro_ADC(bytes, baroADC, 6);
}

void USFSMAX::getGyroADC(int16_t  gyroADC[3])
{
    uint8_t bytes[6];

    readRegisters(G_X_L, 6, bytes);

    getIMU_ADC(bytes, gyroADC, 0);
}

void USFSMAX::getAccADC(int16_t accADC[3])
{
    uint8_t bytes[6];

    readRegisters(A_X_L, 6, bytes);

    getIMU_ADC(bytes, accADC, 0);
}

void USFSMAX::getMagADC(int16_t magADC[3])
{
    uint8_t bytes[6];

    readRegisters(M_X_L, 6, bytes);

    getIMU_ADC(bytes, magADC, 0);
}

void USFSMAX::getBaroADC(int32_t & baroADC)
{
    uint8_t bytes[3];

    readRegisters(BARO_XL, 3, bytes);

    getBaro_ADC(bytes, baroADC, 0);
}

void USFSMAX::getMxMy(void)
{
    uint8_t bytes[4];
    int16_t H[2];

    readRegisters(MX_L, 4, bytes);

    H[0] = ((int16_t)bytes[1] << 8) | bytes[0];
    H[1] = ((int16_t)bytes[3] << 8) | bytes[2];

    Mx = H[0] * MAG_UT_PER_COUNT;
    My = H[1] * MAG_UT_PER_COUNT;
}

void USFSMAX::getQuat(float quat[4])
{
    uint8_t bytes[16];

    readRegisters(Q0_BYTE0, 16, bytes);
    quat[0] = uint32_reg_to_float (&bytes[0]);
    quat[1] = uint32_reg_to_float (&bytes[4]);
    quat[2] = uint32_reg_to_float (&bytes[8]);
    quat[3] = uint32_reg_to_float (&bytes[12]);
}

void USFSMAX::getQuatLin(float quat[4])
{
    uint8_t bytes[28];

    readRegisters(Q0_BYTE0, 28, bytes);
    quat[0] = uint32_reg_to_float (&bytes[0]);
    quat[1] = uint32_reg_to_float (&bytes[4]);
    quat[2] = uint32_reg_to_float (&bytes[8]);
    quat[3] = uint32_reg_to_float (&bytes[12]);
    _accLin[0] = ((int16_t)bytes[17] << 8) | bytes[16];
    _accLin[1] = ((int16_t)bytes[19] << 8) | bytes[18];
    _accLin[2] = ((int16_t)bytes[21] << 8) | bytes[20];
    _grav[0]   = ((int16_t)bytes[23] << 8) | bytes[22];
    _grav[1]   = ((int16_t)bytes[25] << 8) | bytes[24];
    _grav[2]   = ((int16_t)bytes[27] << 8) | bytes[26];
}

void USFSMAX::getLinAccADC()
{
    uint8_t bytes[12];

    readRegisters(LIN_X_L, 12, bytes);
    _accLin[0] = ((int16_t)bytes[1] << 8) | bytes[0];
    _accLin[1] = ((int16_t)bytes[3] << 8) | bytes[2];
    _accLin[2] = ((int16_t)bytes[5] << 8) | bytes[4];
    _grav[0]   = ((int16_t)bytes[7] << 8) | bytes[6];
    _grav[1]   = ((int16_t)bytes[9] << 8) | bytes[8];
    _grav[2]   = ((int16_t)bytes[11] << 8) | bytes[10];
}

void USFSMAX::getDHIRsq()
{
    uint8_t bytes[2];

    readRegisters(DHI_RSQ_L, 2, bytes);
    Rsq = ((float)((int16_t)bytes[1] << 8 | bytes[0]))/10000.0f;
}

void USFSMAX::resetDHI()
{
    writeRegister(CALIBRATION_REQUEST, _use2DDHICorrector ? 0x70 : 0x30);
}

void USFSMAX::retrieveConfig()
{
    readRegisters(COPRO_CFG_DATA0, 30, &_cfg_buff[0]);
    delay(100);
    readRegisters(COPRO_CFG_DATA1, (sizeof(CoProcessorConfig_t) - 30), &_cfg_buff[30]);
    memcpy(&_Cfg[0], _cfg_buff, sizeof(CoProcessorConfig_t));
}

void USFSMAX::retrieveFullAccelCal()
{
    uint8_t AccelCal_buff[sizeof(fullAdvCal_t)];
    readRegisters(ACCEL_CAL_DATA0, 30, &AccelCal_buff[0]);
    delay(100);
    readRegisters(ACCEL_CAL_DATA1, (sizeof(fullAdvCal_t) - 30), &AccelCal_buff[30]);
    memcpy(&accelCal, AccelCal_buff, sizeof(fullAdvCal_t));
}

void USFSMAX::retrieveEllipMagCal()
{
    uint8_t EllipMagCal_buff[sizeof(fullAdvCal_t)];
    readRegisters(ELLIP_MAG_CAL_DATA0, 30, &EllipMagCal_buff[0]);
    delay(100);
    readRegisters(ELLIP_MAG_CAL_DATA1, (sizeof(fullAdvCal_t) - 30), &EllipMagCal_buff[30]);
    memcpy(&ellipsoidMagCal, EllipMagCal_buff, sizeof(fullAdvCal_t));
}

void USFSMAX::retrieveFinalMagCal()
{
    uint8_t FineMagCal_buff[sizeof(fullAdvCal_t)];
    readRegisters(FINE_MAG_CAL_DATA0, 30, &FineMagCal_buff[0]);
    delay(100);
    readRegisters(FINE_MAG_CAL_DATA1, (sizeof(fullAdvCal_t) - 30), &FineMagCal_buff[30]);
    memcpy(&finalMagCal, FineMagCal_buff, sizeof(fullAdvCal_t));
}

void USFSMAX::retrieveFullGyroCal()
{
    uint8_t GyroCal_buff[sizeof(fullAdvCal_t)];
    readRegisters(GYRO_CAL_DATA0, 30, &GyroCal_buff[0]);
    delay(100);
    readRegisters(GYRO_CAL_DATA1, (sizeof(fullAdvCal_t) - 30), &GyroCal_buff[30]);
    memcpy(&gyroCal, GyroCal_buff, sizeof(fullAdvCal_t));
}

float USFSMAX::uint32_reg_to_float (uint8_t *buf)
{
    union
    {
        uint32_t ui32;
        float f;
    } u;

    u.ui32 = (((uint32_t)buf[0]) +
            (((uint32_t)buf[1]) <<  8) +
            (((uint32_t)buf[2]) << 16) +
            (((uint32_t)buf[3]) << 24));
    return u.f;
}

void USFSMAX::getIMU_ADC(uint8_t bytes[], int16_t imuADC[3], uint8_t start)
{
    imuADC[0] = ((int16_t)bytes[start+1] << 8) | bytes[start];
    imuADC[1] = ((int16_t)bytes[start+3] << 8) | bytes[start+2];
    imuADC[2] = ((int16_t)bytes[start+5] << 8) | bytes[start+4];
}

void USFSMAX::getBaro_ADC(uint8_t bytes[], int32_t & baroADC, uint8_t start)
{
    baroADC = (int32_t)bytes[start+2] << 16 | (int32_t)bytes[start+1] << 8 | bytes[start];
}

uint8_t USFSMAX::getCalibrationStatus(void)
{
    return readRegister(CALIBRATION_STATUS);
}

uint8_t USFSMAX::getDataReadyStatus(void)
{
    return readRegister(COMBO_DRDY_STAT);
}

void USFSMAX::sendGyroCalibrationRequest(void)
{
    writeRegister(CALIBRATION_REQUEST, 0x01);
}

// General I^2C ----------------------------------------------------------------

uint8_t USFSMAX::readRegister(uint8_t address)
{
    uint8_t data;
    readRegisters(address, 1, &data);
    return data;
}

void USFSMAX::writeRegister(uint8_t address, uint8_t data)
{
    cpi2c_writeRegister(MAX32660_ADDR, address, data);
}

void USFSMAX::readRegisters(uint8_t address, uint8_t count, uint8_t * dst)
{
    cpi2c_readRegisters(MAX32660_ADDR, address, count, dst);
}

void USFSMAX::writeRegisters(uint8_t address, uint8_t count, uint8_t * src)
{
    cpi2c_writeRegisters(MAX32660_ADDR, address, count, src);
}
