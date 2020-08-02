/* 
   MasterTest.cpp: Example sketch for running USFS SENtral sensor hub in master mode 

   Copyright (c) 2018 Simon D. Levy

   This file is part of USFS.

   USFS is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   USFS is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with USFS.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "USFS_Master.h"

#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <math.h>

static const uint8_t  MAG_RATE       = 100;  // Hz
static const uint16_t ACCEL_RATE     = 200;  // Hz
static const uint16_t GYRO_RATE      = 200;  // Hz
static const uint8_t  BARO_RATE      = 50;   // Hz
static const uint8_t  Q_RATE_DIVISOR = 3;    // 1/3 gyro rate
 
USFS_Master usfs = USFS_Master(MAG_RATE, ACCEL_RATE, GYRO_RATE, BARO_RATE, Q_RATE_DIVISOR);

void setup()
{
    // Set up the wiringPi library
    if (wiringPiSetup () < 0) {
        fprintf (stderr, "Unable to setup wiringPi: %s\n", strerror (errno));
        exit(1);
    }

    delay(100);

    // Start the USFS in master mode
    if (!usfs.begin()) {

        while (true) {
            fprintf(stderr, "%s\n", usfs.getErrorString());
        }
    }
}

void loop()
{  
    usfs.checkEventStatus();

    if (usfs.gotError()) {
        fprintf(stderr, "ERROR: %s\n", usfs.getErrorString());
        return;
    }

    //  Define output variables from updated quaternion---these are Tait-Bryan
    //  angles, commonly used in aircraft orientation.  In this coordinate
    //  system, the positive z-axis is down toward Earth.  Yaw is the angle
    //  between Sensor x-axis and Earth magnetic North (or true North if
    //  corrected for local declination, looking down on the sensor positive
    //  yaw is counterclockwise.  Pitch is angle between sensor x-axis and
    //  Earth ground plane, toward the Earth is positive, up toward the sky is
    //  negative.  Roll is angle between sensor y-axis and Earth ground plane,
    //  y-axis up is positive roll.  These arise from the definition of the
    //  homogeneous rotation matrix constructed from q.  Tait-Bryan
    //  angles as well as Euler angles are non-commutative; that is, the get
    //  the correct orientation the rotations must be applied in the correct
    //  order which for this configuration is yaw, pitch, and then roll.  For
    //  more see http://en.wikipedia.org/wiki/Conversion_between_q_and_Euler_angles 
    //  which has additional links.

    if (usfs.gotQuaternion()) {

        float qw, qx, qy, qz;

        usfs.readQuaternion(qw, qx, qy, qz);

        float roll  = atan2(2.0f * (qw * qx + qy * qz), qw * qw - qx * qx - qy * qy + qz * qz);
        float pitch = -asin(2.0f * (qx * qz - qw * qy));
        float yaw   = atan2(2.0f * (qx * qy + qw * qz), qw * qw + qx * qx - qy * qy - qz * qz);   

        pitch *= 180.0f / M_PI;
        yaw   *= 180.0f / M_PI; 
        yaw   += 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
        if(yaw < 0) yaw   += 360.0f ; // Ensure yaw stays between 0 and 360
        roll  *= 180.0f / M_PI;

        printf("Quaternion Roll, Pitch, Yaw: %+2.2f, %+2.2f, %+2.2f\n", roll, pitch, yaw);
    }

    if (usfs.gotAccelerometer()) {
        float ax, ay, az;
        usfs.readAccelerometer(ax, ay, az);
        printf("Accel: %+3.3f, %+3.3f, %+3.3f\n", ax, ay, az);
    }

    if (usfs.gotGyrometer()) {
        float gx, gy, gz;
        usfs.readGyrometer(gx, gy, gz);
        printf("Gyro: %+3.3f, %+3.3f, %+3.3f\n", gx, gy, gz);
    }

    // Or define output variable according to the Android system, where
    // heading (0 to 360) is defined by the angle between the y-axis and True
    // North, pitch is rotation about the x-axis (-180 to +180), and roll is
    // rotation about the y-axis (-90 to +90) In this systen, the z-axis is
    // pointing away from Earth, the +y-axis is at the "top" of the device
    // (cellphone) and the +x-axis points toward the right of the device.

    if (usfs.gotBarometer()) 
    {
        float temperature, pressure;

        usfs.readBarometer(pressure, temperature);

        printf("Baro:\n");
        printf("  Altimeter temperature = %2.2f C\n", temperature); 
        printf("  Altimeter pressure = %2.2f mbar\n", pressure); 
        float altitude = (1.0f - powf(pressure / 1013.25f, 0.190295f)) * 44330.0f;
        printf("  Altitude = %5.2f m\n\n", altitude); 
    }
    
    delay(100);
}
