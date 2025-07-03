// +-------------------------------------------------------------
//
// Equipment:
// DOIT ESP32 Dev Kit V1, MPU-6050, OLED SSD1306, WS2812B Addressable LEDS x3,
// 4-relay module, 2 linear actuators, RT-11(JCHT35K9) remote/handset,
// and 650W 12V Power supply
//
// File: calibrateData.h
//
// Description:
//
// Provides the one way to find the calibration requirements for the
// MPU-6050, and the linear actuators to control the movement of the standup
// portion of the workstation.
//
// History:     6-Mar-2024     Scarecrow1965   Created
//
// +-------------------------------------------------------------

#ifndef CALIBRATEDATA_H
#define CALIBRATEDATA_H

#include <Arduino.h>
#include <Wire.h>

// FROM: https://wired.chillibasket.com/2015/01/calibrating-mpu6050/

// Arduino sketch that returns calibration offsets for MPU6050
//   Version 1.1  (31th January 2014)
// Done by Luis Ródenas <luisrodenaslorda@gmail.com>
// Based on the I2Cdev library and previous work by Jeff Rowberg <jeff@rowberg.net>
// Updates (of the library) should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib

// These offsets were meant to calibrate MPU6050's internal DMP, but can be also useful for reading sensors.
// The effect of temperature has not been taken into account so I can't promise that it will work if you
// calibrate indoors and then use it outdoors. Best is to calibrate and use at the same room temperature.

/* ==========  LICENSE  ==================================
 I2Cdev device library code is placed under the MIT license
 Copyright (c) 2011 Jeff Rowberg

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
 =========================================================
 */

#include <SPI.h>
#include <SD.h>
#include <FS.h>

// I2Cdev and MPU6050 must be installed as libraries
#include <I2Cdev.h>
#include "dataLogger.h" // included in case you want to log the data to a file

#include "ultrasonic.h"         // included in case you want to use the ultrasonic sensor
void getFilteredDistance(void); // get the distance from the ultrasonic sensor

#include "filterData.h"                                                                                                               // included in case you want to filter the data
void loadOffsets(void);                                                                                                               // load the offsets from the config file
void setMPUOffsets(int16_t ax_offset, int16_t ay_offset, int16_t az_offset, int16_t gx_offset, int16_t gy_offset, int16_t gz_offset); // set the offsets for the MPU
void loadDistances(void);                                                                                                             // load the distances from the config file

#include "LinearActuator.h"
void levelTable(void); // level the table

// from linearActuator.h
// template for boolChecker function is:
// bool rt11Working
// String DOWN_PIN
// String UP_PIN
// String isLeftActuator
// String isRightActuator
extern bool isStop;
extern bool isMovingUp;
extern bool isMovingDown;
extern bool isStationary;
extern bool isTableLevel;
extern bool isTableMax;
extern bool isBottomOut;

extern int16_t ax, ay, az, gx, gy, gz;                                                 // from main.cpp
extern bool isCalibrationLevel;                                                        // from main.cpp
extern int16_t accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z;                      // from filterData.h
extern int16_t ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;       // from filterData.h
extern int16_t mean_ax2, mean_ay2, mean_az2, mean_gx2, mean_gy2, mean_gz2;             // from filterData.h
extern int16_t ax_offset2, ay_offset2, az_offset2, gx_offset2, gy_offset2, gz_offset2; // from filterData.h
// extern int8_t ax_filter, ay_filter, az_filter, gx_filter, gy_filter, gz_filter;        // from filterData.h
extern float distanceCm, distanceInch;         // from ultrasonic.h
extern float baseDistanceCm, baseDistanceInch; // from datalogger.h

extern String fileName; // from main.cpp
extern File dataFile;   // from main.cpp
// extern File configFile; // from main.cpp

// delay equivalence
extern unsigned long previousMillis;
extern unsigned long currentMillis;
extern const unsigned int timePeriod002; // delay equal to 2 ms
extern const unsigned int timePeriod005; // delay equal to 5 ms
extern const unsigned int timePeriod010; // Delay equal to 10 ms
extern const unsigned int timePeriod050; // Delay equal to 50 ms
extern const unsigned int timePeriod100; // Delay equal to 100 ms
extern const unsigned int timePeriod250; // Delay equal to 1/4 second
extern const unsigned int timePeriod500; // Delay equal to 1/2 second
extern const unsigned int timePeriod1;   // Delay equal to 1 sec pause
extern const unsigned int timePeriod2;   // Delay equal to 2 second pause
extern const unsigned int timePeriod5;   // Delay equal to 5 seconds pause
extern const unsigned int timePeriod10;  // Delay equal to 10 seconds pause
extern const unsigned int timePeriod15;  // Delay equal to 15 seconds duration
extern const unsigned int timePeriod30;  // Delay equal to 30 second duration

#define USE_LIBRARY2
#ifdef USE_LIBRARY2
// #include <MPU6050.h>

// ================
// this code is not tested yet
// the preferences library is part of the arduino-ESP32 library
// #include <Preferences.h>

// Preferences preferences;
// ================

///////////////////////////////////   CONFIGURATION   /////////////////////////////
// Change this 3 variables if you want to fine tune the sketch to your needs.
int buffersize = 1000;      // Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
uint8_t accel_deadzone = 8; // Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
uint8_t gyro_deadzone = 1;  // Gyro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)

// default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
// MPU6050 MPU2;

// variables for the calibration
int16_t mean_ax = 0, mean_ay = 0, mean_az = 0, mean_gx = 0, mean_gy = 0, mean_gz = 0;
int count = 0;
uint8_t state = 0;
bool isCalibrating = false;
int error = 0;
int ready = 0;

void UnCalibratedlevelTable(void)
{
    float lowestPositive = 999.99;  // initialize to the maximum float value
    float lowestNegative = -0.01; // initialize to the minimum float value
    float tolerance_value = 3.0;

    while (true) // start an infinite loop
    {
        // existing code to update angleX...
        // get the filtered data from the MPU-6050
        filterData();
        // looking to stabilize the X-axis (side to side) of the table
        angleX = kalmanloop(accel_x, gyro_x, calculateXAngle, "X");
        Serial.println("Over angleX is now:\t" + String(angleX)); // used for testing purposes only

        // update lowestPositive and highestNegative
        if (angleX > 0 && angleX < lowestPositive)
        {
            lowestPositive = angleX;
        }
        else if (angleX < 0 && angleX > lowestNegative)
        {
            lowestNegative = angleX;
        }

        if (angleX >= lowestPositive)
        {
            isMovingUp = true;
            isMovingDown = false;
            isStationary = false;

            // Adjust table to correct angle
            moveActuators(false, true);
            // LA2RH -> if false then, move down
            // digitalWrite(relay3_LA2RHP_Pin, HIGH); // redundant protection
            // digitalWrite(relay4_LA2RHM_Pin, LOW);

            delay(20); // delay for 20 milli second to let the table adjust
        }
        else if (angleX <= lowestNegative)
        {
            isMovingDown = true;
            isMovingUp = false;
            isStationary = false;
            // Adjust table to correct angle
            // LA1LH -> if false then, move down
            // digitalWrite(relay1_LA1LHP_Pin, HIGH); // redundant protection
            // digitalWrite(relay2_LA1LHM_Pin, LOW);
            // moveActuators(bool leftUp, bool rightUp)
            moveActuators(true, false);
            delay(20); // delay for 20 milli second to let the table adjust
        }

        // break the loop if angleX is within the new tolerance values
        if (!((angleX >= lowestPositive + tolerance_value && angleX >= lowestPositive - tolerance_value) || (angleX <= lowestNegative + tolerance_value && angleX <= lowestNegative - tolerance_value)))
        {
            Serial.println("lowestPositive is now:\t" + String(lowestPositive)); // used for testing purposes only
            Serial.println("lowestNegative is now:\t" + String(lowestNegative)); // used for testing purposes only

            // ALL STOP
            digitalWrite(relay1_LA1LHP_Pin, HIGH);
            digitalWrite(relay2_LA1LHM_Pin, HIGH);
            digitalWrite(relay3_LA2RHP_Pin, HIGH);
            digitalWrite(relay4_LA2RHM_Pin, HIGH);

            // template for boolChecker function is:
            // bool rt11Working
            // String DOWN_PIN
            // String UP_PIN
            // String isLeftActuator
            // String isRightActuator
            isStop = true;
            isMovingUp = false;
            isMovingDown = false;
            isStationary = true;
            // bool isTableLevel
            // bool isTableMax
            // bool isBottomOut
            isCalibrationLevel = true;
            break; // exit the loop
            // return; // exit the function
        }
    }
}; // end of UnCalibratedlevelTable

//--------------------------------------------//
// Get the mean values from the sensor
//--------------------------------------------//
void meansensors()
{
    long buffer_ax = 0, buffer_ay = 0, buffer_az = 0, buffer_gx = 0, buffer_gy = 0, buffer_gz = 0;
    count = 0;

    while (count < (buffersize + 101))
    {
        // read raw accel/gyro measurements from device
        MPU2.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        // used for testing purposes only
        // Serial.print("MPU Output: AccX: ");
        // Serial.print(ax);
        // Serial.print("\t AccY: ");
        // Serial.print(ay);
        // Serial.print("\t AccZ: ");
        // Serial.print(az);
        // Serial.print("\t GyroX: ");
        // Serial.print(gx);
        // Serial.print("\t GyroY: ");
        // Serial.print(gy);
        // Serial.print("\t GyroZ: ");
        // Serial.print(gz);
        // Serial.print("\t buffer size :");
        // Serial.print(buffersize);
        // Serial.println();

        if (count > 100 && count <= (buffersize + 100))
        {
            // First 100 measures are discarded
            buffer_ax = buffer_ax + ax;
            buffer_ay = buffer_ay + ay;
            buffer_az = buffer_az + az;
            buffer_gx = buffer_gx + gx;
            buffer_gy = buffer_gy + gy;
            buffer_gz = buffer_gz + gz;
        }
        if (count == (buffersize + 100))
        {
            mean_ax = buffer_ax / buffersize;
            mean_ay = buffer_ay / buffersize;
            mean_az = buffer_az / buffersize;
            mean_gx = buffer_gx / buffersize;
            mean_gy = buffer_gy / buffersize;
            mean_gz = buffer_gz / buffersize;
        }
        count++;

        currentMillis = millis();
        if (currentMillis - previousMillis >= timePeriod002)
        {
            // delay(2); // Needed so we don't get repeated measures
            previousMillis = currentMillis;
        }
    }
}; // meansensors

//--------------------------------------------//
// Calibrate sensor
//--------------------------------------------//
void calibration(void)
{
    ax_offset = -mean_ax / 8;
    ay_offset = -mean_ay / 8;
    // az_offset = (16384 - mean_az) / 8; // this is based on setAccelerometerRange(MPU6050_RANGE_2_G) = 16384
    az_offset = (8192 - mean_az) / 8; // this is based on setAccelerometerRange(MPU6050_RANGE_4_G) = 8192

    gx_offset = -mean_gx / 4;
    gy_offset = -mean_gy / 4;
    gz_offset = -mean_gz / 4;

    while (1)
    {
        ready = 0;
        MPU2.setXAccelOffset(ax_offset);
        MPU2.setYAccelOffset(ay_offset);
        MPU2.setZAccelOffset(az_offset);

        MPU2.setXGyroOffset(gx_offset);
        MPU2.setYGyroOffset(gy_offset);
        MPU2.setZGyroOffset(gz_offset);

        // used for testing purposes only
        // Serial.print("MPU Calibration output: AX_offset: ");
        // Serial.print(ax_offset);
        // Serial.print("\t AY_offset: ");
        // Serial.print(ay_offset);
        // Serial.print("\t AZ_offset: ");
        // Serial.print(az_offset);
        // Serial.print("\t GX_offset: ");
        // Serial.print(gx_offset);
        // Serial.print("\t GY_offset: ");
        // Serial.print(gy_offset);
        // Serial.print("\t GZ_offset: ");
        // Serial.print(gz_offset);
        // Serial.println();

        meansensors();

        Serial.println("...");

        if (abs(mean_ax) <= accel_deadzone)
            ready++;
        else
            ax_offset = ax_offset - mean_ax / accel_deadzone;

        if (abs(mean_ay) <= accel_deadzone)
            ready++;
        else
            ay_offset = ay_offset - mean_ay / accel_deadzone;
        // this is based on setAccelerometerRange(MPU6050_RANGE_4_G) = 8192
        // if setAccelerometerRange(MPU6050_RANGE_2_G), then use 16384
        if (abs(8192 - mean_az) <= accel_deadzone)
            ready++;
        else
            az_offset = az_offset + (8192 - mean_az) / accel_deadzone;

        if (abs(mean_gx) <= gyro_deadzone)
            ready++;
        else
            gx_offset = gx_offset - mean_gx / (gyro_deadzone + 1);

        if (abs(mean_gy) <= gyro_deadzone)
            ready++;
        else
            gy_offset = gy_offset - mean_gy / (gyro_deadzone + 1);

        if (abs(mean_gz) <= gyro_deadzone)
            ready++;
        else
            gz_offset = gz_offset - mean_gz / (gyro_deadzone + 1);

        if (ready == 6)
            break; // exit the loop
    }
}; // calibration

// THIS WAS SUGGFESTED BY GIUTHUB AI AND AMENDED TO REFLECT THE FILTER VALUES
void verifyAndSpeedUpCalibration(void)
{
    loadOffsets();
    // used for testing purposes only
    // Serial.println("Offsets_from_file_loaded!");
    // Serial.println("mean_ax2: " + String(mean_ax2));
    // Serial.println("mean_ay2: " + String(mean_ay2));
    // Serial.println("mean_az2: " + String(mean_az2));
    // Serial.println("mean_gx2: " + String(mean_gx2));
    // Serial.println("mean_gy2: " + String(mean_gy2));
    // Serial.println("mean_gz2: " + String(mean_gz2));

    // Serial.println("ax_offset2: " + String(ax_offset2));
    // Serial.println("ay_offset2: " + String(ay_offset2));
    // Serial.println("az_offset2: " + String(az_offset2));
    // Serial.println("gx_offset2: " + String(gx_offset2));
    // Serial.println("gy_offset2: " + String(gy_offset2));
    // Serial.println("gz_offset2: " + String(gz_offset2));

    setMPUOffsets(ax_offset2, ay_offset2, az_offset2, gx_offset2, gy_offset2, gz_offset2);
    // Serial.println("Saved_Offset2s_are_now_into_the_MPU!"); // used for testing purposes only
    logSensorData(7); // logging information before re-calibration
    // used for testing purposes only
    Serial.println("old ax_offset2: " + String(ax_offset2));
    Serial.println("old ay_offset2: " + String(ay_offset2));
    Serial.println("old az_offset2: " + String(az_offset2));
    Serial.println("old gx_offset2: " + String(gx_offset2));
    Serial.println("old gy_offset2: " + String(gy_offset2));
    Serial.println("old gz_offset2: " + String(gz_offset2));

    meansensors();
    // used for testing purposes only
    // Serial.println("Mean_sensor_values:");
    // Serial.println("mean_ax: " + String(mean_ax));
    // Serial.println("mean_ay: " + String(mean_ay));
    // Serial.println("mean_az: " + String(mean_az));
    // Serial.println("mean_gx: " + String(mean_gx));
    // Serial.println("mean_gy: " + String(mean_gy));
    // Serial.println("mean_gz: " + String(mean_gz));

    ax_offset = -mean_ax / 8;
    ay_offset = -mean_ay / 8;
    az_offset = (8192 - mean_az) / 8; // this is based on setAccelerometerRange(MPU6050_RANGE_4_G) = 8192
    gx_offset = -mean_gx / 4;
    gy_offset = -mean_gy / 4;
    gz_offset = -mean_gz / 4;
    // used for testing purposes only
    // Serial.println("ax_Offset: " + String(ax_offset));
    // Serial.println("ay_Offset: " + String(ay_offset));
    // Serial.println("az_Offset: " + String(az_offset));
    // Serial.println("gx_Offset: " + String(gx_offset));
    // Serial.println("gy_Offset: " + String(gy_offset));
    // Serial.println("gz_Offset: " + String(gz_offset));

    Serial.println("Verifying_calibration..."); // used for testing purposes only

    bool isCalibrated = true;

    while (1)
    {
        ready = 0;

        meansensors();
        // used for testing purposes only
        // Serial.println("Mean_sensor_values:");
        // Serial.println("mean_ax: " + String(mean_ax));
        // Serial.println("mean_ay: " + String(mean_ay));
        // Serial.println("mean_az: " + String(mean_az));
        // Serial.println("mean_gx: " + String(mean_gx));
        // Serial.println("mean_gy: " + String(mean_gy));
        // Serial.println("mean_gz: " + String(mean_gz));
        // Serial.println("...");

        if (abs(mean_ax) <= accel_deadzone)
        {
            ready++;
        }
        else
        {
            isCalibrated = false;

            // used for testing purposes only
            // Serial.print("Accel_X_is_not_calibrated-> ");
            // Serial.print("\tmean_ax:\t" + String(mean_ax));
            // Serial.println("\tax_offset2: " + String(ax_offset2));

            ax_offset2 = ax_offset2 - mean_ax / accel_deadzone;
            MPU2.setXAccelOffset(ax_offset2);
        }

        if (abs(mean_ay) <= accel_deadzone)
        {
            ready++;
        }
        else
        {
            isCalibrated = false;

            // used for testing purposes only
            // Serial.print("Accel_Y_is_not_calibrated-> ");
            // Serial.print("\tmean_ay:\t" + String(mean_ay));
            // Serial.println("\tay_offset2:\t" + String(ay_offset2));

            ay_offset2 = ay_offset2 - mean_ay / accel_deadzone;
            MPU2.setYAccelOffset(ay_offset2);
        }

        if (abs(8192 - mean_az) <= accel_deadzone)
        {
            ready++;
        }
        else
        {
            isCalibrated = false;

            // used for testing purposes only
            // Serial.print("Accel_Z_is_not_calibrated-> ");
            // Serial.print("\tmean_az:\t" + String(mean_az));
            // Serial.println("\taz_offset2:\t " + String(az_offset2));

            az_offset2 = az_offset2 + (8192 - mean_az) / accel_deadzone;
            MPU2.setZAccelOffset(az_offset2);
        }

        if (abs(mean_gx) <= gyro_deadzone)
        {
            ready++;
        }
        else
        {
            isCalibrated = false;

            // used for testing purposes only
            // Serial.print("Gyro_X_is_not_calibrated-> ");
            // Serial.print("\tmean_gx:\t" + String(mean_gx));
            // Serial.println("\tgx_offset2:\t" + String(gx_offset2));

            gx_offset2 = gx_offset2 - mean_gx / (gyro_deadzone + 1);
            MPU2.setXGyroOffset(gx_offset2);
        }

        if (abs(mean_gy) <= gyro_deadzone)
        {
            ready++;
        }
        else
        {
            isCalibrated = false;

            // used for testing purposes only
            // Serial.print("Gyro_Y_is_not_calibrated-> ");
            // Serial.print("\tmean_gy:\t" + String(mean_gy));
            // Serial.println("\tgy_offset2:\t" + String(gy_offset2));

            gy_offset2 = gy_offset2 - mean_gy / (gyro_deadzone + 1);
            MPU2.setYGyroOffset(gy_offset2);
        }

        if (abs(mean_gz) <= gyro_deadzone)
        {
            ready++;
        }
        else
        {
            isCalibrated = false;

            // used for testing purposes only
            // Serial.print("Gyro_Z_is_not_calibrated-> ");
            // Serial.print("\tmean_gz:\t" + String(mean_gz));
            // Serial.println("\tgz_offset2:\t" + String(gz_offset2));

            gz_offset2 = gz_offset2 - mean_gz / (gyro_deadzone + 1);
            MPU2.setZGyroOffset(gz_offset2);
        }

        if (ready == 6)
        {
            // used for testing purposes only
            Serial.println("Calibration_verified. Here_are_the_new_offsets.");
            Serial.println("ax_offset2:\t" + String(ax_offset2));
            Serial.println("ay_offset2:\t" + String(ay_offset2));
            Serial.println("az_offset2:\t" + String(az_offset2));
            Serial.println("gx_offset2:\t" + String(gx_offset2));
            Serial.println("gy_offset2:\t" + String(gy_offset2));
            Serial.println("gz_offset2:\t" + String(gz_offset2));
            
            logSensorData(7); // logging information after re-calibration

            isCalibrated = true;
            break; // exit the loop
        }
    }

    if (isCalibrated)
    {
        Serial.println("Calibration_verified. No_need_for_further_calibration.");
        logSensorData(7);
    }
    else
    {
        Serial.println("Calibration_failed. Further_calibration_is_needed.");
    }
}; // verifyAndSpeedUpCalibration

// unsure if I will use this function
// void calibrateXAccel(void)
// {
//     MPU2.setXAccelOffset(0);
//     // sensor reading
//     long buff_ax = 0;
//     count = 0;
//
//     while (count < (buffersize + 101))
//     {
//         // read raw accel/gyro measurements from device
//         MPU2.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
//
//         // used for testing purposes only
//         // Serial.print("MPU Output: AccX:\t");
//         // Serial.println(ax);
//
//         if (count > 100 && count <= (buffersize + 100))
//         {
//             // First 100 measures are discarded
//             buff_ax = buff_ax + ax;
//         }
//         if (count == (buffersize + 100))
//         {
//             mean_ax = buff_ax / buffersize;
//         }
//         count++;
//
//         currentMillis = millis();
//         if (currentMillis - previousMillis >= timePeriod002)
//         {
//             // delay(2); // Needed so we don't get repeated measures
//             previousMillis = currentMillis;
//         }
//     }
//
//     // calibrating time
//     ax_offset = -mean_ax / 8;
//     while (1)
//     {
//         int ready = 0;
//         MPU2.setXAccelOffset(ax_offset);
//         // used for testing purposes only
//         // Serial.print("MPU Calibration output: AX_offset set at:\t");
//         // Serial.println(ax_offset);
//
//         // sensor reading
//         long buff_ax = 0;
//         count = 0;
//
//         while (count < (buffersize + 101))
//         {
//             // read raw accel/gyro measurements from device
//             MPU2.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
//
//             // used for testing purposes only
//             // Serial.print("MPU Output: AccX: ");
//             // Serial.println(ax);
//
//             if (count > 100 && count <= (buffersize + 100))
//             { // First 100 measures are discarded
//                 buff_ax = buff_ax + ax;
//             }
//             if (count == (buffersize + 100))
//             {
//                 mean_ax = buff_ax / buffersize;
//             }
//             count++;
//
//             currentMillis = millis();
//             if (currentMillis - previousMillis >= timePeriod002)
//             {
//                 // delay(2); // Needed so we don't get repeated measures
//                 previousMillis = currentMillis;
//             }
//         }
//
//         if (abs(mean_ax) <= accel_deadzone)
//             ready++;
//         else
//             ax_offset = ax_offset - mean_ax / accel_deadzone;
//
//         if (ready == 1)
//             break; // exit the loop
//     }
//     Serial.println("\nFINISHED!");
//     Serial.print("\nSensor Accel X reading with offset:\t");
//     Serial.println(mean_ax);
//     Serial.print("Your new Accel X offset:\t");
//     Serial.println(ax_offset);
// };

// void calibrateYAccel(void)
// {
//     MPU2.setYAccelOffset(0);
//     // sensor reading
//     long buff_ay = 0;
//     count = 0;
//
//     while (count < (buffersize + 101))
//     {
//         // read raw accel/gyro measurements from device
//         MPU2.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
//
//         // used for testing purposes only
//         // Serial.print("MPU Output: AccY:\t");
//         // Serial.println(ay);
//
//         if (count > 100 && count <= (buffersize + 100))
//         { // First 100 measures are discarded
//             buff_ay = buff_ay + ay;
//         }
//         if (count == (buffersize + 100))
//         {
//             mean_ay = buff_ay / buffersize;
//         }
//         count++;
//
//         currentMillis = millis();
//         if (currentMillis - previousMillis >= timePeriod002)
//         {
//             // delay(2); // Needed so we don't get repeated measures
//             previousMillis = currentMillis;
//         }
//     }
//
//     // calibrating time
//     ay_offset = -mean_ay / 8;
//     while (1)
//     {
//         int ready = 0;
//         MPU2.setYAccelOffset(ay_offset);
//         // used for testing purposes only
//         // Serial.print("MPU Calibration output: AY_offset set at:\t");
//         // Serial.println(ay_offset);
//
//         // sensor reading
//         long buff_ay = 0;
//         count = 0;
//
//         while (count < (buffersize + 101))
//         {
//             // read raw accel/gyro measurements from device
//             MPU2.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
//
//             // used for testing purposes only
//             // Serial.print("MPU Output: AccY: ");
//             // Serial.println(ay);
//
//             if (count > 100 && count <= (buffersize + 100))
//             { // First 100 measures are discarded
//                 buff_ay = buff_ay + ay;
//             }
//             if (count == (buffersize + 100))
//             {
//                 mean_ay = buff_ay / buffersize;
//             }
//             count++;
//
//             currentMillis = millis();
//             if (currentMillis - previousMillis >= timePeriod002)
//             {
//                 // delay(2); // Needed so we don't get repeated measures
//                 previousMillis = currentMillis;
//             }
//         }
//
//         if (abs(mean_ay) <= accel_deadzone)
//             ready++;
//         else
//             ay_offset = ay_offset - mean_ay / accel_deadzone;
//
//         if (ready == 1)
//             break; // exit the loop
//     }
//     Serial.println("\nFINISHED!");
//     Serial.print("\nSensor Accel Y reading with offset:\t");
//     Serial.println(mean_ay);
//     Serial.print("Your new Accel Y offset:\t");
//     Serial.println(ay_offset);
// };

// void calibrateZAccel(void)
// {
//     MPU2.setZAccelOffset(0);
//     // sensor reading
//     long buff_az = 0;
//     count = 0;
//
//     while (count < (buffersize + 101))
//     {
//         // read raw accel/gyro measurements from device
//         MPU2.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
//
//         // used for testing purposes only
//         // Serial.print("MPU Output: AccX:\t");
//         // Serial.println(ax);
//
//         if (count > 100 && count <= (buffersize + 100))
//         { // First 100 measures are discarded
//             buff_az = buff_az + az;
//         }
//         if (count == (buffersize + 100))
//         {
//             mean_az = buff_az / buffersize;
//         }
//         count++;
//
//         currentMillis = millis();
//         if (currentMillis - previousMillis >= timePeriod002)
//         {
//             // delay(2); // Needed so we don't get repeated measures
//             previousMillis = currentMillis;
//         }
//     }
//
//     // calibrating time
//     ax_offset = -mean_az / 8;
//     while (1)
//     {
//         int ready = 0;
//         MPU2.setZAccelOffset(az_offset);
//         // used for testing purposes only
//         // Serial.print("MPU Calibration output: AX_offset set at:\t");
//         // Serial.println(ax_offset);
//
//         // sensor reading
//         long buff_az = 0;
//         count = 0;
//
//         while (count < (buffersize + 101))
//         {
//             // read raw accel/gyro measurements from device
//             MPU2.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
//
//             // used for testing purposes only
//             // Serial.print("MPU Output: AccX: ");
//             // Serial.println(ax);
//
//             if (count > 100 && count <= (buffersize + 100))
//             { // First 100 measures are discarded
//                 buff_az = buff_az + az;
//             }
//             if (count == (buffersize + 100))
//             {
//                 mean_az = buff_az / buffersize;
//             }
//             count++;
//
//             currentMillis = millis();
//             if (currentMillis - previousMillis >= timePeriod002)
//             {
//                 // delay(2); // Needed so we don't get repeated measures
//                 previousMillis = currentMillis;
//             }
//         }
//
//         if (abs(8192 - mean_az) <= accel_deadzone)
//             ready++;
//         else
//             az_offset = az_offset + (8192 - mean_az) / accel_deadzone;
//
//         if (ready == 1)
//             break; // exit the loop
//     }
//     Serial.println("\nFINISHED!");
//     Serial.print("\nSensor Accel Z reading with offset:\t");
//     Serial.println(mean_az);
//     Serial.print("Your new Accel Z offset:\t");
//     Serial.println(az_offset);
// };

// void calibrateXGyro(void)
// {
//     MPU2.setXGyroOffset(0);
//     // sensor reading
//     long buff_gx = 0;
//     count = 0;
//
//     while (count < (buffersize + 101))
//     {
//         // read raw accel/gyro measurements from device
//         MPU2.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
//
//         // used for testing purposes only
//         // Serial.print("MPU Output: GyroX:\t");
//         // Serial.println(gx);
//
//         if (count > 100 && count <= (buffersize + 100))
//         { // First 100 measures are discarded
//             buff_gx = buff_gx + gx;
//         }
//         if (count == (buffersize + 100))
//         {
//             mean_gx = buff_gx / buffersize;
//         }
//         count++;
//
//         currentMillis = millis();
//         if (currentMillis - previousMillis >= timePeriod002)
//         {
//             // delay(2); // Needed so we don't get repeated measures
//             previousMillis = currentMillis;
//         }
//     }
//
//     // calibrating time
//     gx_offset = -mean_gx / 4;
//     while (1)
//     {
//         int ready = 0;
//         MPU2.setXGyroOffset(gx_offset);
//         // used for testing purposes only
//         // Serial.print("MPU Calibration output: GX_offset set at:\t");
//         // Serial.println(gx_offset);
//
//         // sensor reading
//         long buff_gx = 0;
//         count = 0;
//
//         while (count < (buffersize + 101))
//         {
//             // read raw accel/gyro measurements from device
//             MPU2.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
//
//             // used for testing purposes only
//             // Serial.print("MPU Output: GyroX: ");
//             // Serial.println(gx);
//
//             if (count > 100 && count <= (buffersize + 100))
//             { // First 100 measures are discarded
//                 buff_gx = buff_gx + gx;
//             }
//             if (count == (buffersize + 100))
//             {
//                 mean_gx = buff_gx / buffersize;
//             }
//             count++;
//
//             currentMillis = millis();
//             if (currentMillis - previousMillis >= timePeriod002)
//             {
//                 previousMillis = currentMillis;
//                 // delay(2); // Needed so we don't get repeated measures
//             }
//         }
//
//         if (abs(mean_gx) <= accel_deadzone)
//             ready++;
//         else
//             gx_offset = gx_offset - mean_gx / (gyro_deadzone + 1);
//
//         if (ready == 1)
//             break; // exit the loop
//     }
//     Serial.println("\nFINISHED!");
//     Serial.print("\nSensor Gyro X reading with offset:\t");
//     Serial.println(mean_gx);
//     Serial.print("Your new Gyro X offset:\t");
//     Serial.println(gx_offset);
// };

// void calibrateYGyro(void)
// {
//     MPU2.setYGyroOffset(0);
//     // sensor reading
//     long buff_gy = 0;
//     count = 0;
//
//     while (count < (buffersize + 101))
//     {
//         // read raw accel/gyro measurements from device
//         MPU2.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
//
//         // used for testing purposes only
//         // Serial.print("MPU Output: GyroY:\t");
//         // Serial.println(gy);
//
//         if (count > 100 && count <= (buffersize + 100))
//         { // First 100 measures are discarded
//             buff_gy = buff_gy + gy;
//         }
//         if (count == (buffersize + 100))
//         {
//             mean_gy = buff_gy / buffersize;
//         }
//         count++;
//
//         currentMillis = millis();
//         if (currentMillis - previousMillis >= timePeriod002)
//         {
//             previousMillis = currentMillis;
//             // delay(2); // Needed so we don't get repeated measures
//         }
//     }
//
//     // calibrating time
//     gy_offset = -mean_gy / 4;
//     while (1)
//     {
//         int ready = 0;
//         MPU2.setYGyroOffset(gy_offset);
//         // used for testing purposes only
//         // Serial.print("MPU Calibration output: GY_offset set at:\t");
//         // Serial.println(gy_offset);
//
//         // sensor reading
//         long buff_gy = 0;
//         count = 0;
//
//         while (count < (buffersize + 101))
//         {
//             // read raw accel/gyro measurements from device
//             MPU2.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
//
//             // used for testing purposes only
//             // Serial.print("MPU Output: GyroY: ");
//             // Serial.println(gy);
//
//             if (count > 100 && count <= (buffersize + 100))
//             { // First 100 measures are discarded
//                 buff_gy = buff_gy + gy;
//             }
//             if (count == (buffersize + 100))
//             {
//                 mean_gy = buff_gy / buffersize;
//             }
//             count++;
//
//             currentMillis = millis();
//             if (currentMillis - previousMillis >= timePeriod002)
//             {
//                 previousMillis = currentMillis;
//                 // delay(2); // Needed so we don't get repeated measures
//             }
//         }
//
//         if (abs(mean_gy) <= gyro_deadzone)
//             ready++;
//         else
//             gy_offset = gy_offset - mean_gy / (gyro_deadzone + 1);
//
//         if (ready == 1)
//             break; // exit the loop
//     }
//     Serial.println("\nFINISHED!");
//     Serial.print("\nSensor Gyro Y reading with offset:\t");
//     Serial.println(mean_gy);
//     Serial.print("Your new Gyro Y offset:\t");
//     Serial.println(gy_offset);
// };

// void calibrateZGyro(void)
// {
//     MPU2.setZGyroOffset(0);
//     // sensor reading
//     long buff_gz = 0;
//     count = 0;
//
//     while (count < (buffersize + 101))
//     {
//         // read raw accel/gyro measurements from device
//         MPU2.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
//
//         // used for testing purposes only
//         // Serial.print("MPU Output: GyroZ:\t");
//         // Serial.println(gz);
//
//         if (count > 100 && count <= (buffersize + 100))
//         { // First 100 measures are discarded
//             buff_gz = buff_gz + gz;
//         }
//         if (count == (buffersize + 100))
//         {
//             mean_gz = buff_gz / buffersize;
//         }
//         count++;
//
//         if (currentMillis - previousMillis >= timePeriod002)
//         {
//             previousMillis = currentMillis;
//             // delay(2); // Needed so we don't get repeated measures
//         }
//     }
//
//     // calibrating time
//     gz_offset = -mean_gz / 4;
//     while (1)
//     {
//         int ready = 0;
//         MPU2.setZGyroOffset(gz_offset);
//         // used for testing purposes only
//         // Serial.print("MPU Calibration output: GZ_offset set at:\t");
//         // Serial.println(gz_offset);
//
//         // sensor reading
//         long buff_gz = 0;
//         count = 0;
//
//         while (count < (buffersize + 101))
//         {
//             // read raw accel/gyro measurements from device
//             MPU2.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
//
//             // used for testing purposes only
//             // Serial.print("MPU Output: GyroZ: ");
//             // Serial.println(gz);
//
//             if (count > 100 && count <= (buffersize + 100))
//             {
//                 // First 100 measures are discarded
//                 buff_gz = buff_gz + gz;
//             }
//             if (count == (buffersize + 100))
//             {
//                 mean_gz = buff_gz / buffersize;
//             }
//             count++;
//
//             currentMillis = millis();
//             if (currentMillis - previousMillis >= timePeriod002)
//             {
//                 previousMillis = currentMillis;
//                 // delay(2); // Needed so we don't get repeated measures
//             }
//         }
//
//         if (abs(mean_gz) <= gyro_deadzone)
//             ready++;
//         else
//             gz_offset = gz_offset - mean_gz / (gyro_deadzone + 1);
//
//         if (ready == 1)
//             break; // exit the loop
//     }
//     Serial.println("\nFINISHED!");
//     Serial.print("\nSensor Gyro Z reading with offset:\t");
//     Serial.println(mean_gz);
//     Serial.print("Your new Gyro Z offset:\t");
//     Serial.println(gz_offset);
// };

// ================================================================
// ===                       MAIN SETUP                         ===
// ================================================================
void calibrateDatasetup()
{
    // start message
    Serial.println("\nStarting_MPU6050_Calibration");
    // using millis() for timing delays
    currentMillis = millis();
    // delay of 1 second
    // delay(1000);
    if (currentMillis - previousMillis >= timePeriod1)
    {
        Serial.print(".");
        previousMillis = currentMillis;
    }
    // delay(2000); // original
    Serial.println("\nYour_MPU6050_should_be_placed_in_horizontal_position,_with_package_letters_facing_up. \nDon't_touch_it_until_you_see_a_finish_message.\n");
    if (currentMillis - previousMillis >= timePeriod1)
    {
        Serial.print(".");
        previousMillis = currentMillis;
    }
    // delay(3000); // original

    // reset offsets
    // DO NOT COMMENT THESE LINES IF YOU HAVE ALREADY SETUP THE OFFSETS
    MPU2.setXAccelOffset(0);
    MPU2.setYAccelOffset(0);
    MPU2.setZAccelOffset(0);
    MPU2.setXGyroOffset(0);
    MPU2.setYGyroOffset(0);
    MPU2.setZGyroOffset(0);
}; // end of calibrateDatasetup

// ================================================================
// ===                        MAIN LOOP                         ===
// ================================================================
//----------------------------------------------------//
// Calibrate bias of the accelerometer and gyroscope
// Sensor needs to be calibrated at each power cycle.
//----------------------------------------------------//

void calibrateDataloop()
{
    if (state == 0)
    {
        Serial.println("\nReading_sensors_for_first_time...");
        // dataFile.println("\nReading sensors for first time...");
        meansensors();
        state++;
        // creating a delay loop
        currentMillis = millis();
        if (currentMillis - previousMillis >= timePeriod1)
        {
            Serial.print(".");
            previousMillis = currentMillis;
            // delay(1000);
        }
    }

    if (state == 1)
    {
        Serial.println("\nCalculating_offsets...");
        // dataFile.println("\nCalculating offsets...");
        calibration();
        state++;
        // creating a delay loop
        currentMillis = millis();
        if (currentMillis - previousMillis >= timePeriod1)
        {
            Serial.print(".");
            previousMillis = currentMillis;
            // delay(1000);
        }
    }

    if (state == 2)
    {
        meansensors();

        // ===========
        // used for testing purposes only
        // display it on the serial monitor
        // Serial.println("\nFINISHED!");
        // Serial.print("Sensor readings with offsets:\t");
        // Serial.print(mean_ax);
        // Serial.print("\t");
        // Serial.print(mean_ay);
        // Serial.print("\t");
        // Serial.print(mean_az);
        // Serial.print("\t");
        // Serial.print(mean_gx);
        // Serial.print("\t");
        // Serial.print(mean_gy);
        // Serial.print("\t");
        // Serial.println(mean_gz);
        // Serial.print("Your offsets:\t");
        // Serial.print(ax_offset);
        // Serial.print("\t");
        // Serial.print(ay_offset);
        // Serial.print("\t");
        // Serial.print(az_offset);
        // Serial.print("\t");
        // Serial.print(gx_offset);
        // Serial.print("\t");
        // Serial.print(gy_offset);
        // Serial.print("\t");
        // Serial.println(gz_offset);
        // Serial.println("\nData is printed as: accelX accelY accelZ gyroX gyroY gyroZ");
        // Serial.println("Check that your sensor readings are close to 0 0 16384 0 0 0"); // based on setAccelerometerRange(MPU6050_RANGE_2_G) = 16384
        // Serial.println("Check that your sensor readings are close to 0 0 8192 0 0 0"); // based on setAccelerometerRange(MPU6050_RANGE_4_G) = 8192
        // Serial.println("If calibration was succesful write down your offsets so you can set them in your projects using something similar to mpu.setXAccelOffset(youroffset)");

        // ===========
        // save data to file
        // ensuring that we capture the data if there is no config file
        // levelTable();
        // Serial.println("Table is level");

        isCalibrating = true;
        // time to get the true baseDistanceCm and baseDistanceInch
        getFilteredDistance(); // captures the distance after calibration
        baseDistanceCm = distanceCm;
        baseDistanceInch = distanceInch;

        logSensorData(8); // to save CONFIG DATA to a config data file
        Serial.println("Calibration_data_saved!");

        logSensorData(1); // to save the CALIBRATE DATA to a data logging file

        // HERE IS THE RESPONSES:
        // Sensor readings with offsets:   2       -1      16388   0       -1      1
        // Your offsets : -527 - 4177 5472 - 191 - 41 - 12
        // Sensor readings with offsets:   7       -6      16383   0       -1      1
        // Your offsets : -524 - 4177 5446 - 187 - 40 - 11
        // Sensor readings with offsets:   3       -3      16379   0       0       0
        // Your offsets : -525 - 4177 5453 - 189 - 40 - 12
        // Sensor readings with offsets:   0       -5      16378   -2      1       0
        // Your offsets : -527 - 4177 5469 - 191 - 40 - 13
        // Sensor readings with offsets : -3 - 5 16389 - 1 0 - 1
        // Your offsets : -528 - 4178 5469 - 191 - 41 - 14
        // Data is printed as : acelX acelY acelZ giroX giroY giroZ Check that your sensor readings are close to 0 0 16384 0 0 0
        // Check that your sensor readings are close to 0 0 8192 0 0 0
        // current means and offset settings are:
        // 6	0	8185	0	0	0	-528	-4175	0	-190	-42	-5
        // or
        // -3	0	8192	0	0	0	-528	-4178	1	-191	-42	-5
    }
}; // end of calibrateDataloop

void calibratingData(uint8_t saved)
{
    // 1 means that it is a not saved calibrated data
    if (saved == 1)
    {
        calibrateDatasetup(); // clears the offsets
        // Serial.println("Calibrating_MPU6050"); // used for testing purposes only
        calibrateDataloop();
        // Serial.println("Calibration_Complete!"); // used for testing purposes only

        // ===========
        // This is a not tested code
        // Store the offset values
        // getMPUOffsets(); // getting the offsets from the MPU
        // preferences.begin("MPU_calibration", false);
        // preferences.putLong("gyroXOffset", gx_offset);
        // preferences.putLong("gyroYOffset", gy_offset);
        // preferences.putLong("gyroZOffset", gz_offset);
        // preferences.putLong("accelXOffset", ax_offset);
        // preferences.putLong("accelYOffset", ay_offset);
        // preferences.putLong("accelZOffset", az_offset);
        // preferences.end();
        // ===========

        return; // exit the function
    }
    else if (saved == 2)
    {
        // 2 means that it is a saved calibrated data

        // attempting to level table before calibration
        if (isCalibrationLevel == false)
        {
            Serial.println("Attempting_to_level_table_before_calibration..."); // used for testing purposes only
            UnCalibratedlevelTable();
        }

        verifyAndSpeedUpCalibration();

        loadDistances();

        Serial.println("Re-Calibration_Complete!"); // used for testing purposes only
        return; // exit the function
    }
    else
    {
        // anything else means it is an error
        Serial.println("Calibration_data_not_saved!");
    }
}; // end of calibratingData

#endif // USE_LIBRARY2

#endif // CALIBRATEDATA_H

// =========================================================
// END OF PROGRAM
// =========================================================