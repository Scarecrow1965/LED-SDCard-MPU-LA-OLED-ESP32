// +-------------------------------------------------------------
//
// Equipment:
// DOIT ESP32 Dev Kit V1, MPU-6050, OLED SSD1306, WS2812B Addressable LEDS x3,
// 4-relay module, 2 linear actuators, RT-11(JCHT35K9) remote/handset,
// and 650W 12V Power supply
//
// File: dataLogger.h
//
// Description:
//
// Provides the one way to find the calibration requirements for the
// MPU-6050, and the linear actuators to control the movement of the standup
// portion of the workstation.
//
// History:     27-Mar-2024     Scarecrow1965   Created
//
// +-------------------------------------------------------------

#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H

#include <Arduino.h>
#include <Wire.h>

// ========
// from SimpleTime.ino example
// https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP32/examples/Time/SimpleTime/SimpleTime.ino
// modifedprintLocalTime function. was initially "void printLocalTime(void)"
String printLocalTime(void)
{
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo))
    {
        Serial.println("No time available (yet)");
        // return; // original line
        const char *notready = "uhoh"; // modified line
        return notready;               // modified line
    }
    // Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S"); // original line
    // modified lines below
    char buffer[80];
    strftime(buffer, sizeof(buffer), "%A, %B %d %Y %H:%M:%S", &timeinfo);
    String formattedTime = String(buffer);
    return formattedTime;
}; // end of printLocalTime
//
// Callback function (get's called when time adjusts via NTP)
void timeavailable(struct timeval *t)
{
    Serial.println("Got time adjustment from NTP!");
    printLocalTime();
}; // end of timeavailable
// ========

#include <SPI.h>
#include <SD.h>

// for the SD card
extern String fileName; // from main.cpp
extern File dataFile;   // from main.cpp
extern File configFile; // from main.cpp

// variables
extern int16_t ax, ay, az, gx, gy, gz; // from main.cpp
extern double tempC; // from main.cpp
extern bool isStop, isMovingUp, isMovingDown, isStationary, isTableLevel, isTableMax, isBottomOut, rt11Working; // from linearActuator.h
extern int pinState1, pinState2; // from linearActuator.h to se if the RT-11 is signaling to move up or down
extern int Offsets[6], MeanValues[6]; // from IMU_Zero

#include "ultrasonic.h"
extern float distanceCm, distanceInch;
extern float baseDistanceCm, baseDistanceInch;
void getFilteredDistance(void);
void getRawDistance(void);

#include "filterData.h"
extern int16_t accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z;
extern double angleX, angleY;
void getMPUOffsets(void);
void setMPUOffsets(int16_t ax_offset, int16_t ay_offset, int16_t az_offset, int16_t gx_offset, int16_t gy_offset, int16_t gz_offset); // ax_offset2, ay_offset2, az_offset2, gx_offset2, gy_offset2, gz_offset2
void filterData(void);

#include "calibrateData.h"
extern int16_t mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz, ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;
extern bool isCalibrating;

void logDataHeader(void)
{
    File dataFile = SD.open(fileName, FILE_WRITE);
    // old data scheme
    // dataFile.print("Event,Raw AccX,Raw AccY,Raw AccZ,Raw GyroX,Raw GyroY,Raw GyroZ,Raw Temp,Filter accel_x,Filter accel_y,Filter accel_z,Filter gyro_x,Filter gyro_y,Filter gyro_z,Calibrated Mean AccX,Calibrated Mean AccY,Calibrated Mean AccZ,Calibrated Mean GyroX,Calibrated Mean GyroY,Calibrated Mean GyroZ,Calibrated AccelX Offset,Calibrated AccelY Offset,Calibrated AccelZ Offset,Calibrated GyroX Offset,Calibrated GyroY Offset,Calibrated GyroZ Offset,Kalman New AngleX,Kalman New AngleY,Kalman New AngleZ,Delta Time,Calc Velocity X,Calc Velocity Y,Calc Velocity Z,Calc Position X,Calc Position Y,Calc Position Z,Level Angle X,Level Angle Y, Current Distance CM, Current Distance Inch, Base Distance CM, Base Distance Inch,\n");
    // dataFile.print(" ,ax,ay,az,gx,gy,gz,tempC,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz,ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset,axFiltered,ayFiltered,azFiltered,deltaTime,velX,velY,velZ,posX,posY,posZ,angleX,angleY,distanceCm,distanceInch,baseDistanceCm,baseDistanceInch,\n");
    // new data scheme
    dataFile.print("Event,Raw AccX,Raw AccY,Raw AccZ,Raw GyroX,Raw GyroY,Raw GyroZ,Raw Temp,Filter accel_x,Filter accel_y,Filter accel_z,Filter gyro_x,Filter gyro_y,Filter gyro_z,Cal Mean AccX,Cal Mean AccY,Cal Mean AccZ,Cal Mean GyroX,Cal Mean GyroY,Cal Mean GyroZ,Cal AccelX Offset,Cal AccelY Offset,Cal AccelZ Offset,Cal GyroX Offset,Cal GyroY Offset,Cal GyroZ Offset,RT11 Working,DOWN_PIN,UP_PIN,Is Table Stop,Is Table Moving Up,Is Table Moving Down,Is Table Stationary,Is Table Level,Is Table Max Pos,Is Table at Bottom,Level Angle X,Level Angle Y,Current Distance CM,Current Distance Inch,Base Distance CM,Base Distance Inch,Time\n");
    dataFile.print(" ,ax,ay,az,gx,gy,gz,tempC,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz,ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset,rt11Working,pinState1,pinState2,isStop,isMovingUp,isMovingDown,isStationary,isTableLevel,isTableMax,isBottomOut,angleX,angleY,distanceCm,distanceInch,baseDistanceCm,baseDistanceInch,\n");
    // 
    dataFile.println("0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43");
    dataFile.flush();
    if (dataFile)
    {
        // Serial.println("Header written to " + fileName); // used for testing purpose only
    }
    else
    {
        Serial.println("Error writing Header to " + fileName); // used for testing purpose only
    }
    dataFile.close();
}; // end of logDataHeader

void configDataHeader(void)
{
    File configFile = SD.open("/config.csv", FILE_WRITE);
    // old data scheme
    // configFile.print("Event,Raw AccX,Raw AccY,Raw AccZ,Raw GyroX,Raw GyroY,Raw GyroZ,Raw Temp,Filter accel_x,Filter accel_y,Filter accel_z,Filter gyro_x,Filter gyro_y,Filter gyro_z,Cal Mean AccX,Cal Mean AccY,Cal Mean AccZ,Cal Mean GyroX,Cal Mean GyroY,Cal Mean GyroZ,Cal AccelX Offset,Cal AccelY Offset,Cal AccelZ Offset,Cal GyroX Offset,Cal GyroY Offset,Cal GyroZ Offset,Kalman New AngleX,Kalman New AngleY,Kalman New AngleZ,Delta Time,Calc Velocity X,Calc Velocity Y,Calc Velocity Z,Calc Position X,Calc Position Y,Calc Position Z,Level Angle X,Level Angle Y,Current Distance CM,Current Distance Inch,Base Distance CM,Base Distance Inch,\n");
    // configFile.print(" ,ax,ay,az,gx,gy,gz,tempC,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz,ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset,axFiltered,ayFiltered,azFiltered,deltaTime,velX,velY,velZ,posX,posY,posZ,angleX,angleY,distanceCm,distanceInch,baseDistanceCm,baseDistanceInch,\n");
    // new data scheme
    configFile.print("Event,Raw AccX,Raw AccY,Raw AccZ,Raw GyroX,Raw GyroY,Raw GyroZ,Raw Temp,Filter accel_x,Filter accel_y,Filter accel_z,Filter gyro_x,Filter gyro_y,Filter gyro_z,Cal Mean AccX,Cal Mean AccY,Cal Mean AccZ,Cal Mean GyroX,Cal Mean GyroY,Cal Mean GyroZ,Cal AccelX Offset,Cal AccelY Offset,Cal AccelZ Offset,Cal GyroX Offset,Cal GyroY Offset,Cal GyroZ Offset,Rt11 Working,DOWN_PIN,UP_PIN,Is Table Stop,Is Table Moving Up,Is Table Moving Down,Is Table Stationary,Is Table Level,Is Table Max Pos,Is Table at Bottom,Level Angle X,Level Angle Y,Current Distance CM,Current Distance Inch,Base Distance CM,Base Distance Inch,\n");
    configFile.print(" ,ax,ay,az,gx,gy,gz,tempC,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz,ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset,rt11Working,pinState1,pinState2 ,isStop,isMovingUp,isMovingDown,isStationary,isTableLevel,isTableMax,isBottomOut,angleX,angleY,distanceCm,distanceInch,baseDistanceCm,baseDistanceInch,\n");
    // 
    configFile.println("0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42");
    configFile.flush();
    if (configFile)
    {
        // Serial.println("Header written to config.csv"); // used for testing purpose only
    }
    else
    {
        Serial.println("Error writing Header to config.csv"); // used for testing purpose only
    }
    configFile.close();
}; // end of logDataHeader

class Logger
{
public:
    virtual void logData(String event, File &dataFile) = 0;
    virtual void handleError(String errorMessage)
    {
        Serial.println(errorMessage);
    }; // end of handleError

    virtual ~Logger(){
        // what else needs to be destroyed?

        // Serial.println("Logger destroyed"); // used for testing purpose only
    }; // end of destructor

}; // end of class Logger

class MPUCalibratedDataLogger : public Logger
{
private:
    String event;
    int16_t mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz, ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;
    const float baseDistanceCm, baseDistanceInch;
    // ax, ay, az, gx, gy, gz, tempC, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z,
    // isStop, isMovingUp, isMovingDown, isStationary, isTableLevel, isTableMax, isBottomOut, rt11Working,
    // pinState1, pinState2
    // angleX, angleY
    // distanceCm, distanceInch

public:
    MPUCalibratedDataLogger(String event, int16_t mean_ax, int16_t mean_ay, int16_t mean_az, int16_t mean_gx, int16_t mean_gy, int16_t mean_gz, int16_t ax_offset, int16_t ay_offset, int16_t az_offset, int16_t gx_offset, int16_t gy_offset, int16_t gz_offset, float baseDistanceCm, float baseDistanceInch)
        : event(event), mean_ax(mean_ax), mean_ay(mean_ay), mean_az(mean_az), mean_gx(mean_gx), mean_gy(mean_gy), mean_gz(mean_gz), ax_offset(ax_offset), ay_offset(ay_offset), az_offset(az_offset), gx_offset(gx_offset), gy_offset(gy_offset), gz_offset(gz_offset), baseDistanceCm(baseDistanceCm), baseDistanceInch(baseDistanceInch) {}

    void logData(String event, File &dataFile) override
    {
        if (!dataFile)
        {
            handleError("Error: Could not open file for writing");
            return;
        }

        dataFile.print(event + ":\t,"); // 1 tab here
        // get the raw data
        // dataFile.print(ax);
        dataFile.print(",");
        // dataFile.print(ay);
        dataFile.print(",");
        // dataFile.print(az);
        dataFile.print(",");
        // dataFile.print(gx);
        dataFile.print(",");
        // dataFile.print(gy);
        dataFile.print(",");
        // dataFile.print(gz);
        dataFile.print(",");
        // dataFile.print(tempC);
        dataFile.print(",");
        // get the filtered data
        // dataFile.print(accel_x);
        dataFile.print(",");
        // dataFile.print(accel_y);
        dataFile.print(",");
        // dataFile.print(accel_z);
        dataFile.print(",");
        // dataFile.print(gyro_x);
        dataFile.print(",");
        // dataFile.print(gyro_y);
        dataFile.print(",");
        // dataFile.print(gyro_z);
        dataFile.print(",");
        // account for the mean values which are seen on in the calibration cycle
        dataFile.print(mean_ax);
        dataFile.print(",");
        dataFile.print(mean_ay);
        dataFile.print(",");
        dataFile.print(mean_az);
        dataFile.print(",");
        dataFile.print(mean_gx);
        dataFile.print(",");
        dataFile.print(mean_gy);
        dataFile.print(",");
        dataFile.print(mean_gz);
        dataFile.print(",");
        // get the MPU offsets
        dataFile.print(ax_offset);
        dataFile.print(",");
        dataFile.print(ay_offset);
        dataFile.print(",");
        dataFile.print(az_offset);
        dataFile.print(",");
        dataFile.print(gx_offset);
        dataFile.print(",");
        dataFile.print(gy_offset);
        dataFile.print(",");
        dataFile.print(gz_offset);
        dataFile.print(",");
        // dataFile.print(rt11Working);
        dataFile.print(",");
        // dataFile.print(pinState1);
        dataFile.print(",");
        // dataFile.print(pinState2);
        dataFile.print(",");
        // dataFile.print(isStop);
        dataFile.print(",");
        // dataFile.print(isMovingUp);
        dataFile.print(",");
        // dataFile.print(isMovingDown);
        dataFile.print(",");
        // dataFile.print(isStationary);
        dataFile.print(",");
        // dataFile.print(isTableLevel);
        dataFile.print(",");
        // dataFile.print(isTableMax);
        dataFile.print(",");
        // dataFile.print(isBottomOut);
        dataFile.print(",");
        // get the angle data
        // dataFile.print(angleX);
        dataFile.print(",");
        // dataFile.print(angleY);
        dataFile.print(",");
        // get the distance data
        // dataFile.print(distanceCm);
        dataFile.print(",");
        // dataFile.print(distanceInch);
        dataFile.print(",");
        dataFile.print(baseDistanceCm);
        dataFile.print(",");
        dataFile.print(baseDistanceInch);
        dataFile.print(",");
        // get a timestamp
        // from SimpleTime.ino example
        // dataFile.print(printLocalTime()); // modified to fit the format of the dataFile
        // ========

        dataFile.println();
        dataFile.println("Check that your sensor readings are close to 0 0 8192 0 0 0");
        // should be the same as the end of the calibration cycle info
        dataFile.flush();

    }; // end of logData

    ~MPUCalibratedDataLogger()
    {
        // If dataFile is a file handle, close it
        if (dataFile)
        {
            dataFile.close();
        }

        // Serial.println("MPUCalibratedDataLogger destroyed"); // used for testing purpose only
    }; // end of destructor

}; // end of class MPUCalibratedDataLogger

class MPUIMUZeroData : public Logger
{
private:
    String event;
    int *Offsets, *MeanValues;
    float distanceCm, distanceInch, baseDistanceCm, baseDistanceInch;
    // ax, ay, az, gx, gy, gz, tempC, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z,
    // mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz, ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset,
    // isStop, isMovingUp, isMovingDown, isStationary, isTableLevel, isTableMax, isBottomOut, rt11Working,
    // pinState1, pinState2,
    // angleX, angleY,
    

public:
    MPUIMUZeroData(String(event), int Offsets[], int MeanValues[], float distanceCm, float distanceInch, float baseDistanceCm, float baseDistanceInch)
        : event(event), Offsets(Offsets), MeanValues(MeanValues), distanceCm(distanceCm), distanceInch(distanceInch), baseDistanceCm(baseDistanceCm), baseDistanceInch(baseDistanceInch) {}

    void logData(String event, File &configFile) override
    {
        if (!configFile)
        {
            handleError("Error: Could not open file for writing");
            return; // exit the function
        }

        configFile.print(event + " :\t,"); // 1 tab here
        // get the raw data
        // configFile.print(ax);
        configFile.print(",");
        // configFile.print(ay);
        configFile.print(",");
        // configFile.print(az);
        configFile.print(",");
        // configFile.print(gx);
        configFile.print(",");
        // configFile.print(gy);
        configFile.print(",");
        // configFile.print(gz);
        configFile.print(",");
        // configFile.print(tempC);
        configFile.print(",");
        // get the filtered data
        // configFile.print(accel_x);
        configFile.print(",");
        // configFile.print(accel_y);
        configFile.print(",");
        // configFile.print(accel_z);
        configFile.print(",");
        // configFile.print(gyro_x);
        configFile.print(",");
        // configFile.print(gyro_y);
        configFile.print(",");
        // configFile.print(gyro_z);
        configFile.print(",");
        // account for the mean values which are seen on in the calibration cycle
        configFile.print(MeanValues[0]);
        configFile.print(",");
        configFile.print(MeanValues[1]);
        configFile.print(",");
        configFile.print(MeanValues[2]);
        configFile.print(",");
        configFile.print(MeanValues[3]);
        configFile.print(",");
        configFile.print(MeanValues[4]);
        configFile.print(",");
        configFile.print(MeanValues[5]);
        configFile.print(",");
        // configFile.print("X Accel Offset =\t");
        // configFile.println(Offsets[0]);
        configFile.print(Offsets[0]);
        configFile.print(",");
        // configFile.print("Y Accel Offset =\t");
        // configFile.println(Offsets[1]);
        configFile.print(Offsets[1]);
        configFile.print(",");
        // configFile.print("Z Accel Offset =\t");
        // configFile.println(Offsets[2]);
        configFile.print(Offsets[2]);
        configFile.print(",");
        // configFile.print("X Gyro Offset =\t");
        // configFile.println(Offsets[3]);
        configFile.print(Offsets[3]);
        configFile.print(",");
        // configFile.print("Y Gyro Offset =\t");
        // configFile.println(Offsets[4]);
        configFile.print(Offsets[4]);
        configFile.print(",");
        // configFile.print("Z Gyro Offset =\t");
        // configFile.println(Offsets[5]);
        configFile.print(Offsets[5]);
        configFile.print(",");
        // configFile.pint(rt11Working);
        configFile.print(",");
        // configFile.print(pinState1);
        configFile.print(",");
        // configFile.print(pinState2);
        configFile.print(",");
        // configFile.print(isStop);
        configFile.print(",");
        // configFile.print(isMovingUp);
        configFile.print(",");
        // configFile.print(isMovingDown);
        configFile.print(",");
        // configFile.print(isStationary);
        configFile.print(",");
        // configFile.print(isTableLevel);
        configFile.print(",");
        // dataFconfigFileile.print(isTableMax);
        configFile.print(",");
        // configFile.print(isBottomOut);
        configFile.print(",");
        // get the angle data
        // configFile.print(angleX);
        configFile.print(",");
        // configFile.print(angleY);
        configFile.print(",");
        // get the distance data
        configFile.print(distanceCm);
        configFile.print(",");
        configFile.print(distanceInch);
        configFile.print(",");
        configFile.print(baseDistanceCm);
        configFile.print(",");
        configFile.print(baseDistanceInch);
        configFile.print(",\n");
        // get a timestamp
        // from SimpleTime.ino example
        // dataFile.print(printLocalTime()); // modified to fit the format of the dataFile
        // ========

        configFile.println("Check that your sensor readings are close to 0 0 8192 0 0 0");
        // should be the same as the end of the calibration cycle info
        configFile.flush();

    }; // end of logData

    ~MPUIMUZeroData()
    {
        // If dataFile is a file handle, close it
        if (dataFile)
        {
            dataFile.close();
        }

        delete[] Offsets;
        delete[] MeanValues;

        // Serial.println("MPUIMUZeroData destroyed"); // used for testing purpose only
    }; // end of destructor

}; // end of class MPUIMUZeroData

class MPURawDataLogger : public Logger
{
private:
    String event;
    int16_t ax, ay, az, gx, gy, gz;
    double tempC;
    // accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z,
    // mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz, ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset,
    // angleX, angleY
    // isStop, isMovingUp, isMovingDown, isStationary, isTableLevel, isTableMax, isBottomOut;
    // distanceCm, distanceInch

public:
    MPURawDataLogger(String event, int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz, double tempC)
        : event(event), ax(ax), ay(ay), az(az), gx(gx), gy(gy), gz(gz), tempC(tempC) {}

    void logData(String event, File &dataFile) override
    {
        if (!dataFile)
        {
            handleError("Error: Could not open file for writing");
            return; // exit the function
        }
        dataFile.print(event + ":\t,");
        dataFile.print(ax);
        dataFile.print(",");
        dataFile.print(ay);
        dataFile.print(",");
        dataFile.print(az);
        dataFile.print(",");
        dataFile.print(gx);
        dataFile.print(",");
        dataFile.print(gy);
        dataFile.print(",");
        dataFile.print(gz);
        dataFile.print(",");
        dataFile.print(tempC);
        dataFile.println();
        dataFile.flush();
    }; // end of logData

    ~MPURawDataLogger()
    {
        // If dataFile is a file handle, close it
        if (dataFile)
        {
            dataFile.close();
        }

        // Serial.println("MPURawDataLogger destroyed"); // used for testing purpose only
    }; // end of destructor

}; // end of class MPURawDataLogger

class MPUFilteredDataLogger : public Logger
{
private:
    String event;
    int16_t ax, ay, az, gx, gy, gz, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z;
    double tempC;
    // mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz, ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset,
    // angleX, angleY
    // isStop, isMovingUp, isMovingDown, isStationary, isTableLevel, isTableMax, isBottomOut;
    // distanceCm, distanceInch

public:
    MPUFilteredDataLogger(String event, double tempC, int16_t accel_x, int16_t accel_y, int16_t accel_z, int16_t gyro_x, int16_t gyro_y, int16_t gyro_z)
        : event(event), tempC(tempC), accel_x(accel_x), accel_y(accel_y), accel_z(accel_z), gyro_x(gyro_x), gyro_y(gyro_y), gyro_z(gyro_z) {}

    void logData(String event, File &dataFile) override
    {
        if (!dataFile)
        {
            handleError("Error: Could not open file for writing");
            return; // exit the function
        }
        dataFile.print(event + ":\t,"); // 1 tabs here
        // dataFile.print(ax);
        dataFile.print(",");
        // dataFile.print(ay);
        dataFile.print(",");
        // dataFile.print(az);
        dataFile.print(",");
        // dataFile.print(gx);
        dataFile.print(",");
        // dataFile.print(gy);
        dataFile.print(",");
        // dataFile.print(gz);
        dataFile.print(",");
        dataFile.print(tempC);
        dataFile.print(",");
        dataFile.print(accel_x);
        dataFile.print(",");
        dataFile.print(accel_y);
        dataFile.print(",");
        dataFile.print(accel_z);
        dataFile.print(",");
        dataFile.print(gyro_x);
        dataFile.print(",");
        dataFile.print(gyro_y);
        dataFile.print(",");
        dataFile.print(gyro_z);
        dataFile.println();
        dataFile.flush();
    }; // end of logData

    ~MPUFilteredDataLogger()
    {
        // If dataFile is a file handle, close it
        if (dataFile)
        {
            dataFile.close();
        }

        // Serial.println("MPUFilteredDataLogger destroyed"); // used for testing purpose only
    }; // end of destructor

}; // end of class MPUFilteredDataLogger

class MPUgetAccelerationData : public Logger
{
private:
    String event;
    int16_t accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z;
    double tempC;
    bool isStop, isMovingUp, isMovingDown, isStationary, isTableLevel, isTableMax, isBottomOut, rt11Working;
    int pinState1, pinState2;
    double angleX, angleY;
    float distanceCm, distanceInch;
    const float baseDistanceCm, baseDistanceInch;
    // ax, ay, az, gx, gy, gz,
    // mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz, ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset,

public:
    MPUgetAccelerationData(String event, double tempC, int16_t accel_x, int16_t accel_y, int16_t accel_z, int16_t gyro_x, int16_t gyro_y, int16_t gyro_z, bool rt11Working, int pinState1, int pinState2, bool isStop, bool isMovingUp, bool isMovingDown, bool isStationary, bool isTableLevel, bool isTableMax, bool isBottomOut, double angleX, double angleY, float distanceCm, float distanceInch, float baseDistanceCm, float baseDistanceInch)
        : event(event), tempC(tempC), accel_x(accel_x), accel_y(accel_y), accel_z(accel_z), gyro_x(gyro_x), gyro_y(gyro_y), gyro_z(gyro_z), rt11Working(rt11Working), pinState1(pinState1), pinState2(pinState2), isStop(isStop), isMovingUp(isMovingUp), isMovingDown(isMovingDown), isStationary(isStationary), isTableLevel(isTableLevel), isTableMax(isTableMax), isBottomOut(isBottomOut), angleX(angleX), angleY(angleY), distanceCm(distanceCm), distanceInch(distanceInch), baseDistanceCm(baseDistanceCm), baseDistanceInch(baseDistanceInch) {}

    void logData(String event, File &dataFile) override
    {
        if (!dataFile)
        {
            handleError("Error: Could not open file for writing");
            return; // exit the function
        }
        dataFile.print(event + ":\t,"); // 1 tab here
        // dataFile.print(ax);
        dataFile.print(",");
        // dataFile.print(ay);
        dataFile.print(",");
        // dataFile.print(az);
        dataFile.print(",");
        // dataFile.print(gx);
        dataFile.print(",");
        // dataFile.print(gy);
        dataFile.print(",");
        // dataFile.print(gz);
        dataFile.print(",");
        dataFile.print(tempC);
        dataFile.print(",");
        dataFile.print(accel_x);
        dataFile.print(",");
        dataFile.print(accel_y);
        dataFile.print(",");
        dataFile.print(accel_z);
        dataFile.print(",");
        dataFile.print(gyro_x);
        dataFile.print(",");
        dataFile.print(gyro_y);
        dataFile.print(",");
        dataFile.print(gyro_z);
        dataFile.print(",");
        // dataFile.print(mean_ax);
        dataFile.print(",");
        // dataFile.print(mean_ay);
        dataFile.print(",");
        // dataFile.print(mean_az);
        dataFile.print(",");
        // dataFile.print(mean_gx);
        dataFile.print(",");
        // dataFile.print(mean_gy);
        dataFile.print(",");
        // dataFile.print(mean_gz);
        dataFile.print(",");
        // dataFile.print(ax_offset);
        dataFile.print(",");
        // dataFile.print(ay_offset);
        dataFile.print(",");
        // dataFile.print(az_offset);
        dataFile.print(",");
        // dataFile.print(gx_offset);
        dataFile.print(",");
        // dataFile.print(gy_offset);
        dataFile.print(",");
        // dataFile.print(gz_offset);
        dataFile.print(",");
        dataFile.print(String(rt11Working ? "True" : "False"));
        dataFile.print(",");
        dataFile.print(pinState1);
        // dataFile.print(pinState1 ? "ON" : "OFF");
        dataFile.print(",");
        dataFile.print(pinState2);
        // dataFile.print(pinState2 ? "ON" : "OFF");
        dataFile.print(",");
        dataFile.print(String(isStop ? "True" : "False"));
        dataFile.print(",");
        dataFile.print(String(isMovingUp ? "True" : "False"));
        dataFile.print(",");
        dataFile.print(String(isMovingDown ? "True" : "False"));
        dataFile.print(",");
        dataFile.print(String(isStationary ? "True" : "False"));       
        dataFile.print(",");
        dataFile.print(String(isTableLevel ? "True" : "False"));
        dataFile.print(",");
        dataFile.print(String(isTableMax ? "True" : "False"));
        dataFile.print(",");
        dataFile.print(String(isBottomOut ? "True" : "False"));
        dataFile.print(",");
        dataFile.print(angleX);
        dataFile.print(",");
        dataFile.print(angleY);
        dataFile.print(",");
        dataFile.print(distanceCm);
        dataFile.print(",");
        dataFile.print(distanceInch);
        dataFile.print(",");
        // dataFile.print(baseDistanceCm);
        dataFile.print(",");
        // dataFile.print(baseDistanceInch);
        dataFile.print(",");
        // get a timestamp
        // from SimpleTime.ino example
        dataFile.print(printLocalTime()); // modified to fit the format of the dataFile
        // ========
        
        dataFile.println();
        dataFile.flush();
    }; // end of logData

    ~MPUgetAccelerationData()
    {
        // If dataFile is a file handle, close it
        if (dataFile)
        {
            dataFile.close();
        }

        // Serial.println("MPUgetAccelerationData destroyed"); // used for testing purpose only
    }; // end of destructor

}; // end of class MPUgetAccelerationData

class MPUstopMovementData : public Logger
{
private:
    String event;
    int16_t accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z;
    double tempC, angleX, angleY;
    float distanceCm, distanceInch;
    const float baseDistanceCm, baseDistanceInch;
    bool isStop, isMovingUp, isMovingDown, isStationary, isTableLevel, isTableMax, isBottomOut, rt11Working;
    int pinState1, pinState2;
    // ax, ay, az, gx, gy, gz,
    // mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz, ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset,

public:
    MPUstopMovementData(String event, double tempC, int16_t accel_x, int16_t accel_y, int16_t accel_z, int16_t gyro_x, int16_t gyro_y, int16_t gyro_z, bool rt11Working, int pinState1, int pinState2, bool IsStop, bool isMovingUp, bool isMovingDown, bool isStationary, bool isTableLevel, bool isTableMax, bool isBottomOut, double angleX, double angleY, float distanceCm, float distanceInch, float baseDistanceCm, float baseDistanceInch)
        : event(event), tempC(tempC), accel_x(accel_x), accel_y(accel_y), accel_z(accel_z), gyro_x(gyro_x), gyro_y(gyro_y), gyro_z(gyro_z), rt11Working(rt11Working), pinState1(pinState1), pinState2(pinState2), isStop(isStop), isMovingUp(isMovingUp), isMovingDown(isMovingDown), isStationary(isStationary), isTableLevel(isTableLevel), isTableMax(isTableMax), isBottomOut(isBottomOut), angleX(angleX), angleY(angleY), distanceCm(distanceCm), distanceInch(distanceInch), baseDistanceCm(baseDistanceCm), baseDistanceInch(baseDistanceInch) {}

    void logData(String event, File &dataFile) override
    {
        if (!dataFile)
        {
            handleError("Error: Could not open file for writing");
            return; // exit the function
        }
        dataFile.print(event + ":\t,"); // 1 tab here
        // get the raw data
        // dataFile.print(ax);
        dataFile.print(",");
        // dataFile.print(ay);
        dataFile.print(",");
        // dataFile.print(az);
        dataFile.print(",");
        // dataFile.print(gx);
        dataFile.print(",");
        // dataFile.print(gy);
        dataFile.print(",");
        // dataFile.print(gz);
        dataFile.print(",");
        dataFile.print(tempC);
        dataFile.print(",");
        // get the filtered data
        dataFile.print(accel_x);
        dataFile.print(",");
        dataFile.print(accel_y);
        dataFile.print(",");
        dataFile.print(accel_z);
        dataFile.print(",");
        dataFile.print(gyro_x);
        dataFile.print(",");
        dataFile.print(gyro_y);
        dataFile.print(",");
        dataFile.print(gyro_z);
        dataFile.print(",");
        // account for the mean values which are seen on in the calibration cycle
        // dataFile.print(mean_ax);
        dataFile.print(",");
        // dataFile.print(mean_ay);
        dataFile.print(",");
        // dataFile.print(mean_az);
        dataFile.print(",");
        // dataFile.print(mean_gx);
        dataFile.print(",");
        // dataFile.print(mean_gy);
        dataFile.print(",");
        // dataFile.print(mean_gz);
        dataFile.print(",");
        // get the MPU offsets
        // getMPUOffsets();
        // dataFile.print(ax_offset);
        dataFile.print(",");
        // dataFile.print(ay_offset);
        dataFile.print(",");
        // dataFile.print(az_offset);
        dataFile.print(",");
        // dataFile.print(gx_offset);
        dataFile.print(",");
        // dataFile.print(gy_offset);
        dataFile.print(",");
        // dataFile.print(gz_offset);
        dataFile.print(",");
        dataFile.print(String(rt11Working ? "True" : "False"));
        dataFile.print(",");
        dataFile.print(pinState1);
        // dataFile.print(pinState1 ? "ON" : "OFF");
        dataFile.print(",");
        dataFile.print(pinState2);
        // dataFile.print(pinState2 ? "ON" : "OFF");
        dataFile.print(",");
        dataFile.print(String(isStop ? "True" : "False"));
        dataFile.print(",");
        dataFile.print(String(isMovingUp ? "True" : "False"));
        dataFile.print(",");
        dataFile.print(String(isMovingDown ? "True" : "False"));
        dataFile.print(",");
        dataFile.print(String(isStationary ? "True" : "False"));       
        dataFile.print(",");
        dataFile.print(String(isTableLevel ? "True" : "False"));
        dataFile.print(",");
        dataFile.print(String(isTableMax ? "True" : "False"));
        dataFile.print(",");
        dataFile.print(String(isBottomOut ? "True" : "False"));
        dataFile.print(",");
        // get the angle data
        dataFile.print(angleX);
        dataFile.print(",");
        dataFile.print(angleY);
        dataFile.print(",");
        // get the distance data
        dataFile.print(distanceCm);
        dataFile.print(",");
        dataFile.print(distanceInch);
        dataFile.print(",");
        dataFile.print(baseDistanceCm);
        dataFile.print(",");
        dataFile.print(baseDistanceInch);
        dataFile.print(",");
        // get a timestamp
        // from SimpleTime.ino example
        dataFile.print(printLocalTime()); // modified to fit the format of the dataFile
        // ========

        dataFile.println();
        dataFile.flush();
    }; // end of logData

    ~MPUstopMovementData()
    {
        // If dataFile is a file handle, close it
        if (dataFile)
        {
            dataFile.close();
        }

        // Serial.println("MPUstopMovementData destroyed"); // used for testing purpose only
    }; // end of destructor

}; // end of class MPUstopMovementData

class MPUgetEverythingData : public Logger
{
private:
    String event;
    int16_t ax, ay, az, gx, gy, gz, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz, ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;
    double tempC, angleX, angleY;
    bool isStop, isMovingUp, isMovingDown, isStationary, isTableLevel, isTableMax, isBottomOut, rt11Working;
    int pinState1, pinState2;
    float distanceCm, distanceInch;
    const float baseDistanceCm, baseDistanceInch;

public:
    MPUgetEverythingData(String event, int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz, double tempC, int16_t accel_x, int16_t accel_y, int16_t accel_z, int16_t gyro_x, int16_t gyro_y, int16_t gyro_z, int16_t mean_ax, int16_t mean_ay, int16_t mean_az, int16_t mean_gx, int16_t mean_gy, int16_t mean_gz, int16_t ax_offset, int16_t ay_offset, int16_t az_offset, int16_t gx_offset, int16_t gy_offset, int16_t gz_offset, bool rt11Working, int pinState1, int pinState2, bool IsStop, bool isMovingUp, bool isMovingDown, bool isStationary, bool isTableLevel, bool isTableMax, bool isBottomOut, double angleX, double angleY, float distanceCm, float distanceInch, float baseDistanceCm, float baseDistanceInch)
        : event(event), ax(ax), ay(ay), az(az), gx(gx), gy(gy), gz(gz), tempC(tempC), accel_x(accel_x), accel_y(accel_y), accel_z(accel_z), gyro_x(gyro_x), gyro_y(gyro_y), gyro_z(gyro_z), mean_ax(mean_ax), mean_ay(mean_ay), mean_az(mean_az), mean_gx(mean_gx), mean_gy(mean_gy), mean_gz(mean_gz), ax_offset(ax_offset), ay_offset(ay_offset), az_offset(az_offset), gx_offset(gx_offset), gy_offset(gy_offset), gz_offset(gz_offset), rt11Working(rt11Working), pinState1(pinState1), pinState2(pinState2), isStop(isStop), isMovingUp(isMovingUp), isMovingDown(isMovingDown), isStationary(isStationary), isTableLevel(isTableLevel), isTableMax(isTableMax), isBottomOut(isBottomOut), angleX(angleX), angleY(angleY), distanceCm(distanceCm), distanceInch(distanceInch), baseDistanceCm(baseDistanceCm), baseDistanceInch(baseDistanceInch) {}

    void logData(String event, File &dataFile) override
    {
        if (!dataFile)
        {
            handleError("Error: Could not open file for writing");
            return; // exit the function
        }

        // get the event
        dataFile.print(event + ":\t,"); // 1 tab here
        // get the raw data
        dataFile.print(ax);
        dataFile.print(",");
        dataFile.print(ay);
        dataFile.print(",");
        dataFile.print(az);
        dataFile.print(",");
        dataFile.print(gx);
        dataFile.print(",");
        dataFile.print(gy);
        dataFile.print(",");
        dataFile.print(gz);
        dataFile.print(",");
        dataFile.print(tempC);
        dataFile.print(",");
        // get the filtered data
        dataFile.print(accel_x);
        dataFile.print(",");
        dataFile.print(accel_y);
        dataFile.print(",");
        dataFile.print(accel_z);
        dataFile.print(",");
        dataFile.print(gyro_x);
        dataFile.print(",");
        dataFile.print(gyro_y);
        dataFile.print(",");
        dataFile.print(gyro_z);
        dataFile.print(",");
        // account for the mean values which are seen on in the calibration cycle
        dataFile.print(mean_ax);
        dataFile.print(",");
        dataFile.print(mean_ay);
        dataFile.print(",");
        dataFile.print(mean_az);
        dataFile.print(",");
        dataFile.print(mean_gx);
        dataFile.print(",");
        dataFile.print(mean_gy);
        dataFile.print(",");
        dataFile.print(mean_gz);
        dataFile.print(",");
        // get the MPU offsets
        getMPUOffsets();
        dataFile.print(ax_offset);
        dataFile.print(",");
        dataFile.print(ay_offset);
        dataFile.print(",");
        dataFile.print(az_offset);
        dataFile.print(",");
        dataFile.print(gx_offset);
        dataFile.print(",");
        dataFile.print(gy_offset);
        dataFile.print(",");
        dataFile.print(gz_offset);
        dataFile.print(",");
        dataFile.print(String(rt11Working ? "True" : "False"));
        dataFile.print(",");
        dataFile.print(pinState1);
        // dataFile.print(pinState1 ? "ON" : "OFF");
        dataFile.print(",");
        dataFile.print(pinState2);
        // dataFile.print(pinState2 ? "ON" : "OFF");
        dataFile.print(",");
        dataFile.print(String(isStop ? "True" : "False"));
        dataFile.print(",");
        dataFile.print(String(isMovingUp ? "True" : "False"));
        dataFile.print(",");
        dataFile.print(String(isMovingDown ? "True" : "False"));
        dataFile.print(",");
        dataFile.print(String(isStationary ? "True" : "False"));       
        dataFile.print(",");
        dataFile.print(String(isTableLevel ? "True" : "False"));
        dataFile.print(",");
        dataFile.print(String(isTableMax ? "True" : "False"));
        dataFile.print(",");
        dataFile.print(String(isBottomOut ? "True" : "False"));
        dataFile.print(",");
        // get the angle data
        dataFile.print(angleX);
        dataFile.print(",");
        dataFile.print(angleY);
        dataFile.print(",");
        // get the distance data
        dataFile.print(distanceCm);
        dataFile.print(",");
        dataFile.print(distanceInch);
        dataFile.print(",");
        dataFile.print(baseDistanceCm);
        dataFile.print(",");
        dataFile.print(baseDistanceInch);
        dataFile.print(",");
        // get a timestamp
        // from SimpleTime.ino example
        dataFile.print(printLocalTime()); // modified to fit the format of the dataFile
        // ========

        dataFile.println();
        dataFile.flush();
    }; // end of logData

    ~MPUgetEverythingData()
    {
        // If dataFile is a file handle, close it
        if (dataFile)
        {
            dataFile.close();
        }

        // Add code here to destroy any dynamically allocated resources or perform any necessary cleanup.
        // For example, if you have dynamically allocated memory using "new", you should delete it here.
        // If you have acquired any system resources, such as file handles or network connections, you should release them here.
        // If you have registered any event handlers or callbacks, you should unregister them here.
        // If you have initialized any external libraries or components, you should deinitialize them here.
        // If you have opened any files, you should close them here.
        // If you have started any threads, you should join them here.

        // Serial.println("MPUgetEverythingData destroyed"); // used for testing purpose only
    }; // end of destructor

}; // end of class MPUgetAccelerationData

class SavedMPUandHCSR04BaseData : public Logger
{
private:
    String event;
    bool isCalibrating;
    int *Offsets, *MeanValues;
    int16_t ax, ay, az, gx, gy, gz, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz, ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;
    double tempC;
    bool isStop, isMovingUp, isMovingDown, isStationary, isTableLevel, isTableMax, isBottomOut;
    float distanceCm, distanceInch;
    const float baseDistanceCm, baseDistanceInch;

public:
    SavedMPUandHCSR04BaseData(String event, int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz, double tempC, int16_t accel_x, int16_t accel_y, int16_t accel_z, int16_t gyro_x, int16_t gyro_y, int16_t gyro_z, int16_t mean_ax, int16_t mean_ay, int16_t mean_az, int16_t mean_gx, int16_t mean_gy, int16_t mean_gz, int16_t ax_offset, int16_t ay_offset, int16_t az_offset, int16_t gx_offset, int16_t gy_offset, int16_t gz_offset, int *Offsets, int *MeanValues, float distanceCm, float distanceInch, float baseDistanceCm, float baseDistanceInch, bool isCalibrating)
        : event(event), ax(ax), ay(ay), az(az), gx(gx), gy(gy), gz(gz), tempC(tempC), accel_x(accel_x), accel_y(accel_y), accel_z(accel_z), gyro_x(gyro_x), gyro_y(gyro_y), gyro_z(gyro_z), mean_ax(mean_ax), mean_ay(mean_ay), mean_az(mean_az), mean_gx(mean_gx), mean_gy(mean_gy), mean_gz(mean_gz), ax_offset(ax_offset), ay_offset(ay_offset), az_offset(az_offset), gx_offset(gx_offset), gy_offset(gy_offset), gz_offset(gz_offset), Offsets(Offsets), MeanValues(MeanValues), distanceCm(distanceCm), distanceInch(distanceInch), baseDistanceCm(baseDistanceCm), baseDistanceInch(baseDistanceInch), isCalibrating(isCalibrating) {}

    void logData(String event, File &configFile) override
    {
        if (!configFile)
        {
            handleError("Error: Could not open file for writing");
            return; // exit the function
        }
        // get the event
        configFile.print(event + ":\t,"); // 1 tab here
        // get the raw data
        configFile.print(ax);
        configFile.print(",");
        configFile.print(ay);
        configFile.print(",");
        configFile.print(az);
        configFile.print(",");
        configFile.print(gx);
        configFile.print(",");
        configFile.print(gy);
        configFile.print(",");
        configFile.print(gz);
        configFile.print(",");
        // get the filtered data
        filterData();
        configFile.print(tempC);
        configFile.print(",");
        configFile.print(accel_x);
        configFile.print(",");
        configFile.print(accel_y);
        configFile.print(",");
        configFile.print(accel_z);
        configFile.print(",");
        configFile.print(gyro_x);
        configFile.print(",");
        configFile.print(gyro_y);
        configFile.print(",");
        configFile.print(gyro_z);
        configFile.print(",");
        // get the mean values and offsets
        if (isCalibrating)
        {
            // account for the mean values which are seen on in the calibration cycle
            configFile.print(mean_ax);
            configFile.print(",");
            configFile.print(mean_ay);
            configFile.print(",");
            configFile.print(mean_az);
            configFile.print(",");
            configFile.print(mean_gx);
            configFile.print(",");
            configFile.print(mean_gy);
            configFile.print(",");
            configFile.print(mean_gz);
            configFile.print(",");
            // get the MPU offsets
            configFile.print(ax_offset);
            configFile.print(",");
            configFile.print(ay_offset);
            configFile.print(",");
            configFile.print(az_offset);
            configFile.print(",");
            configFile.print(gx_offset);
            configFile.print(",");
            configFile.print(gy_offset);
            configFile.print(",");
            configFile.print(gz_offset);
            configFile.print(",");
        }
        else
        {
            // then it comes from IMU_Zero.h
            // account for the mean values which are seen on in the calibration cycle
            configFile.print(MeanValues[0]);
            configFile.print(",");
            configFile.print(MeanValues[1]);
            configFile.print(",");
            configFile.print(MeanValues[2]);
            configFile.print(",");
            configFile.print(MeanValues[3]);
            configFile.print(",");
            configFile.print(MeanValues[4]);
            configFile.print(",");
            configFile.print(MeanValues[5]);
            configFile.print(",");
            // get the MPU offsets
            configFile.print(Offsets[0]);
            configFile.print(",");
            configFile.print(Offsets[1]);
            configFile.print(",");
            configFile.print(Offsets[2]);
            configFile.print(",");
            configFile.print(Offsets[3]);
            configFile.print(",");
            configFile.print(Offsets[4]);
            configFile.print(",");
            configFile.print(Offsets[5]);
        }

        configFile.print(",");
        // configFile.print(rt11Working));
        configFile.print(",");
        // configFile.print(pinState1);
        configFile.print(",");
        // configFile.print(pinState2);
        configFile.print(",");
        // configFile.print(isStop);
        configFile.print(",");
        // configFile.print(isMovingUp);
        configFile.print(",");
        // configFile.print(isMovingDown);
        configFile.print(",");
        // configFile.print(isStationary);
        configFile.print(",");
        // configFile.print(isTableLevel);
        configFile.print(",");
        // configFile.print(isTableMax);
        configFile.print(",");
        // configFile.print(isBottomOut);
        // get the angle data
        // dataFile.print(angleX);
        configFile.print(",");
        // dataFile.print(angleY);
        configFile.print(",");
        // get the distance data
        getFilteredDistance();
        configFile.print(distanceCm);
        configFile.print(",");
        configFile.print(distanceInch);
        configFile.print(",");
        configFile.print(baseDistanceCm);
        configFile.print(",");
        configFile.print(baseDistanceInch);
        configFile.print(",");
        configFile.println();
        configFile.flush();
    }; // end of logData

    ~SavedMPUandHCSR04BaseData()
    {
        // what else needs to be destroyed?
        delete[] Offsets;
        delete[] MeanValues;

        // If dataFile is a file handle, close it
        if (dataFile)
        {
            dataFile.close();
        }

        // Serial.println("SavedMPUandHCSR04BaseData destroyed"); // used for testing purposes only
    }; // end of destructor

}; // end of class MPUgetAccelerationData

void logSensorData(int state)
{
    // ... code to read sensor data ...
    Logger *logger;

    String event;
    if (state == 1)
    {
        event = "MPU-Calibrate-Data";
        logger = new MPUCalibratedDataLogger(event, mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz, ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset, baseDistanceCm, baseDistanceInch);
    }
    else if (state == 2)
    {
        event = "MPU-Raw-Data";
        logger = new MPURawDataLogger(event, ax, ay, az, gx, gy, gz, tempC);
    }
    else if (state == 3)
    {
        event = "MPU-Filtered-Data";
        logger = new MPUFilteredDataLogger(event, tempC, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z);
    }
    else if (state == 4)
    {
        event = "get-Acceleration-Data";
        logger = new MPUgetAccelerationData(event, tempC, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, rt11Working, pinState1, pinState2, isStop, isMovingUp, isMovingDown, isStationary, isTableLevel, isTableMax, isBottomOut, angleX, angleY, distanceCm, distanceInch, baseDistanceCm, baseDistanceInch);
    }
    else if (state == 5)
    {
        event = "Stop-Movement-Data";
        logger = new MPUstopMovementData(event, tempC, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, rt11Working, pinState1, pinState2, isStop, isMovingUp, isMovingDown, isStationary, isTableLevel, isTableMax, isBottomOut, angleX, angleY, distanceCm, distanceInch, baseDistanceCm, baseDistanceInch);
    }
    else if (state == 6)
    {
        event = "MPU-IMUZero-Calibrate-Data";
        logger = new MPUIMUZeroData(event, Offsets, MeanValues, distanceCm, distanceInch, baseDistanceCm, baseDistanceInch);
    }
    else if (state == 7)
    {
        event = "get-Everything-Data";
        logger = new MPUgetEverythingData(event, ax, ay, az, gx, gy, gz, tempC, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz, ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset, rt11Working, pinState1, pinState2, isStop, isMovingUp, isMovingDown, isStationary, isTableLevel, isTableMax, isBottomOut, angleX, angleY, distanceCm, distanceInch, baseDistanceCm, baseDistanceInch);
    }
    else if (state == 8)
    {
        event = "Saved-MPU-and-HCSR04-Base-Data";
        logger = new SavedMPUandHCSR04BaseData(event, ax, ay, az, gx, gy, gz, tempC, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz, ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset, Offsets, MeanValues, distanceCm, distanceInch, baseDistanceCm, baseDistanceInch, isCalibrating);
    }
    else
    {
        Serial.print("Invalid state: ");
    }

    if (state == 1 || state == 3 || state == 4 || state == 5 || state == 6 || state == 7)
    {
        // Serial.println("Saving to Data file"); // used for testing purpose only
        File dataFile = SD.open(fileName, FILE_APPEND);
        if (dataFile)
        {
            logger->logData(event, dataFile);
            // Serial.println(event + " written to file"); // used for testing purpose only
            dataFile.flush();
            dataFile.close();
            // Serial.println(fileName + " closed successfully."); // used for testing purpose only
        }
    }
    else if (state == 2 || state == 8)
    {
        // Serial.println("Saving to Config file"); // used for testing purpose only
        File configFile = SD.open("/config.csv", FILE_APPEND);
        if (configFile)
        {
            logger->logData(event, configFile);
            // Serial.println(event + " written to file"); // used for testing purpose only
            configFile.flush();
            configFile.close();
            // Serial.println("config.csv closed successfully."); // used for testing purpose only
        }
    }
    else
    {
        Serial.println("Invalid state: " + state);
    }

    delete logger; // no need to delete logger since we are using smart pointer
}; // end of logSensorData

#endif // DATA_LOGGER_H

// =========================================================
// END OF PROGRAM
// =========================================================