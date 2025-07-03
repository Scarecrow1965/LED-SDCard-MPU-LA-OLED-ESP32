// +-------------------------------------------------------------
//
// Equipment:
// DOIT ESP32 Dev Kit V1, MPU-6050, OLED SSD1306, WS2812B Addressable LEDS x3,
// 4-relay module, 2 linear actuators, RT-11(JCHT35K9) remote/handset,
// and 650W 12V Power supply
//
// File: filterData.h
//
// Description:
//
// Provides the one way to find the calibration requirements for the
// MPU-6050, and the linear actuators to control the movement of the standup
// portion of the workstation.
//
// History:     12-Mar-2024     Scarecrow1965   Created
//
// +-------------------------------------------------------------

#ifndef FILTERDATA_H
#define FILTERDATA_H

#include <Arduino.h>
#include <Wire.h>
// #include <SPI.h>
#include <SD.h>

// libraries for the OLED
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <U8g2lib.h>

// libraries for MPU-6050
#include "calibrateData.h"
// void calibrateXAccel(void); // not used at this time
// void calibrateYAccel(void); // not used at this time
// void calibrateZAccel(void); // not used at this time
// void calibrateXGyro(void); // not used at this time
// void calibrateYGyro(void); // not used at this time
// void calibrateZGyro(void); // not used at this time

#include <I2Cdev.h>

// #define USE_LIBRARY2
#ifdef USE_LIBRARY2

// from main.cpp variables
extern int16_t ax, ay, az, gx, gy, gz;
extern double tempC;
// from calibrateData.h variables
extern int count;
// from main.cpp variables
extern String fileName;
extern File dataFile;
extern File configFile;

extern U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2; // This works but according to the function, it shouldn't
extern uint8_t oled_LineH;
extern Adafruit_SSD1306 display;
// from IMU_Zero.h variables
extern int Offsets[6]; // if the data comes from the IMUSetup function in the IMU_Zero.h file
// from LinearActuator.h variables
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
// this will be used for the PID control of the linear actuators
// #include <PID_v1.h>
// // Legend: LA = Linear Actuator, LHP = Left Actuator Positive, LHM = Left Actuator Negative, RHP = Right Actuator Positive, RHM = Right Actuator Negative
// extern const int relay1_LA1LHP_Pin; // Relay controlling actuator #1 positive // it is actually pin 6 on the DOIT ESP32 Dev Kit V1 or GPIO27
// extern const int relay2_LA1LHM_Pin; // Relay controlling actuator #1 negative // it is actually pin 7 on the DOIT ESP32 Dev Kit V1 or GPIO26
// extern const int relay3_LA2RHP_Pin; // Relay controlling actuator #2 positive // it is actually pin 8 on the DOIT ESP32 Dev Kit V1 or GPIO25
// extern const int relay4_LA2RHM_Pin; // Relay controlling actuator #2 negative // it is actually pin 9 on the DOIT ESP32 Dev Kit V1 or GPIO33

#include "ultrasonic.h"
// void getFilteredDistance(void);
// from ultrasonic.h variables
extern float distanceCm, distanceInch;
extern float baseDistanceCm, baseDistanceInch;

#include "dataLogger.h" // for logging the data to a file

// Initialize the angle and the previous gyro measurement
double angleX, angleY, newAngle;
// variables for filtered data
int16_t accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z;
int16_t ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;
int16_t mean_ax2, mean_ay2, mean_az2, mean_gx2, mean_gy2, mean_gz2;
int16_t ax_offset2, ay_offset2, az_offset2, gx_offset2, gy_offset2, gz_offset2;

// variables
long ax_buffer, ay_buffer, az_buffer, gx_buffer, gy_buffer, gz_buffer;
int16_t ax_mean = 0, ay_mean = 0, az_mean = 0, gx_mean = 0, gy_mean = 0, gz_mean = 0;
// Change this 3 variables if you want to fine tune the sketch to your needs.
// int buffersize = 1000;     // Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int filtersize = 300; // Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:300)
// int acel_deadzone = 8;     // Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
// int gyro_deadzone = 1;     // Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)

// based on IMU_Zero.h, I am currently using the following:
// setAccelerometerRange(MPU6050_RANGE_4_G);, setGyroRange(MPU6050_RANGE_250_DEG);,
// setFilterBandwidth(MPU6050_BAND_44_HZ);, setCycleRate(MPU6050_CYCLE_40_HZ);,
double gyro_sensitivity = 131.0; // 131 LSB/(degrees/sec) // 250 Degrees/second
// double gyro_sensitivity = 65.5;  // 65.5 LSB/(degrees/sec) // 500 Degrees/second
// double gyro_sensitivity = 32.8;  // 32.8 LSB/(degrees/sec) // 1000 Degrees/second
// double gyro_sensitivity = 16.4;  // 16.4 LSB/(degrees/sec) // 2000 Degrees/second
// double accel_sensitivity = 16384.0; // 16384 LSB/g // 2g full scale range
double accel_sensitivity = 8192.0; // 8192 LSB/g // 4g full scale range
// double accel_sensitivity = 4096.0;  // 4096 LSB/g // 8g full scale range
// double accel_sensitivity = 2048.0;  // 2048 LSB/g // 16g full scale range

// Define the time step and the filter constant for the complimentary filter function
float dt = 0.01;    // Time step (100Hz update rate)
float alpha = 0.98; // Filter constant

// Use with Kalman Filter
const float Q_angle = 0.001;  // Process noise variance for the accelerometer
const float Q_bias = 0.003;   // Process noise variance for the gyro bias
const float R_measure = 0.03; // Measurement noise variance
float angle = 0;              // The angle calculated by the Kalman filter
float bias = 0;               // The gyro bias calculated by the Kalman filter
float rate;                   // Unbiased rate calculated from the rate and the calculated bias
unsigned long timer;
float P[2][2] = {{0, 0}, {0, 0}}; // Error covariance matrix

// end variables

void filterData()
{
    count = 0;
    ax_buffer = 0, ay_buffer = 0, az_buffer = 0, gx_buffer = 0, gy_buffer = 0, gz_buffer = 0;

    while (count < (filtersize + 101))
    {
        // Execute the command
        // For example, if the command is to blink an LED, you can toggle the LED state here
        // read raw accel/gyro measurements from device
        MPU2.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        // used for testing purposes only
        // Serial.print("MPU before Filter: AccX: ");
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
        // Serial.println();

        if (count > 100 && count <= (filtersize + 100))
        {
            // First 100 measures are discarded
            ax_buffer = ax_buffer + ax;
            ay_buffer = ay_buffer + ay;
            az_buffer = az_buffer + az;
            gx_buffer = gx_buffer + gx;
            gy_buffer = gy_buffer + gy;
            gz_buffer = gz_buffer + gz;
        }
        if (count == (filtersize + 100))
        {
            ax_mean = ax_buffer / filtersize;
            ay_mean = ay_buffer / filtersize;
            az_mean = az_buffer / filtersize;
            gx_mean = gx_buffer / filtersize;
            gy_mean = gy_buffer / filtersize;
            gz_mean = gz_buffer / filtersize;
        }
        count++;

        currentMillis = millis();
        if (currentMillis - previousMillis >= timePeriod002)
        {
            // delay(2); // Needed so we don't get repeated measures
            previousMillis = currentMillis;
        }
    }
    double temp = MPU2.getTemperature();
    tempC = (temp / 340.00) + 36.53;
    // ================
    // x_accel_offset = x_accel_mean / accel_filter_X;
    // y_accel_offset = y_accel_mean / acel_filter_Y;
    az_offset = (accel_sensitivity - az_mean) / tempC;
    // x_gyro_offset = x_gyro_mean / gyro_filter_X;
    // y_gyro_offset = y_gyro_mean / gyro_filter_Y;
    // z_gyro_offset = z_gyro_mean / gyro_filter_Z;
    //
    // this is what the offset would be like?
    // ax = x_accel_offset;
    // ay = y_accel_offset;
    accel_z = az_offset;
    // gx = x_gyro_offset;
    // gy = y_gyro_offset;
    // gz = z_gyro_offset;
    // ================

    // ================
    // this gives me a filtered look at what the MPU is actually feeling
    accel_x = ax_mean;
    accel_y = ay_mean;
    // az = z_accel_mean;
    gyro_x = gx_mean;
    gyro_y = gy_mean;
    gyro_z = gz_mean;
    // if ax and ay are within =/- 10, then the MPU is at rest
    // if gx, gy, and gz are within =/- 20, then the MPU is at rest
    // if az is close to 8192, then I could subtract mean_az from it
    // ================

    // ================
    // used for testing purposes only
    // Serial.print("MPU after Filter:\nAccX: ");
    // Serial.print(ax);
    // Serial.print("\tAccY: ");
    // Serial.print(ay);
    // Serial.print("\tAccZ: ");
    // Serial.print(az);
    // Serial.print("\tGyroX: ");
    // Serial.print(gx);
    // Serial.print("\tGyroY: ");
    // Serial.print(gy);
    // Serial.print("\tGyroZ: ");
    // Serial.print(gz);
    // Serial.print("\tTemps: ");
    // Serial.println(tempC);

    // used to save the filtered data to a CSV file
    // logSensorData(3); // get the ACCELERATION DATA into the logging file on the SD Card
    // ================
}; // end filterData

float kalmanFilter(float newAngle, float newRate, float dt)
{
    // The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
    // Discrete Kalman filter time update equations - Time Update ("Predict")
    // Update xhat - Project the state ahead
    rate = newRate - bias;
    angle += dt * rate;

    // Update estimation error covariance - Project the error covariance ahead
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    float S = P[0][0] + R_measure; // Estimate error
    float K[2];                    // Kalman gain - This is a 2x1 vector
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    float y = newAngle - angle; // Angle difference
    angle += K[0] * y;
    bias += K[1] * y;

    // Calculate estimation error covariance - Update the error covariance
    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle;
}; // end kalmanFilter

float kalmanloop(int16_t accelRaw, int16_t gyroRaw, float (*angleCalculation)(int16_t, int16_t), String axis)
{
    // Convert gyro readings to degrees/sec
    float gyroRate = gx / gyro_sensitivity; // sensitivity is 131 LSB/(degrees/sec) for 250 degrees/second

    // Calculate the angle from the accelerometer
    // float accelAngle = atan2(ay, az) * 180 / PI; // working on the Y-axis
    // float accelAngle = atan2(ax, az) * 180 / PI; // working on the X-axis
    float accelAngle = angleCalculation(accelRaw, az) * 180 / PI;

    // Calculate dt
    float dt = (micros() - timer) / 1000000.0;
    timer = micros();

    // Apply the Kalman filter
    double newAngle = kalmanFilter(accelAngle, gyroRate, dt);

    // used for testing purposes only
    // Print the angle
    // Serial.print("Kalman Loop newAngle for:\t");
    // Serial.print(axis);
    // Serial.print(" axis:\t");
    // Serial.println(newAngle);
    return newAngle;
}; // end kalmanloop

// need to retrieve the offsets from the MPU6050
void getMPUOffsets(void)
{
    ax_offset = (int16_t)MPU2.getXAccelOffset();
    ay_offset = (int16_t)MPU2.getYAccelOffset();
    az_offset = (int16_t)MPU2.getZAccelOffset();
    gx_offset = (int16_t)MPU2.getXGyroOffset();
    gy_offset = (int16_t)MPU2.getYGyroOffset();
    gz_offset = (int16_t)MPU2.getZGyroOffset();

    // used for testing purposes only
    // Serial.print("X AccOffset: ");
    // Serial.print(ax_offset);
    // Serial.print("\tY AccOffset: ");
    // Serial.print(ay_offset);
    // Serial.print("\tZ AccOffset: ");
    // Serial.print(az_offset);
    // Serial.print("\tX GyroOffset: ");
    // Serial.print(gx_offset);
    // Serial.print("\tY GyroOffset: ");
    // Serial.print(gy_offset);
    // Serial.print("\tZ GyroOffset: ");
    // Serial.println(gz_offset);
}; // end getMPUOffsets

void setMPUOffsets(int16_t ax_offset2, int16_t ay_offset2, int16_t az_offset2, int16_t gx_offset2, int16_t gy_offset2, int16_t gz_offset2)
{
    MPU2.setXAccelOffset(ax_offset2);
    MPU2.setYAccelOffset(ay_offset2);
    MPU2.setZAccelOffset(az_offset2);
    MPU2.setXGyroOffset(gx_offset2);
    MPU2.setYGyroOffset(gy_offset2);
    MPU2.setZGyroOffset(gz_offset2);
}; // end setMPUOffsets

// unsure if I will use this function
void getMPUandHCSR04Data(void)
{
    // get the filtered data from the MPU6050
    filterData();
    // get the distance from the HCSR04 sensor
    getFilteredDistance();
    // get MPU Offsets;
    getMPUOffsets();

    // save it all unto file
    logSensorData(8); // Save the offset data to the config file on the SD Card
};

// unsure if I will use these functions
// Function to control the speed of the linear actuator movement
// int8_t ax_filter = 10; // Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
// int8_t ay_filter = 10; // Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
// int8_t az_filter = 10; // Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
// int8_t gx_filter = 10; // Gyro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)
// int8_t gy_filter = 10; // Gyro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)
// int8_t gz_filter = 10; // Gyro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)
// Function to see if the MPU-6050 is actually setup correctly
// bool isXAccelAboveThreshold = false;
// bool isYAccelAboveThreshold = false;
// bool isZAccelAboveThreshold = false;
// bool isXGyroAboveThreshold = false;
// bool isYGyroAboveThreshold = false;
// bool isZGyroAboveThreshold = false;
//
// uint8_t isXaccelaboveCount = 0;
// uint8_t isYaccelaboveCount = 0;
// uint8_t isZaccelaboveCount = 0;
// uint8_t isXgyroaboveCount = 0;
// uint8_t isYgyroaboveCount = 0;
// uint8_t isZgyroaboveCount = 0;
//
// int XaccelConsecutiveCount = 0;
// int YaccelConsecutiveCount = 0;
// int ZaccelConsecutiveCount = 0;
// int XgyroConsecutiveCount = 0;
// int YgyroConsecutiveCount = 0;
// int ZgyroConsecutiveCount = 0;
//
// void handleAxis(int16_t axisValue, int16_t axisFilter, bool &isAxisAboveThreshold, uint8_t &axisAboveCount, int &axisConsecutiveCount, String axis, void (*calibrateFunc)())
// {
//     if (abs(axisValue) > axisFilter)
//     {
//         Serial.print(axis);
//         Serial.println(" Axis is not at rest");
//         isAxisAboveThreshold = true;
//         axisAboveCount = 1;
//         axisConsecutiveCount++;
//     }
//     else if (abs(axisValue) < axisFilter)
//     {
//         Serial.print(axis);
//         Serial.println(" Axis is at rest");
//         isAxisAboveThreshold = false;
//         axisAboveCount = 0;
//         axisConsecutiveCount = 0;
//     }
//
//     if (isAxisAboveThreshold && axisAboveCount && axisConsecutiveCount > axisFilter)
//     {
//         Serial.print("Calibrating ");
//         Serial.println(axis);
//         calibrateFunc();
//     }
// };
// void getThresholdResets()
// {
//     // end of function calling is located in calibrateData.h (ex: calibrateXAccel)
//     handleAxis(ax, ax_filter, isXAccelAboveThreshold, isXaccelaboveCount, XaccelConsecutiveCount, "X Accel", calibrateXAccel);
//     handleAxis(ay, ay_filter, isYAccelAboveThreshold, isYaccelaboveCount, YaccelConsecutiveCount, "Y Accel", calibrateYAccel);
//     handleAxis(az, az_filter, isZAccelAboveThreshold, isZaccelaboveCount, ZaccelConsecutiveCount, "Z Accel", calibrateZAccel);
//     handleAxis(gx, gx_filter, isXGyroAboveThreshold, isXgyroaboveCount, XgyroConsecutiveCount, "Gyro X", calibrateXGyro);
//     handleAxis(gy, gy_filter, isYGyroAboveThreshold, isYgyroaboveCount, YgyroConsecutiveCount, "Gyro Y", calibrateYGyro);
//     handleAxis(gz, gz_filter, isZGyroAboveThreshold, isZgyroaboveCount, ZgyroConsecutiveCount, "Gyro Z", calibrateZGyro);
// };
//
// version 1
// void getThresholdResets(void)
// {
//     getMPUOffsets();
//     filterData(); // averaging raw outputs for 300 samples
//     if (abs(ax) > x_accel_filter)
//     {
//         Serial.println("X-Axis is not at rest");
//         isXAccelAboveThreshold = true;
//         isXaccelaboveCount = 1;
//         XaccelConsecutiveCount++;
//     }
//     else if (abs(ax) < x_accel_filter)
//     {
//         Serial.println("X-Axis is at rest");
//         isXAccelAboveThreshold = false;
//         isXaccelaboveCount = 0;
//         XaccelConsecutiveCount = 0;
//     }
//
//    if (abs(ay) > y_accel_filter)
//     {
//         Serial.println("Y-Axis is not at rest");
//         isYAccelAboveThreshold = true;
//         isYaccelaboveCount = 1;
//         YaccelConsecutiveCount++;
//     }
//     else if (abs(ay) < y_accel_filter)
//     {
//         Serial.println("Y-Axis is at rest");
//         isYAccelAboveThreshold = false;
//         isYaccelaboveCount = 0;
//         YaccelConsecutiveCount = 0;
//     }
//
//     if (abs(az) > z_accel_filter)
//     {
//         Serial.println("Z-Axis is not at rest");
//         isZAccelAboveThreshold = true;
//         isZaccelaboveCount = 1;
//         ZaccelConsecutiveCount++;
//     }
//     else if (abs(az) < z_accel_filter)
//     {
//         Serial.println("Z-Axis is at rest");
//         isZAccelAboveThreshold = false;
//         isZaccelaboveCount = 0;
//         ZaccelConsecutiveCount = 0;
//     }
//
//     if (isXAccelAboveThreshold && isXaccelaboveCount && XaccelConsecutiveCount > x_accel_filter)
//     {
//         calibrateXAccel(); // function located in calibrateData.h
//     }
//     if (isYAccelAboveThreshold && isYaccelaboveCount && YaccelConsecutiveCount > y_accel_filter)
//     {
//         calibrateYAccel(); // function located in calibrateData.h
//     }
//     if (isZAccelAboveThreshold && isZaccelaboveCount && ZaccelConsecutiveCount > y_accel_filter)
//     {
//         calibrateZAccel(); // function located in calibrateData.h
//     }
//     if (isXGyroAboveThreshold && isXgyroaboveCount && XgyroConsecutiveCount > x_gyro_filter)
//     {
//         calibrateXGyro(); // function located in calibrateData.h
//     }
//     if (isYGyroAboveThreshold && isYgyroaboveCount && YgyroConsecutiveCount > y_gyro_filter)
//     {
//         calibrateYGyro(); // function located in calibrateData.h
//     }
//     if (isZGyroAboveThreshold && isZgyroaboveCount && ZgyroConsecutiveCount > z_gyro_filter)
//     {
//         calibrateZGyro(); // function located in calibrateData.h
//     }
// };

void loadOffsets(void)
{
    File configFile = SD.open("/config.csv", FILE_READ);
    if (configFile)
    {
        String line;
        while (configFile.available())
        {
            line = configFile.readStringUntil('\n');
            // Assuming the offsets and base distance are stored in the last line of the file
        }
        configFile.close();

        // Parse the line to get the offsets and base distance
        // This depends on how you've formatted the data in the file
        int startPos = 0;
        int commaPos = line.indexOf(',', startPos);
        int index = 0;

        // from the dataLogger.h library
        // dataFile.print("Event,Raw AccX,Raw AccY,Raw AccZ,Raw GyroX,Raw GyroY,Raw GyroZ,Raw Temp,
        // Filter accel_x,Filter accel_y,Filter accel_z,Filter gyro_x,Filter gyro_y,Filter gyro_z,
        // Cal Mean AccX,Cal Mean AccY,Cal Mean AccZ,Cal Mean GyroX,Cal Mean GyroY,Cal Mean GyroZ,Cal AccelX Offset,Cal AccelY Offset,Cal AccelZ Offset,Cal GyroX Offset,Cal GyroY Offset,Cal GyroZ Offset,
        //  , , ,Is Table Stop,Is Table Moving Up,Is Table Moving Down,Is Table Stationary,Is Table Level,Is Table Max Pos,Is Table at Bottom,
        // Level Angle X,Level Angle Y,Current Distance CM,Current Distance Inch,Base Distance CM,Base Distance Inch,\n");
        // dataFile.print(" ,ax,ay,az,gx,gy,gz,tempC,
        // accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,
        // mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz,ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset,
        //  , , ,isStop,isMovingUp,isMovingDown,isStationary,isTableLevel,isTableMax,isBottomOut,
        // angleX,angleY,distanceCm,distanceInch,baseDistanceCm,baseDistanceInch,\n");
        // therefore: event = 0, ax = 1, ay = 2, az = 3, gx = 4, gy = 5, gz = 6, tempC = 7, accel_x = 8, accel_y = 9, accel_z = 10, gyro_x = 11, gyro_y = 12, gyro_z = 13,
        // mean_ax = 14, mean_ay = 15, mean_az = 16, mean_gx = 17, mean_gy = 18, mean_gx = 19, ax_offset = 20, ay_offset = 21, az_offset = 22, gx_offset = 23, gy_offset = 24, gz_offset = 25,
        // blank =  26, blank = 27, blank = 28, isStop = 29, isMovingUp = 30, isMovingDown = 31, isStationary = 32, isTableLevel = 33, isTableMax = 34, isBottomOut = 35, angleX = 36, angleY = 37
        // distanceCm = 38, distanceInch = 39, baseDistanceCm = 40, baseDistanceInch = 41
        while (commaPos != -1)
        {
            String value = line.substring(startPos, commaPos);
            // Update the corresponding offset or base distance with the value
            // This depends on the order of the values in the line
            // Mean values are in positions 14 to 19
            if (index == 14)
            {
                mean_ax2 = value.toInt();
            }
            else if (index == 15)
            {
                mean_ay2 = value.toInt();
            }
            else if (index == 16)
            {
                mean_az2 = value.toInt();
            }
            else if (index == 17)
            {
                mean_gx2 = value.toInt();
            }
            else if (index == 18)
            {
                mean_gy2 = value.toInt();
            }
            else if (index == 19)
            {
                mean_gz2 = value.toInt();
            }
            // Offsets are in positions 20 to 25
            else if (index == 20)
            {
                ax_offset2 = value.toInt();
            }
            else if (index == 21)
            {
                ay_offset2 = value.toInt();
            }
            else if (index == 22)
            {
                az_offset2 = value.toInt();
            }
            else if (index == 23)
            {
                gx_offset2 = value.toInt();
            }
            else if (index == 24)
            {
                gy_offset2 = value.toInt();
            }
            else if (index == 25)
            {
                gz_offset2 = value.toInt();
            }
            // base distances are in positions 40 and 41
            // else if (index == 40)
            // {
            //    // baseDistanceCm is in position 40
            //     baseDistanceCm = value.toFloat();
            // }
            // else if (index == 41)
            // {
            //     // baseDistanceInch is in position 41
            //     baseDistanceInch = value.toFloat();
            // }

            startPos = commaPos + 1;
            commaPos = line.indexOf(',', startPos);
            index++;
        }
    }
    else
    {
        Serial.println("Error: Could not open config.csv");
    }

    // used for testing purposes only
    // Serial.print("Mean Accel X: ");
    // Serial.print(mean_ax2);
    // Serial.print("\tMean Accel Y: ");
    // Serial.print(mean_ay2);
    // Serial.print("\tMean Accel Z: ");
    // Serial.print(mean_az2);
    // Serial.print("\tMean Gyro X: ");
    // Serial.print(mean_gx2);
    // Serial.print("\tMean Gyro Y: ");
    // Serial.print(mean_gy2);
    // Serial.print("\tMean Gyro Z: ");
    // Serial.println(mean_gz2);
    // Serial.print("Accel X Offset: ");
    // Serial.print(ax_offset2);
    // Serial.print("\tAccel Y Offset: ");
    // Serial.print(ay_offset2);
    // Serial.print("\tAccel Z Offset: ");
    // Serial.print(az_offset2);
    // Serial.print("\tGyro X Offset: ");
    // Serial.print(gx_offset2);
    // Serial.print("\tGyro Y Offset: ");
    // Serial.print(gy_offset2);
    // Serial.print("\tGyro Z Offset: ");
    // Serial.println(gz_offset2);

    // Set the offsets
    // setMPUOffsets(ax_offset2, ay_offset2, az_offset2, gx_offset2, gy_offset2, gz_offset2);
    MPU2.setXAccelOffset(ax_offset2);
    MPU2.setYAccelOffset(ay_offset2);
    MPU2.setZAccelOffset(az_offset2);
    MPU2.setXGyroOffset(gx_offset2);
    MPU2.setYGyroOffset(gy_offset2);
    MPU2.setZGyroOffset(gz_offset2);
}; // end loadOffsets

void loadDistances(void)
{
    File configFile = SD.open("/config.csv", FILE_READ);
    if (configFile)
    {
        String line;
        while (configFile.available())
        {
            line = configFile.readStringUntil('\n');
            // Assuming the offsets and base distance are stored in the last line of the file
        }
        configFile.close();

        // Parse the line to get the offsets and base distance
        // This depends on how you've formatted the data in the file
        int startPos = 0;
        int commaPos = line.indexOf(',', startPos);
        int index = 0;

        // from the dataLogger.h library
        // configFile.print("Event,Raw AccX,Raw AccY,Raw AccZ,Raw GyroX,Raw GyroY,Raw GyroZ,Raw Temp,
        // Filter accel_x,Filter accel_y,Filter accel_z,Filter gyro_x,Filter gyro_y,Filter gyro_z,
        // Cal Mean AccX,Cal Mean AccY,Cal Mean AccZ,Cal Mean GyroX,Cal Mean GyroY,Cal Mean GyroZ,Cal AccelX Offset,Cal AccelY Offset,Cal AccelZ Offset,Cal GyroX Offset,Cal GyroY Offset,Cal GyroZ Offset,
        //  , , ,Is Table Stop,Is Table Moving Up,Is Table Moving Down,Is Table Stationary,Is Table Level,Is Table Max Pos,Is Table at Bottom,
        // Level Angle X,Level Angle Y,Current Distance CM,Current Distance Inch,Base Distance CM,Base Distance Inch,\n");
        // configFile.print(" ,ax,ay,az,gx,gy,gz,tempC,
        // accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,
        // mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz,ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset,
        //  , , ,isStop,isMovingUp,isMovingDown,isStationary,isTableLevel,isTableMax,isBottomOut,
        // angleX,angleY,distanceCm,distanceInch,baseDistanceCm,baseDistanceInch,\n");
        // therefore: event = 0, ax = 1, ay = 2, az = 3, gx = 4, gy = 5, gz = 6, tempC = 7, accel_x = 8, accel_y = 9, accel_z = 10, gyro_x = 11, gyro_y = 12, gyro_z = 13,
        // mean_ax = 14, mean_ay = 15, mean_az = 16, mean_gx = 17, mean_gy = 18, mean_gx = 19, ax_offset = 20, ay_offset = 21, az_offset = 22, gx_offset = 23, gy_offset = 24, gz_offset = 25,
        // blank =  26, blank = 27, blank = 28, isStop = 29, isMovingUp = 30, isMovingDown = 31, isStationary = 32, isTableLevel = 33, isTableMax = 34, isBottomOut = 35, angleX = 36, angleY = 37
        // distanceCm = 38, distanceInch = 39, baseDistanceCm = 40, baseDistanceInch = 41
        while (commaPos != -1)
        {
            String value = line.substring(startPos, commaPos);
            // Update the corresponding offset or base distance with the value
            // This depends on the order of the values in the line
            if (index == 40)
            {
                // baseDistanceCm is in position 40
                baseDistanceCm = value.toFloat();
            }
            else if (index == 41)
            {
                // baseDistanceInch is in position 41
                baseDistanceInch = value.toFloat();
            }

            // Move to the next value in the line
            startPos = commaPos + 1;
            commaPos = line.indexOf(',', startPos);
            index++;
        }
    }
    else
    {
        Serial.println("Error: Could not open config.csv");
    }

    // used for testing purposes only
    // Serial.print("Base_Distance_in_CM:\t");
    // Serial.print(baseDistanceCm);
    // Serial.print("\tBase_Distance_in_Inch:\t");
    // Serial.println(baseDistanceInch);
}; // end loadDistances

// ============================================
// THIS PART IS NOT NEEDED BUT IT IS A GOOD EXAMPLE OF HOW TO GET THE ANGLE FROM THE MPU6050
// FOR STABILIZING THE TABLE
//
// unsigned long lastTime = 0;
//--------------------------------------------//
// Get angle from the gyroscope. Uint: degree
//--------------------------------------------//
// float getGyroRoll(int gyroX, int gyroBiasX, uint32_t lastTime)
// {
//     float gyroRoll;
//
//     // integrate gyroscope value in order to get angle value
//     gyroRoll = ((gyroX - gyroBiasX) / 131) * ((float)(micros() - lastTime) / 1000000);
//     return gyroRoll;
// };
//
//-----------------------------------------------//
// Get angle from the accelerometer. Uint: degree
//-----------------------------------------------//
// float getAccRoll(int accY, int accBiasY, int accZ, int accBiasZ)
// {
//     float accRoll;
//
//     // calculate angle value
//     accRoll = (atan2((accY - accBiasY), (accZ - accBiasZ))) * RAD_TO_DEG;
//
//     if (accRoll <= 360 && accRoll >= 180)
//     {
//         accRoll = 360 - accRoll;
//     }
//     return accRoll;
// };
//
//--------------------------------------------//
// Get current angle of the robot
//--------------------------------------------//
// float currentAngle()
// {
//
//     // Get raw IMU data
//     // readIMU();
//     MPU3.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
//
//     // Complementary filter for angle calculation
//     float gRoll = getGyroRoll(gx, gx_offset, lastTime);
//     float aRoll = getAccRoll(ay, ay_offset, az, az_offset);
//
//     float angleGet = 0.98 * (angleGet + gRoll) + 0.02 * (aRoll);
//
//     lastTime = micros(); // Reset the timer
//
//     return angleGet;
// };
// ============================================

// =========================================================
// FROM: http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
// =========================================================
// NOTE: ALL THIS CODE IS LOCATED IN PIDv1.h OF THE ARDUINO LIBRARY
// 
/*working variables*/
// unsigned long PIDlastTime;
// double Input, Output, Setpoint;
// double lastInput, outputSum;
// double kp, ki, kd;
// int SampleTime = 1000; // 1 sec
// double outMin, outMax;
// bool inAuto = false;
// 
// #define MANUAL 0
// #define AUTOMATIC 1
// 
// #define DIRECT 0
// #define REVERSE 1
// int controllerDirection = DIRECT;
// 
// #define P_ON_M 0
// #define P_ON_E 1
// bool pOnE = true, pOnM = false;
// double pOnEKp, pOnMKp;
// 
// void Compute()
// {
//     if (!inAuto)
//     {
//         return; // exit the function
//     }
//     /*How long since we last calculated*/
//     unsigned long now = millis();
//     int timeChange = (now - PIDlastTime);
// 
//     if (timeChange >= SampleTime)
//     {
//         /*Compute all the working error variables*/
//         double error = Setpoint - Input;
//         double dInput = (Input - lastInput);
//         outputSum = +(ki * error);
// 
//         /*Add Proportional on Measurement, if P_ON_M is specified*/
//         if (pOnM)
//         {
//             outputSum -= pOnMKp * dInput;
//         }
// 
//         if (outputSum > outMax)
//         {
//             outputSum = outMax;
//         }
//         else if (outputSum < outMin)
//         {
//             outputSum = outMin;
//         }
// 
//         /*Add Proportional on Error, if P_ON_E is specified*/
//         if (pOnE)
//         {
//             Output = pOnEKp * error;
//         }
//         else
//         {
//             Output = 0;
//         }
// 
//         /*Compute Rest of PID Output*/
//         Output += outputSum - kd * dInput;
//         if (Output > outMax)
//         {
//             Output = outMax;
//         }
//         else if (Output < outMin)
//         {
//             Output = outMin;
//         }
// 
//         /*Remember some variables for next time*/
//         lastInput = Input;
//         PIDlastTime = now;
//     }
// }; // end Compute
// 
// void SetTunings(double Kp, double Ki, double Kd, double pOn)
// {
//     if (Kp < 0 || Ki < 0 || Kd < 0 || pOn < 0 || pOn > 1)
//     {
//         return; // exit the function
//     }
// 
//     pOnE = pOn > 0; // some p on error is desired;
//     pOnM = pOn < 1; // some p on measurement is desired;
// 
//     double SampleTimeInSec = ((double)SampleTime) / 1000;
//     kp = Kp;
//     ki = Ki * SampleTimeInSec;
//     kd = Kd / SampleTimeInSec;
// 
//     if (controllerDirection == REVERSE)
//     {
//         kp = (0 - kp);
//         ki = (0 - ki);
//         kd = (0 - kd);
//     }
// 
//     pOnEKp = pOn * kp;
//     pOnMKp = (1 - pOn) * kp;
// }; // end SetTunings
// 
// void SetSampleTime(int NewSampleTime)
// {
//     if (NewSampleTime > 0)
//     {
//         double ratio = (double)NewSampleTime / (double)SampleTime;
//         ki *= ratio;
//         kd /= ratio;
//         SampleTime = (unsigned long)NewSampleTime;
//     }
// }; // end SetSampleTime
// 
// void SetOutputLimits(double Min, double Max)
// {
//     if (Min > Max)
//     {
//         return; // exit the function
//     }
//     outMin = Min;
//     outMax = Max;
// 
//     if (Output > outMax)
//     {
//         Output = outMax;
//     }
//     else if (Output < outMin)
//     {
//         Output = outMin;
//     }
// 
//     if (outputSum > outMax)
//     {
//         outputSum = outMax;
//     }
//     else if (outputSum < outMin)
//     {
//         outputSum = outMin;
//     }
// }; // end SetOutputLimits
// 
// void SetControllerDirection(int Direction)
// {
//     controllerDirection = Direction;
// }; // end SetControllerDirection
// 
// void PIDInitialize()
// {
//     lastInput = Input;
//     outputSum = Output;
//     if (outputSum > outMax)
//     {
//         outputSum = outMax;
//     }
//     else if (outputSum < outMin)
//     {
//         outputSum = outMin;
//     }
// }; // end Initialize
// 
// void SetMode(int Mode)
// {
//     bool newAuto = (Mode == AUTOMATIC);
//     if (newAuto && !inAuto)
//     {
//         /*we just went from manual to auto*/
//         PIDInitialize();
//     }
//     inAuto = (Mode == AUTOMATIC);
// }; // end SetMode
// 
// void PIDSetup(void)
// {
//     // unsigned long setTime = millis();
//     SetSampleTime(1000);
//     SetOutputLimits(0, 255);
//     SetMode(AUTOMATIC);
// };
// 
// void PIDloop()
// {
//     PIDInitialize();
    // 
//     Compute();
//     Output = 0;
// }; // end loop

#endif // USE_LIBRARY2

#endif // FILTERDATA_H

// =========================================================
// END OF PROGRAM
// =========================================================