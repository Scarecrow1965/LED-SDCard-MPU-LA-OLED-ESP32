// +-------------------------------------------------------------
//
// Equipment:
// DOIT ESP32 Dev Kit V1, MPU-6050, OLED SSD1306, WS2812B Addressable LEDS x3,
// 4-relay module, 2 linear actuators, RT-11(JCHT35K9) remote/handset,
// and 650W 12V Power supply
//
// File: MPUData.h
//
// Description:
//
// Provides the computer workstation the ability to use the remote/handset,
// MPU-6050, and the linear actuators to control the movement of the standup
// portion of the workstation.
//
// History:     7-Mar-2024     Scarecrow1965   Created
//
// +-------------------------------------------------------------

#ifndef LINEARACTUATOR_H
#define LINEARACTUATOR_H

#include <Arduino.h>
#include <Wire.h>

#include "FastLED.h"
#include "LEDeffects.h"
void displayPosition(void);
void redAlert2(void);

// from main.cpp variables
extern int16_t ax, ay, az, gx, gy, gz;
extern double tempC;
extern bool isCalibrationLevel;
extern String fileName;
extern File dataFile;
extern File configFile;

// #include <SPI.h>
#include <SD.h>

#include "filterData.h"
void filterData(void);
float kalmanloop(int16_t raw, int16_t gyro, float (*calculateAngle)(int16_t, int16_t), String axis);
extern double angleX;
extern int16_t accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z;

#include "dataLogger.h"
// #include "MPU6050.h"
// #include "calibrateData.h"

#include "ultrasonic.h"
extern float distanceCm;
extern float baseDistanceCm;
extern const float maxDistanceCm, bottomOutDistanceCm;
void getFilteredDistance(void);
void getRawDistance(void);

#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>

// ================
// RT-11(JCHT35K9) remote/handset
// ================
// #include "rt11.h"
// from: https://www.devmire.com/2021/10/03/reverse-engineering-a-standing-desk-for-fun-and-profit/
// #include <SoftwareSerial.h> // commented out since it was interfering with comms
#include <HardwareSerial.h>
#define MAX_MILLIS_TO_WAIT 1000 // or whatever
// #define DOWN_PIN 12 // original code
#define DOWN_PIN 15 // GPIO output pin for the DOWN button
int pinState1 = digitalRead(DOWN_PIN); // created for datalogging purposes only
// #define UP_PIN 13 // original code
#define UP_PIN 4 // GPIO output pin for the UP button
int pinState2 = digitalRead(UP_PIN); // created for datalogging purposes only
#define UPOP 1
#define DNOP 2
#define NOOP 0
#define ACTIVATE LOW
#define DEACTIVATE HIGH
#define DESKMOVEDDURINGREAD 9 // refers to the inches the desk moves during the time it takes to read the height
// const int led = LED_BUILTIN; // commented out since I already have GPIO2 aka LED_BUILTIN used
int currentOperation = NOOP;
unsigned long starttime;
unsigned long value;
int RFin_bytes[4]; // The message 4 bytes long
int operation = 0;
int currentHeight = 0;
int requestedHeight = 0;
int heightMultiplier = 1;
// extern SoftwareSerial deskSerial; // this does not work since we have hardware serial
extern HardwareSerial deskSerial;
// ================

// ===================================================
// Setup for relay Module and linear actuator commands
// ===================================================

// Pin assignments for relays controlling linear actuators
// Legend: LA = Linear Actuator, LHP = Left Actuator Positive, LHM = Left Actuator Negative, RHP = Right Actuator Positive, RHM = Right Actuator Negative
const int relay1_LA1LHP_Pin = 27; // Relay controlling actuator #1 positive // it is actually pin 6 on the DOIT ESP32 Dev Kit V1 or GPIO27
const int relay2_LA1LHM_Pin = 26; // Relay controlling actuator #1 negative // it is actually pin 7 on the DOIT ESP32 Dev Kit V1 or GPIO26
const int relay3_LA2RHP_Pin = 25; // Relay controlling actuator #2 positive // it is actually pin 8 on the DOIT ESP32 Dev Kit V1 or GPIO25
const int relay4_LA2RHM_Pin = 33; // Relay controlling actuator #2 negative // it is actually pin 9 on the DOIT ESP32 Dev Kit V1 or GPIO33

// Variables to control linear actuator movement
String isLeftActuator = "STOP";  // or "UP" or "DOWN" or " "
String isRightActuator = "STOP"; // or "UP" or "DOWN" or " "
bool isStop = false;
bool isMovingUp = false;
bool isMovingDown = false;
bool isStationary = false;
bool isTableLevel = false;
bool isTableMax = false;
bool isBottomOut = false;

// Variables to indicate if the table is level
const double x_angle_max = 3.0;     // should equal to 3 degrees
const double x_angle_ok = 1.0;      // should equal to 1 degrees
const double x_angle_perfect = 0.5; // should equal to 0.25 degrees
bool isDistanceOK = false;
bool isAngleOK = false;

// Threshold values for detecting motion
// const int threshold = 5; // Adjust this value according to your needs
// const int threshold = 3; // Adjust this value according to your needs
// float gyroThresholdX, gyroThresholdY, gyroThresholdZ; // to store the gyro threshold, not set yet
// float accelThresholdX, accelThresholdY, accelThresholdZ; // to store the accelerometer threshold, not set yet

// to ensure the linear actuators are moving
// const int BASE_SPEED = 100;          // Base speed for linear actuator movement = 100%
// const int MAX_SPEED_ADJUSTMENT = 15; // Maximum speed adjustment for linear actuator movement = 15%

// ================
// Timer variables
// unsigned long lastTime = 0;
// unsigned long lastTimeTemperature = 0;
// unsigned long lastTimeAcc = 0;
// unsigned long gyroDelay = 10;
// unsigned long temperatureDelay = 1000;
// unsigned long accelerometerDelay = 200;
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
unsigned long commandsPreviousMillis = 0;
unsigned long displayPreviousMillis = 0;

// delay equivalence
const unsigned int timePeriod002 = 2;    // delay equal to 2 ms
const unsigned int timePeriod005 = 5;    // delay equal to 5 ms
const unsigned int timePeriod010 = 10;   // Delay equal to 10 ms
const unsigned int timePeriod020 = 15;   // Delay equal to 15 ms
const unsigned int timePeriod050 = 50;   // Delay equal to 50 ms
const unsigned int timePeriod100 = 100;  // Delay equal to 100 ms
const unsigned int timePeriod250 = 250;  // Delay equal to 1/4 second
const unsigned int timePeriod500 = 500;  // Delay equal to 1/2 second
const unsigned int timePeriod1 = 1000;   // Delay equal to 1 sec pause
const unsigned int timePeriod2 = 2000;   // Delay equal to 2 second pause
const unsigned int timePeriod5 = 5000;   // Delay equal to 5 seconds pause
const unsigned int timePeriod10 = 10000; // Delay equal to 10 seconds pause
const unsigned int timePeriod15 = 15000; // Delay equal to 15 seconds duration
const unsigned int timePeriod30 = 30000; // Delay equal to 30 second duration
// ================

// ================
// these variables are used to store the position of the MPU-6050
// Variables to store current and previous position
float nowDistanceCm = 0;
float targetDistance = 0;
const float dist_tolerance_level = 0.5;       // 0.5 cm tolerance
const float distance_tolerance = 1.0;         // was 0.5 cm tolerance
const float refined_distance_tolerance = .75; // 0.2 cm tolerance
// const float closetoBottomOutPosition = 5.0;              // 5 cm close to the bottom out position
// float lower_base_distance = baseDistanceCm - 1.54; // 1.54 cm below the bottom out position
// float upper_base_distance = baseDistanceCm + 1.54; // 1.54 cm above the bottom out position
// int previousPosition = 0;
// int newPosition = currentPosition;
// ================

// ================
// this is to check if the data from the MPU-6050 has changed and allow LEDs to work
bool dataChanged = false;
// ================

// ================
// these functions are used to check the boolean values
// used for testing purposes only
void boolChecker(void)
{
    Serial.println("rt11Working:\t" + String(rt11Working ? "True" : "False"));
    // Serial.println("DOWN_PIN:\t" + String(DOWN_PIN));
    Serial.println("DOWN_PIN:\t" + String(pinState1 ? "ON" : "OFF"));
    // Serial.println("UP_PIN:\t" + String(UP_PIN));
    Serial.println("UP_PIN:\t" + String(pinState2 ? "ON" : "OFF"));
    Serial.println("isLeftActuator:\t" + isLeftActuator);
    Serial.println("isRightActuator:\t" + isRightActuator);
    Serial.println("isStop:\t" + String(isStop ? "True" : "False"));
    Serial.println("isMovingUp:\t" + String(isMovingUp ? "True" : "False"));
    Serial.println("isMovingDown:\t" + String(isMovingDown ? "True" : "False"));
    Serial.println("isStationary:\t" + String(isStationary ? "True" : "False"));
    Serial.println("isTableLevel:\t" + String(isTableLevel ? "True" : "False"));
    Serial.println("isTableMax:\t" + String(isTableMax ? "True" : "False"));
    Serial.println("isBottomOut:\t" + String(isBottomOut ? "True" : "False"));
    
    // template for boolChecker function is:
    // bool rt11Working
    // int pinstate1 = PIN_DOWN
    // int pinState2 = PIN_UP
    // String isLeftActuator
    // String isRightActuator
    // bool isStop
    // bool isMovingUp
    // bool isMovingDown
    // bool isStationary
    // bool isTableLevel
    // bool isTableMax
    // bool isBottomOut

}; // end boolChecker
// ================

// ================
// the following functions are used to provide information for the Acceleration data and to calculate the angle at which the gyro/accelerometers are positioned
// volatile bool getAccelerationDataTaskRunning = false;
// 
// this is necessary for the proper calculations of the Kalmanfilter to work.
float calculateXAngle(int16_t accel_x, int16_t accel_z)
{
    return atan2(accel_x, accel_z);
}; // end calculateXAngle

float calculateYAngle(int16_t accel_y, int16_t accel_z)
{
    return atan2(accel_y, accel_z);
}; // end calculateYAngle

float calculateZAngle(int16_t accel_z, int16_t accel_y)
{
    return atan2(accel_z, accel_y);
}; // end calculateZAngle
// 
// double velX = 0, velY = 0, velZ = 0; // velocity
// double posX = 0, posY = 0, posZ = 0; // position
// double deltaTime, axFiltered, ayFiltered, azFiltered;
// 
// The following code is for the linear actuator movement
// void getAccelerationData(void)
// {
//     filterData();
//     currentMillis = millis() / 1000.0;          // get current time in seconds
//     deltaTime = currentMillis - previousMillis; // calculate time difference
//
//     // Apply the Kalman filters to the accelerometer data
//     axFiltered = kalmanloop(accel_x, gyro_x, calculateXAngle, "X");
//     ayFiltered = kalmanloop(accel_y, gyro_y, calculateYAngle, "Y");
//     azFiltered = kalmanloop(accel_z, gyro_z, calculateZAngle, "Z");
//
//     // ================
//     // used for testing purposes only
//     // integrate acceleration to get velocity
//     // Serial.println("getAccelerationData");
//     // Serial.println("MPU RAW data:");
//     // Serial.print("Accel X:\t");      // used for testing purposes only
//     // Serial.print(ax);                // used for testing purposes only
//     // Serial.print("\tAccel Y:\t");    // used for testing purposes only
//     // Serial.print(ay);                // used for testing purposes only
//     // Serial.print("\tAccel Z:\t");    // used for testing purposes only
//     // Serial.print(az);                // used for testing purposes only
//     // Serial.print("\tGyro X:\t");     // used for testing purposes only
//     // Serial.print(gx);                // used for testing purposes only
//     // Serial.print("\tGyro Y:\t");     // used for testing purposes only
//     // Serial.print(gy);                // used for testing purposes only
//     // Serial.print("\tGyro Z:\t");     // used for testing purposes only
//     // Serial.println(gz);              // used for testing purposes only
//     // Serial.print("axFiltered:\t");   // used for testing purposes only
//     // Serial.print(axFiltered);        // used for testing purposes only
//     // Serial.print("\tayFiltered:\t"); // used for testing purposes only
//     // Serial.print(ayFiltered);        // used for testing purposes only
//     // Serial.print("\tazFiltered:\t"); // used for testing purposes only
//     // Serial.println(azFiltered);      // used for testing purposes only
//     // save to file
//     // version 1
//     // dataFile.println("getAccelerationData: ");
//     // dataFile.flush();              // Immediately write all data in the cache to the disk
//     // dataFile.print("Accel X:\t");  // used for testing purposes only
//     // dataFile.print(accel_x);           // used for testing purposes only
//     // dataFile.flush();             // Immediately write all data in the cache to the disk
//     // dataFile.print("\tAccel Y:\t"); // used for testing purposes only
//     // dataFile.print(accel_y);             // used for testing purposes only
//     // dataFile.flush();               // Immediately write all data in the cache to the disk
//     // dataFile.print("\tAccel Z:\t"); // used for testing purposes only
//     // dataFile.print(accel_z);             // used for testing purposes only
//     // dataFile.flush();               // Immediately write all data in the cache to the disk
//     // dataFile.print("\tGyro x:\t");     // used for testing purposes only
//     // dataFile.print(gyro_x);             // used for testing purposes only
//     // dataFile.flush();               // Immediately write all data in the cache to the disk
//     // dataFile.print("\tGyro y:\t");  // used for testing purposes only
//     // dataFile.print(gyro_y);             // used for testing purposes only
//     // dataFile.flush();               // Immediately write all data in the cache to the disk
//     // dataFile.print("\tGyro z:\t");  // used for testing purposes only
//     // dataFile.println(gyro_z);          // used for testing purposes only
//     // dataFile.flush();              // Immediately write all data in the cache to the disk
//     // ================
//
//     // Integrate the filtered acceleration to get velocity
//     velX += axFiltered * deltaTime;
//     velY += ayFiltered * deltaTime;
//     velZ += azFiltered * deltaTime;
//     // unfiltered acceleration
//     // velX += ax * deltaTime;
//     // velY += ay * deltaTime;
//     // velZ += az * deltaTime;
//
//     // ================
//     // used for testing purposes only
//     // see the output
//     // Serial.print("deltaTime:\t");    // used for testing purposes only
//     // Serial.print(deltaTime);         // used for testing purposes only
//     // Serial.print("\tVelocity X:\t"); // used for testing purposes only
//     // Serial.print(velX);              // used for testing purposes only
//     // Serial.print("\tVelocity Y:\t"); // used for testing purposes only
//     // Serial.print(velY);              // used for testing purposes only
//     // Serial.print("\tVelocity Z:\t"); // used for testing purposes only
//     // Serial.println(velZ);            // used for testing purposes only
//     // save to file
//     // version 1
//     // dataFile.print("deltaTime:\t");     // used for testing purposes only
//     // dataFile.print(deltaTime);         // used for testing purposes only
//     // dataFile.flush();                  // Immediately write all data in the cache to the disk
//     // dataFile.print("\tVelocity X:\t"); // used for testing purposes only
//     // dataFile.print(velX);              // used for testing purposes only
//     // dataFile.flush();                  // Immediately write all data in the cache to the disk
//     // dataFile.print("\tVelocity Y:\t"); // used for testing purposes only
//     // dataFile.print(velY);              // used for testing purposes only
//     // dataFile.flush();                  // Immediately write all data in the cache to the disk
//     // dataFile.print("\tVelocity Z:\t"); // used for testing purposes only
//     // dataFile.println(velZ);            // used for testing purposes only
//     // dataFile.flush();                  // Immediately write all data in the cache to the disk
//     // ================
//
//     // integrate velocity to get position
//     posX += velX * deltaTime;
//     posY += velY * deltaTime;
//     posZ += velZ * deltaTime;
//
//     // ================
//     // used for testing purposes only
//     // see the output
//     // Serial.print("Position X:\t");   // used for testing purposes only
//     // Serial.print(posX);              // used for testing purposes only
//     // Serial.print("\tPosition Y:\t"); // used for testing purposes only
//     // Serial.print(posY);              // used for testing purposes only
//     // Serial.print("\tPosition Z:\t"); // used for testing purposes only
//     // Serial.println(posZ);            // used for testing purposes only
//     // save to file
//     // version 1
//     // dataFile.print("Position X:\t"); // used for testing purposes only
//     // dataFile.print(posX);              // used for testing purposes only
//     // dataFile.flush();                  // Immediately write all data in the cache to the disk
//     // dataFile.print("\tPosition Y:\t"); // used for testing purposes only
//     // dataFile.print(posY);              // used for testing purposes only
//     // dataFile.flush();                  // Immediately write all data in the cache to the disk
//     // dataFile.print("\tPosition Z:\t"); // used for testing purposes only
//     // dataFile.println(posZ);            // used for testing purposes only
//     // dataFile.flush();                  // Immediately write all data in the cache to the disk
//     // if (dataFile)
//     // {
//     //     Serial.println("During Acceleration, Data written to " + fileName);
//     // }
//     // else
//     // {
//     //     Serial.println("During Acceleration, Error writing to " + fileName);
//     // }
//     // version 2
//     // logSensorData(4); // this is to get the ACCELERATION DATA to the SD Card logging file
//     logSensorData(7); // this is to save ALL the DATA into logging file
//     // ================
//
//     previousMillis = currentMillis; // update previous time
// }; // end getAccelerationData

// void getAccelerationDataTask(void *parameter)
// {
//     TickType_t xLastWakeTime;
//     const TickType_t xFrequency = pdMS_TO_TICKS(100); // 100ms period
//
//     // Initialise the xLastWakeTime variable with the current time.
//     xLastWakeTime = xTaskGetTickCount();
//
//     while (getAccelerationDataTaskRunning)
//     {
//         getAccelerationData();
//
//         // carrying out a 10 ms delay
//         // currentMillis = millis();
//         // if (currentMillis - previousMillis >= timePeriod010)
//         // {
//         //     // delay(100); // Delay to prevent overloading the processor
//         //     previousMillis = currentMillis;
//         // }
//
//         // Wait for the next cycle.
//         vTaskDelayUntil(&xLastWakeTime, xFrequency);
//     }
//     vTaskDelete(NULL); // Delete this task if the loop is exited
// }; // end getAccelerationDataTask
// ================

// ===================================================

// int16_t previousGyroX = 0, previousGyroY = 0, previousAccX = 0, previousAccY = 0; // to help with providing an interrupt
// // ensure these filetered values are identical to those in the filterData.h file
// uint8_t x_accel_filter = 20; // Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
// uint8_t y_accel_filter = 20; // Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
// uint8_t z_accel_filter = 10; // Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
// uint8_t x_gyro_filter = 10;  // Gyro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)
// uint8_t y_gyro_filter = 10;  // Gyro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)
// uint8_t z_gyro_filter = 10;  // Gyro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)
// version 1
//  to check to see if there is a change in the table position
// void checkMPUData(bool &dataChanged)
// {
//     filteredData.filterData(); // reads the filtered MPU-6050 // Read gyro and accelerometer data
//     filterData(); // reads the filtered MPU-6050 // Read gyro and accelerometer data
//
//     // Check if gyro or accelerometer data has changed beyond the thresholds
//     if (abs(gx - previousGyroX) > x_gyro_filter ||
//         abs(gy - previousGyroY) > y_gyro_filter ||
//         abs(ax - previousAccX) > x_accel_filter ||
//         abs(ay - previousAccY) > y_accel_filter)
//     {
//         // Data has changed
//         previousGyroX = gx;
//         previousGyroY = gy;
//         previousAccX = ax;
//         previousAccY = ay;
//         dataChanged = true;
//         Serial.println("Data has changed"); // used for testing purposes only
//     }
//     else
//     {
//         // Data has not changed
//         dataChanged = false;
//         Serial.println("Data has NOT Changed"); // used for testing purposes only
//     }
// }; // end boolean to check if gyro or accel from mpu has changed position

void checkTableMovement(bool &dataChanging)
{
    boolChecker(); // used for testing purposes only
    
    // Check if the table is moving
    if (isMovingUp || isMovingDown || dataChanging)
    {
        // Data is changing
        dataChanged = true;
        Serial.println("Table is moving"); // used for testing purposes only
    }
    else if (isStop || isStationary)
    {
        // table is not moving therefore
        // Data has not changed
        dataChanged = false;
        Serial.println("Table has NOT Moved"); // used for testing purposes only
    }
}; // end boolean to check if the table has moved
// ===================================================

// ===================================
// FUNCTIONS FOR MOVEMENT OF THE TABLE
// ===================================

void moveActuators(bool leftUp, bool rightUp)
{
    if (leftUp)
    {
        // isLeftActuator = "UP";
        Serial.println("Moving Left Actuator Up"); // used for testing purposes only
        // LA1LH -> if true then, move up
        digitalWrite(relay1_LA1LHP_Pin, LOW);
        digitalWrite(relay2_LA1LHM_Pin, HIGH); // redundant protection
    }
    else
    {
        // isLeftActuator = "DOWN";
        Serial.println("Moving Left Actuator Down"); // used for testing purposes only
        // LA1LH -> if false then, move down
        digitalWrite(relay1_LA1LHP_Pin, HIGH); // redundant protection
        digitalWrite(relay2_LA1LHM_Pin, LOW);
    }
    if (rightUp)
    {
        // isRightActuator = "UP";
        Serial.println("Moving Right Actuator Up"); // used for testing purposes only
        // LA2RH -> if true then, move up
        digitalWrite(relay3_LA2RHP_Pin, LOW);
        digitalWrite(relay4_LA2RHM_Pin, HIGH); // redundant protection
    }
    else
    {
        // isRightActuator = "DOWN";
        Serial.println("Moving Right Actuator Down"); // used for testing purposes only
        // LA2RH -> if false then, move down
        digitalWrite(relay3_LA2RHP_Pin, HIGH); // redundant protection
        digitalWrite(relay4_LA2RHM_Pin, LOW);
    }

    // the follwoing comments are used for referencing Linear Actuators in other functions
    // LA1LH -> if true then, move up
    // digitalWrite(relay1_LA1LHP_Pin, LOW);
    // digitalWrite(relay2_LA1LHM_Pin, HIGH); // redundant protection
    // LA1LH -> if false then, move down
    // digitalWrite(relay1_LA1LHP_Pin, HIGH); // redundant protection
    // digitalWrite(relay2_LA1LHM_Pin, LOW);
    // LA2RH -> if true then, move up
    // digitalWrite(relay3_LA2RHP_Pin, LOW);
    // digitalWrite(relay4_LA2RHM_Pin, HIGH); // redundant protection
    // LA2RH -> if false then, move down
    // digitalWrite(relay3_LA2RHP_Pin, HIGH); // redundant protection
    // digitalWrite(relay4_LA2RHM_Pin, LOW);

    // this statement is already set up for the stopMovement function
    // if (!leftUp && !rightUp)
    // {
    //     Serial.println("Table Stops moving"); // used for testing purposes only
    //     // Stop the movement
    //     digitalWrite(relay1_LA1LHP_Pin, HIGH);
    //     digitalWrite(relay2_LA1LHM_Pin, HIGH);
    //     digitalWrite(relay3_LA2RHP_Pin, HIGH);
    //     digitalWrite(relay4_LA2RHM_Pin, HIGH);
    //     delay(100); // delay for 100 milli second to let the table adjust
    // }
}; // end moveActuators

void levelTable(void)
{
    Serial.println("Leveling Table function starts"); // used for testing purposes only

    // template for boolChecker function is:
    // bool rt11Working
    // int pinstate1
    // int pinState2
    isLeftActuator = " ";
    isRightActuator = " ";
    isStop = false;
    isMovingUp = false;
    isMovingDown = false;
    isStationary = false;
    isTableLevel = false;
    // bool isTableMax
    // bool isBottomOut

    while (!isTableLevel)
    {
        // ================
        // version without the selfBalancing.h library
        // ================

        // getting the filtered data from the MPU-6050
        filterData();
        // looking to statbilize the X-axis (side to side) of the table
        angleX = kalmanloop(accel_x, gyro_x, calculateXAngle, "X");

        // gettting the distance of the table from the ultrasonic sensor
        getFilteredDistance();
        // getRawDistance();
        nowDistanceCm = distanceCm; // store the current distance of the table from the ultrasonic sensor

        // Leveling check
        // here is a copy from above of the variables to indicate if the table is level
        // double x_angle_ok = 1.0;      // should equal to 1 degrees
        // double x_angle_perfect = 0.5; // should equal to 0.25 degrees
        // float distance_tolerance = 1.0;         // originally 0.5 cm tolerance
        // float refined_distance_tolerance = 0.75; // originally 0.2 cm tolerance
        // targetDistance = distanceCm; // from stopMovement(); // set the target distance to the current distance

        // if the table is at the right angle and right distance
        if ((angleX >= -x_angle_ok && angleX <= x_angle_ok) && (nowDistanceCm >= targetDistance - distance_tolerance && nowDistanceCm <= targetDistance + distance_tolerance))
        {
            // RT11
            // digitalWrite(DOWN_PIN, DEACTIVATE);
            // digitalWrite(UP_PIN, DEACTIVATE);
            // 
            
            // ALL Stop the movement
            digitalWrite(relay1_LA1LHP_Pin, HIGH);
            digitalWrite(relay2_LA1LHM_Pin, HIGH);
            digitalWrite(relay3_LA2RHP_Pin, HIGH);
            digitalWrite(relay4_LA2RHM_Pin, HIGH);
            
            // template for boolChecker function is:
            // bool rt11Working
            // int pinstate1
            // int pinState2
            isLeftActuator = "STOP";
            isRightActuator = "STOP";
            isStop = true;
            isMovingUp = false;
            isMovingDown = false;
            isStationary = true;
            isTableLevel = true;
            // bool isTableMax
            // bool isBottomOut

            displayPosition(); // displays the position of the table // from LEDeffects.h

            Serial.println("Table is level now");       // used for testing purposes only
            Serial.println("levelTable function ends"); // used for testing purposes only
            FastLED.clear(); // clear the LEDs

            return; // will exit the function
        }
        else
        {
            // if the table is not at the right angle and right distance
            isDistanceOK = false;
            isAngleOK = false;

            Serial.println("in !tableLevel while, nowDistanceCm:\t" + String(nowDistanceCm));   // used for testing purposes only
            Serial.println("in !tableLevel while, targetDistance:\t" + String(targetDistance)); // used for testing purposes only
            Serial.println("in !tableLevel while, angleX:\t" + (String(angleX)));               // used for testing purposes only

            while (!isDistanceOK && !isAngleOK)
            {
                if (!isDistanceOK)
                {
                    // if the table has changed distance while leveling
                    // if (nowDistanceCm > targetDistance + distance_tolerance || nowDistanceCm < targetDistance - distance_tolerance)
                    // {

                    while (nowDistanceCm > targetDistance + distance_tolerance || nowDistanceCm < targetDistance - distance_tolerance)
                    {
                        getFilteredDistance();
                        nowDistanceCm = distanceCm;
                        Serial.println("!isDistanceOK while, -nowDistanceCm:\t" + String(nowDistanceCm)); // used for testing purposes only

                        // if table is higher than the target distance
                        if (nowDistanceCm > targetDistance + distance_tolerance)
                        {
                            Serial.println("nowDistanceCM > " + String(targetDistance + distance_tolerance)); // used for testing purposes only

                            //  if the previous command was to stop the table from moving
                            if (isStop)
                            {
                                break; // will exit the while loop
                            }

                            // template for boolChecker function is:
                            // bool rt11Working
                            // int pinstate1
                            // int pinState2
                            isLeftActuator = "DOWN";
                            isRightActuator = "DOWN";
                            isStop = false;
                            isMovingUp = false;
                            isMovingDown = true;
                            isStationary = false;
                            isTableLevel = false;
                            // bool isTableMax
                            // bool isBottomOut

                            // Adjust table to correct distance
                            moveActuators(false, false);
                            // LA1LH -> if false then, move down
                            // digitalWrite(relay1_LA1LHP_Pin, HIGH); // redundant protection
                            // digitalWrite(relay2_LA1LHM_Pin, LOW);
                            // LA2RH -> if false then, move down
                            // digitalWrite(relay3_LA2RHP_Pin, HIGH); // redundant protection
                            // digitalWrite(relay4_LA2RHM_Pin, LOW);
                            
                            displayPosition(); // displays the position of the table // from LEDeffects.h

                            // delay(5); // delay for 15 milli second to let the table adjust
                        }
                        else if (nowDistanceCm < targetDistance - distance_tolerance)
                        {
                            Serial.println("nowDistanceCM < " + String(targetDistance - distance_tolerance)); // used for testing purposes only

                            // if the previous command was to stop the table from moving
                            if (isStop)
                            {
                                break; // will exit the while loop
                            }

                            // template for boolChecker function is:
                            // bool rt11Working
                            // int pinstate1
                            // int pinState2
                            isLeftActuator = "UP";
                            isRightActuator = "UP";
                            isStop = false;
                            isMovingUp = true;
                            isMovingDown = false;
                            isStationary = false;
                            isTableLevel = false;
                            // bool isTableMax
                            // bool isBottomOut

                            // Adjust table to correct distance
                            moveActuators(true, true);
                            // LA1LH -> if true then, move up
                            // digitalWrite(relay1_LA1LHP_Pin, LOW);
                            // digitalWrite(relay2_LA1LHM_Pin, HIGH); // redundant protection
                            // LA2RH -> if true then, move up
                            // digitalWrite(relay3_LA2RHP_Pin, LOW);
                            // digitalWrite(relay4_LA2RHM_Pin, HIGH); // redundant protection

                            displayPosition(); // displays the position of the table // from LEDeffects.h

                            // delay(5); // delay for 15 milli second to let the table adjust
                        }
                        else
                        {
                            isDistanceOK = true;
                            Serial.println("Distance is OK"); // used for testing purposes only
                        }; // end if nowDistanceCm > targetDistance + distance_tolerance OR nowDistanceCm < targetDistance - distance_tolerance

                        Serial.print("!isDistanceOK nowDistanceCm =\t");     // used for testing purposes only
                        Serial.println(nowDistanceCm);                       // used for testing purposes only
                        Serial.print("is now within the range of:\t");       // used for testing purposes only
                        Serial.print(targetDistance - distance_tolerance);   // used for testing purposes only
                        Serial.print("\tand\t");                             // used for testing purposes only
                        Serial.println(targetDistance + distance_tolerance); // used for testing purposes only

                        delay(50); // delay for 100 milli second to let the table adjust

                        Serial.println("distance while loop ends"); // used for testing purposes only

                        isDistanceOK = true;

                        // return; // will exit the function
                        break; // will exit the while loop
                    }; // end while nowDistanceCm <> targetDistance +/- distance_tolerance
                    // } // end if nowDistanceCm <> targetDistance

                }; // end if !isDistanceOK

                if (!isAngleOK)
                {
                    // if the angle is not above 3 degrees, then we can adjust the table to level
                    if (angleX > x_angle_ok || angleX < -x_angle_ok)
                    {
                        // this if statement is to stop the table from moving if the table goes beyond 3 degrees
                        if (angleX >= x_angle_max || angleX <= -x_angle_max)
                        {
                            Serial.println("Full STOP: Table angle is beyond tolerance!!"); // used for testing purposes only

                            // RT11
                            // digitalWrite(DOWN_PIN, DEACTIVATE);
                            // digitalWrite(UP_PIN, DEACTIVATE);
                            // 

                            // ALL Stop the movement
                            digitalWrite(relay1_LA1LHP_Pin, HIGH);
                            digitalWrite(relay2_LA1LHM_Pin, HIGH);
                            digitalWrite(relay3_LA2RHP_Pin, HIGH);
                            digitalWrite(relay4_LA2RHM_Pin, HIGH);

                            // template for boolChecker function is:
                            // bool rt11Working
                            // int pinstate1
                            // int pinState2
                            isLeftActuator = "STOP";
                            isRightActuator = "STOP";
                            isStop = true;
                            isMovingUp = false;
                            isMovingDown = false;
                            isStationary = true;
                            isTableLevel = false;
                            // bool isTableMax
                            // bool isBottomOut

                            // displayPosition(); // displays the position of the table // from LEDeffects.h
                            redAlert2(); // used to warn the user that the table is not level // from LEDeffects.h

                            delay(75); // delay for 30 milli second to let the table adjust

                            // see if we can get the table to level itself
                            while (angleX >= x_angle_max || angleX <= -x_angle_max)
                            {
                                // get the filtered data from the MPU-6050
                                filterData();
                                // looking to stabilize the X-axis (side to side) of the table
                                angleX = kalmanloop(accel_x, gyro_x, calculateXAngle, "X");
                                Serial.println("Over angleX is now:\t" + String(angleX)); // used for testing purposes only

                                // if the table is too high on the the left side
                                if (angleX >= x_angle_ok)
                                {

                                    // template for boolChecker function is:
                                    // bool rt11Working
                                    // int pinstate1
                                    // int pinState2
                                    isLeftActuator = "STOP";
                                    isRightActuator = "UP";
                                    isStop = false;
                                    // isMovingUp = true;
                                    isMovingDown = false;
                                    isStationary = false;
                                    isTableLevel = false;
                                    // bool isTableMax
                                    // bool isBottomOut

                                    // Adjust table to correct angle
                                    moveActuators(false, true);
                                    // LA2RH -> if false then, move down
                                    // digitalWrite(relay3_LA2RHP_Pin, HIGH); // redundant protection
                                    // digitalWrite(relay4_LA2RHM_Pin, LOW);

                                    displayPosition(); // displays the position of the table // from LEDeffects.h

                                    delay(15); // delay for 20 milli second to let the table adjust
                                }
                                else if (angleX <= -x_angle_ok)
                                {
                                    // if the table is too high on the right side

                                    // template for boolChecker function is:
                                    // bool rt11Working
                                    // int pinstate1
                                    // int pinState2
                                    isLeftActuator = "UP";
                                    isRightActuator = "STOP";
                                    isStop = false;
                                    isMovingUp = false;
                                    // isMovingDown = true;
                                    isStationary = false;
                                    isTableLevel = false;
                                    // bool isTableMax
                                    // bool isBottomOut

                                    // Adjust table to correct angle
                                    moveActuators(true, false);
                                    // LA1LH -> if false then, move down
                                    // digitalWrite(relay1_LA1LHP_Pin, HIGH); // redundant protection
                                    // digitalWrite(relay2_LA1LHM_Pin, LOW);

                                    displayPosition(); // displays the position of the table // from LEDeffects.h

                                    delay(15); // delay for 20 milli second to let the table adjust
                                }; // end if angleX >= x_angle_ok

                                delay(100);                                // delay for 100 milli second to let the table adjust
                                Serial.println("Over angleX while ends"); // used for testing purposes only

                                // return; // will exit the function
                                break; // will exit the while loop
                            }; // end while angleX <> x_angle_max
                        }; // end if angleX <> x_angle_max

                        while (angleX > x_angle_ok || angleX < -x_angle_ok)
                        {
                            // get the filtered data from the MPU-6050
                            filterData();
                            // looking to stabilize the X-axis (side to side) of the table
                            angleX = kalmanloop(accel_x, gyro_x, calculateXAngle, "X");
                            Serial.println("refined angleX is now:\t" + String(angleX)); // used for testing purposes only

                            // if the table is slightly high on the the left side
                            if (angleX >= x_angle_perfect)
                            {

                                // template for boolChecker function is:
                                // bool rt11Working
                                // int pinstate1
                                // int pinState2
                                isLeftActuator = "STOP";
                                isRightActuator = "UP";
                                isStop = false;
                                // isMovingUp = true;
                                isMovingDown = false;
                                isStationary = false;
                                isTableLevel = false;
                                // bool isTableMax
                                // bool isBottomOut

                                // Adjust table to correct angle
                                moveActuators(false, true);
                                // LA2RH -> if false then, move down
                                // digitalWrite(relay3_LA2RHP_Pin, HIGH); // redundant protection
                                // digitalWrite(relay4_LA2RHM_Pin, LOW);

                                displayPosition(); // displays the position of the table // from LEDeffects.h

                                delay(5); // delay for 20 milli second to let the table adjust
                            }
                            else if (angleX <= -x_angle_perfect)
                            {
                                // if the table is slightly high on the right side

                                // template for boolChecker function is:
                                // bool rt11Working
                                // int pinstate1
                                // int pinState2
                                isLeftActuator = "UP";
                                isRightActuator = "STOP";
                                isStop = false;
                                isMovingUp = false;
                                // isMovingDown = true;
                                isStationary = false;
                                isTableLevel = false;
                                // bool isTableMax
                                // bool isBottomOut

                                // Adjust table to correct angle
                                moveActuators(true, false);
                                // LA1LH -> if false then, move down
                                // digitalWrite(relay1_LA1LHP_Pin, HIGH); // redundant protection
                                // digitalWrite(relay2_LA1LHM_Pin, LOW);
                                // moveActuators(bool leftUp, bool rightUp)

                                displayPosition(); // displays the position of the table // from LEDeffects.h

                                delay(5); // delay for 20 milli second to let the table adjust
                            }
                            else
                            {
                                isAngleOK = true;
                                Serial.println("angleX is now within tolerance"); // used for testing purposes only
                                delay(100);                                        // delay for 100 milli second to let the table adjust
                                Serial.println("refined angleX while ends");      // used for testing purposes only

                                isAngleOK = true;

                                // return; // will exit the function
                                break; // will exit the while loop
                            }; // end if angleX >= x_angle_perfect
                        }; // end while angleX <> x_angle_ok

                        if (isStop || (isDistanceOK && isAngleOK))
                        {
                            // Stop actuators or perform necessary final adjustments

                            Serial.println("Leveling complete or stopped");
                        }
                    } // end if angleX <> x_angle_ok
                }; // end if !isAngleOK
            }; // end while !isDistanceOK && !isAngleOK
        }; // end else (angleX >= -x_angle_ok && angleX <= x_angle_ok) && (nowDistanceCm >= targetDistance - distance_tolerance && nowDistanceCm <= targetDistance + distance_tolerance)
        // ================

        // ================
        // version with the selfBalancing.h library
        // #include "selfBalancing.h"
        // balancingSetup();
        // balancingLoop();
        // ================

    }; // end while !isTableLevel

    Serial.println("Table is level now. end levelTable first while loop"); // used for testing purposes only
}; // end levelTable

void checkTableAfterStop(void)
{
    if (isTableLevel == false)
    {
        levelTable();
    }
    else
    {
        isStop = false; // need to reset the stop flag
        isTableLevel = true;
        Serial.println("Table is level!");
        FastLED.clear(); // clears the position of the table // from LEDeffects.h
    }
}; // end checkTableAfterStop

void stopMovement(void)
{
    Serial.println("Stop Movement function starts"); // used for testing purposes only

    // RT11 - Stop the movement
    // digitalWrite(DOWN_PIN, DEACTIVATE);
    // digitalWrite(UP_PIN, DEACTIVATE);
    // currentHeight = requestedHeight;
    // currentOperation = NOOP;
    // 

    // ALL STOP
    digitalWrite(relay1_LA1LHP_Pin, HIGH);
    digitalWrite(relay2_LA1LHM_Pin, HIGH);
    digitalWrite(relay3_LA2RHP_Pin, HIGH);
    digitalWrite(relay4_LA2RHM_Pin, HIGH);

    // template for boolChecker function is:
    // bool rt11Working
    // int pinstate1
    // int pinState2
    isLeftActuator = "STOP";
    isRightActuator = "STOP";
    isStop = true;
    isMovingUp = false;
    isMovingDown = false;
    isStationary = true;
    isTableLevel = false; // We don't know what level the table is at after stopping movement
    // bool isTableMax
    // bool isBottomOut

    // get the distance of the table from the ultrasonic sensor
    getFilteredDistance();
    // getRawDistance();

    targetDistance = distanceCm; // set the target distance to the current distance

    // Serial.println("MaxDistance tolerance level is:\t" + String(maxDistanceCm + dist_tolerance_level) + "\tto:\t" + String(maxDistanceCm - dist_tolerance_level)); // used for testing purposes only
    // which should be 100.00 - 0.5 = 99.5 cm and 100.00 + 0.5 = 100.5 cm

    if (distanceCm >= maxDistanceCm - dist_tolerance_level && distanceCm <= maxDistanceCm + dist_tolerance_level)
    {
        Serial.println("Table is at the Max Height position tolerance level.");
        isStop = true;
        isTableMax = true;
        targetDistance = distanceCm;
    }

    if (distanceCm >= maxDistanceCm + dist_tolerance_level)
    {
        // just to ensure the target distance is never higher than the max distance
        Serial.println("targetDistance is above the MAX distance position!!"); // used for testing purposes only
        targetDistance = maxDistanceCm - distance_tolerance;                   // which should be 100.00 - 0.5 = 99.5 cm
        isStop = true;
        isTableMax = true;
    }

    // in case there is a distance discrepancy while measuring the distance
    if (distanceCm < bottomOutDistanceCm)
    {
        // just to ensure the target distance is never lower than the bottom out position
        Serial.println("targetDistance is below the bottom out position."); // used for testing purposes only
        targetDistance = bottomOutDistanceCm;
        isStop = true;
        isBottomOut = true;
    }

    Serial.println("targetDistance is now:\t" + String(targetDistance)); // used for testing purposes only

    // Stop the getAccelerationDataTask
    // getAccelerationDataTaskRunning = false;

    // save info to file
    logSensorData(5); // this is to save the STOP MOVEMMENT DATA into logging file on the SD Card

    // to make sure that the table gets leveled or to ensure it is leveled after stopping
    checkTableAfterStop();

    // delay changed to every 50ms
    // currentMillis = millis();
    // if (currentMillis - previousMillis >= timePeriod050)
    // {
    //     // delay(2); // Needed so we don't get repeated measures
    //     previousMillis = currentMillis;
    // }

    Serial.println("Stop Movement function ends"); // used for testing purposes only
    boolChecker();                                 // used for testing purposes only
}; // end stopMovement

void moveDown(void)
{
    Serial.println("Moving Down function starts"); // used for testing purposes only
    float lastDistancecm = 999.0;                  // set to a high number to ensure the table is moving down

    // RT11 - Move the table down
    // digitalWrite(DOWN_PIN, ACTIVATE);
    // 

    // get the distance of the table from the ultrasonic sensor
    getFilteredDistance(); // get the distance of the table from the ultrasonic sensor
    // getRawDistance();

    // if the table is moving down, the current distance should not be more then the last distance measured.
    if (distanceCm < lastDistancecm)
    {
        lastDistancecm = distanceCm;
    }
    else
    {
        distanceCm = lastDistancecm;
    };

    if (distanceCm > baseDistanceCm)
    {
        moveActuators(false, false);
        // Activate the relay controlling actuator #2 positive
        // digitalWrite(relay1_LA1LHP_Pin, HIGH); // redundant protection
        // digitalWrite(relay2_LA1LHM_Pin, LOW);
        // Activate the relay controlling actuator #2 negative
        // digitalWrite(relay3_LA2RHP_Pin, HIGH); // redundant protection
        // digitalWrite(relay4_LA2RHM_Pin, LOW);

        displayPosition(); // displays the position of the table // from LEDeffects.h

        // template for boolChecker function is:
        // bool rt11Working
        // int pinstate1
        // int pinState2
        isLeftActuator = "DOWN";
        isRightActuator = "DOWN";
        isStop = false;
        isMovingUp = false;
        isMovingDown = true;
        isStationary = false;
        isTableLevel = false;
        isTableMax = false;
        isBottomOut = false;

        // Start the getAccelerationDataTask
        // getAccelerationDataTaskRunning = true;
        // xTaskCreate(getAccelerationDataTask, "GetAccelerationData", 10000, NULL, 1, NULL);
        // getAccelerationData(); // get the acceleration data in order to data log
        logSensorData(4); // this is to get the ACCELERATION DATA to the SD Card logging file

        // delay changed to every 50ms
        // currentMillis = millis();
        // if (currentMillis - previousMillis >= timePeriod050)
        // {
        //     // delay(2); // Needed so we don't get repeated measures
        //     previousMillis = currentMillis;
        // }
    }
    else if (distanceCm <= baseDistanceCm)
    {
        Serial.println("Table is now at the Base Height position.");

        // template for boolChecker function is:
        // bool rt11Working
        // int pinstate1
        // int pinState2
        isLeftActuator = "STOP";
        isRightActuator = "STOP";
        isStop = true;
        isMovingUp = false;
        isMovingDown = false;
        isStationary = true;
        isTableLevel = true;
        isTableMax = false;
        isBottomOut = true;

        stopMovement();
    };

    boolChecker(); // used for testing purposes only

    Serial.println("Moving Down function ends"); // used for testing purposes only
}; // end moveDown

// version 2
void moveUp(void)
{
    Serial.println("Moving Up function starts"); // used for testing purposes only
    float lastDistancecm = 0.0;                  // set to a low number to ensure the table is moving up
    isMovingUp = true;

    // RT11 - Move the table up
    // digitalWrite(UP_PIN, ACTIVATE);
    // 

    // if (isTableMax)
    // {
    //     Serial.println("isTableMax is true inside moveUp function");
    // }
    // else
    // {
    //     Serial.println("isTableMax is false inside moveUp function");
    // }

    // get the distance of the table from the ultrasonic sensor
    getFilteredDistance();
    // getRawDistance();
    Serial.println("distanceCm: " + String(distanceCm)); // used for testing purposes only

    // since we are going up, the current distance should not be less then the last distance measured.
    if (distanceCm < lastDistancecm)
    {
        distanceCm = lastDistancecm;
    }
    else
    {
        lastDistancecm = distanceCm;
    }

    Serial.println("maxDistanceCm: " + String(maxDistanceCm)); // used for testing purposes only

    if (isTableMax == true)
    {
        Serial.print("Table is already at the Max Height position.");

        // template for boolChecker function is:
        // bool rt11Working
        // int pinstate1
        // int pinState2
        isLeftActuator = "STOP";
        isRightActuator = "STOP";
        isStop = true;
        isMovingUp = false;
        isMovingDown = false;
        isStationary = true;
        isTableLevel = false;
        isTableMax = true;
        isBottomOut = false;

        stopMovement();
        return; // Exit the function if the table is already at the max height
    }

    if (distanceCm < maxDistanceCm)
    {
        moveActuators(true, true);
        // LA1LH -> if true then, move up
        // digitalWrite(relay1_LA1LHP_Pin, LOW);
        // digitalWrite(relay2_LA1LHM_Pin, HIGH); // redundant protection
        // LA2RH -> if true then, move up
        // digitalWrite(relay3_LA2RHP_Pin, LOW);
        // digitalWrite(relay4_LA2RHM_Pin, HIGH); // redundant protection

        displayPosition(); // displays the position of the table // from LEDeffects.h

        // template for boolChecker function is:
        // bool rt11Working
        // int pinstate1
        // int pinState2
        isLeftActuator = "UP";
        isRightActuator = "UP";
        isStop = false;
        isMovingUp = true;
        isMovingDown = false;
        isStationary = false;
        isTableLevel = false;
        isTableMax = false;
        isBottomOut = false;

        // Start the getAccelerationDataTask
        // getAccelerationDataTaskRunning = true;
        // xTaskCreate(getAccelerationDataTask, "GetAccelerationData", 10000, NULL, 1, NULL);
        // getAccelerationData(); // get the acceleration data
        logSensorData(4); // this is to get the ACCELERATION DATA to the SD Card logging file

        // delay changed to every 50ms
        // currentMillis = millis();
        // if (currentMillis - previousMillis >= timePeriod050)
        // {
        //     // delay(2); // Needed so we don't get repeated measures
        //     previousMillis = currentMillis;
        // }
    }
    else if (distanceCm >= maxDistanceCm - distance_tolerance || distanceCm >= maxDistanceCm + distance_tolerance)
    {
        // if the table is at the max height position and within tolerance

        // template for boolChecker function is:
        // bool rt11Working
        // int pinstate1
        // int pinState2
        isLeftActuator = "STOP";
        isRightActuator = "STOP";
        isStop = true;
        isMovingUp = false;
        isMovingDown = false;
        isStationary = true;
        isTableLevel = false;
        isTableMax = true;
        isBottomOut = false;

        logSensorData(4); // this is to get the ACCELERATION DATA to the SD Card logging file

        Serial.println("Table is now at the Max Height position."); // used for testing purposes only
        stopMovement();
    }
    else if (distanceCm > maxDistanceCm)
    {
        // if the table is above the max height position

        // template for boolChecker function is:
        // bool rt11Working
        // int pinstate1
        // int pinState2
        isLeftActuator = "STOP";
        isRightActuator = "STOP";
        isStop = true;
        isMovingUp = false;
        isMovingDown = false;
        isStationary = true;
        isTableLevel = false;
        isTableMax = true;
        isBottomOut = false;

        logSensorData(4); // this is to get the ACCELERATION DATA to the SD Card logging file

        Serial.println("Table is ABOVE the Max Height position."); // used for testing purposes only
        stopMovement();
    };

    boolChecker(); // used for testing purposes only

    Serial.println("Moving Up function ends"); // used for testing purposes only
}; // end moveUp

// version 1
// void moveUp()
// {
//     Serial.println("Moving Table Up function starts"); // used for testing purposes only
//     if (isStop)
//     {
//         Serial.println("isStop is true inside moveUp function");
//     }
//     else
//     {
//         Serial.println("isStop is false inside moveUp function");
//     }
//
//     if (isTableMax)
//     {
//         Serial.println("isTableMax is true inside moveUp function");
//     }
//     else
//     {
//         Serial.println("isTableMax is false inside moveUp function");
//     }
//
//     if (isTableMax == true)
//     {
//         Serial.print("Table is already at the Max Height position.");
//         stopMovement();
//         return;
//     }
//
//     while (!isStop && !isTableMax && distanceCm < maxDistanceCm - distance_tolerance)
//     {
//         // Check if a new command has been entered
//         if (Serial.available())
//         {
//             Serial.println("New command received.");
//             return; // Exit the function if a new command is received
//         }
//
//         moveActuators(true, true);
//         // Activate the relay controlling actuator #1 positive
//         // digitalWrite(relay1_LA1LHP_Pin, LOW);
//         // digitalWrite(relay2_LA1LHM_Pin, HIGH); // redundant protection
//         // Activate the relay controlling actuator #1 negative
//         // digitalWrite(relay3_LA2RHP_Pin, LOW);
//         // digitalWrite(relay4_LA2RHM_Pin, HIGH); // redundant protection
//
//         // isStop = false;
//         isMovingDown = false;
//         isMovingUp = true;
//         isStationary = false;
//         isTableLevel = false;
//         // isTableMax = false;
//
//         // Start the getAccelerationDataTask
//         // getAccelerationDataTaskRunning = true;
//         // xTaskCreate(getAccelerationDataTask, "GetAccelerationData", 10000, NULL, 1, NULL);
//         getAccelerationData(); // get the acceleration data
//
//         // delay changed to every 50ms
//         // currentMillis = millis();
//         // if (currentMillis - previousMillis >= timePeriod050)
//         // {
//         //     // delay(2); // Needed so we don't get repeated measures
//         //     previousMillis = currentMillis;
//         // }
//
//         // get the distance of the table from the ultrasonic sensor
//         getFilteredDistance();
//         // getRawDistance();
//         Serial.println("distanceCm: " + String(distanceCm));       // used for testing purposes only
//         Serial.println("maxDistanceCm: " + String(maxDistanceCm)); // used for testing purposes only
//     }
//
//     isMovingUp = false;
//     isTableMax = true;
//     Serial.println("Table is now at the Max Height position."); // used for testing purposes only
//     Serial.println("Moving Table Up function ends");            // used for testing purposes only
//     stopMovement();
// }; // end moveUp

void moveToBottomOutPosition(void)
{
    // float tolerance = 0.5; // Set this to a suitable value

    // start function with the following bool values
    // template for boolChecker function is:
    // bool rt11Working
    // int pinstate1
    // int pinState2
    // String isLeftActuator
    // String isRightActuator
    isStop = false;
    isMovingUp = false;
    isMovingDown = true;
    isStationary = false;
    isTableLevel = false; // We don't know what level the table is at after stopping movement
    isTableMax = false;
    isBottomOut = false;

    // if there is a config file, then use the following code
    if (SD.exists("/config.csv"))
    {
        // ================
        // have to load baseDistance from the config file
        loadDistances(); // this function is used to load the distances from the config file
        // ================

        while (true)
        {
            // get the distance of the table from the ultrasonic sensor
            getFilteredDistance();
            // getRawDistance();

            // used for testing purposes only
            // Serial.println("Moving Table..");
            Serial.print("MoveToBottom_baseDistanceCm:\t ");
            Serial.println(bottomOutDistanceCm);
            Serial.print("MoveToBottom_Current_Distance:\t");
            Serial.println(distanceCm);
            Serial.print("MoveToBottom_difference:\t");
            Serial.println(distanceCm - baseDistanceCm);

            // Check if isStop is true
            if (isStop)
            {
                // Stop the actuators
                digitalWrite(relay1_LA1LHP_Pin, HIGH);
                digitalWrite(relay2_LA1LHM_Pin, HIGH);
                digitalWrite(relay3_LA2RHP_Pin, HIGH);
                digitalWrite(relay4_LA2RHM_Pin, HIGH);

                // template for boolChecker function is:
                // bool rt11Working
                // int pinstate1
                // int pinState2
                isLeftActuator = "STOP";
                isRightActuator = "STOP";
                isStop = true;
                isMovingUp = false;
                isMovingDown = false;
                isStationary = true;
                isTableLevel = false; // We don't know what level the table is at after stopping movement
                isTableMax = false;
                isBottomOut = true;

                return; // Exit the function
                // break; // Exit the loop
            }

            if (abs(distanceCm - bottomOutDistanceCm <= dist_tolerance_level))
            {
                // ALL STOP
                digitalWrite(relay1_LA1LHP_Pin, HIGH);
                digitalWrite(relay2_LA1LHM_Pin, HIGH);
                digitalWrite(relay3_LA2RHP_Pin, HIGH);
                digitalWrite(relay4_LA2RHM_Pin, HIGH);

                // Serial.print("From save config. Table has stopped now. It is now at the bottom out position."); // used for testing purposes only

                // template for boolChecker function is:
                // bool rt11Working
                // int pinstate1
                // int pinState2
                isLeftActuator = "STOP";
                isRightActuator = "STOP";
                isStop = true;
                isMovingUp = false;
                isMovingDown = false;
                isStationary = true;
                isTableLevel = false; // We don't know what level the table is at after stopping movement
                isTableMax = false;
                isBottomOut = false;

                // return; // Exit the function
                break; // Break out of the loop
            }
            else
            {
                // Serial.println("Table is moving down..."); // used for testing purposes only

                moveActuators(false, false);
                // Activate the relay controlling actuator #1 positive
                // digitalWrite(relay1_LA1LHP_Pin, HIGH); // redundant protection
                // digitalWrite(relay2_LA1LHM_Pin, LOW);
                // Activate the relay controlling actuator #2 negative
                // digitalWrite(relay3_LA2RHP_Pin, HIGH); // redundant protection
                // digitalWrite(relay4_LA2RHM_Pin, LOW);

                displayPosition(); // displays the position of the table // from LEDeffects.h

                // template for boolChecker function is:
                // bool rt11Working
                // int pinstate1
                // int pinState2
                isLeftActuator = "DOWN";
                isRightActuator = "DOWN";
                isStop = false;
                isMovingUp = false;
                isMovingDown = true;
                isStationary = false;
                isTableLevel = false; // We don't know what level the table is at after stopping movement
                isTableMax = false;
                isBottomOut = false;

                // delay(500); // wait for 1/2 second
                // delay changed to every 10ms
                // currentMillis = millis();
                // if (currentMillis - previousMillis >= timePeriod010)
                // {
                //     // delay(2); // Needed so we don't get repeated measures
                //     previousMillis = currentMillis;
                // }
            }
        }
    }
    else
    {
        // if the config file does not exist, then use the following code

        // float previousDistanceCm = 0;
        // or use the bottomOutDistanceCm

        while (true)
        {
            // get the distance of the table from the ultrasonic sensor
            getFilteredDistance();
            // getRawDistance();

            // used for testing purposes only
            // Serial.println("Moving Table..");
            Serial.print("MoveToBottom_bottomOutDistanceCm:\t ");
            Serial.println(bottomOutDistanceCm);
            Serial.print("MoveToBottom_Current_Distance:\t");
            Serial.println(distanceCm);
            Serial.print("MoveToBottom_with_distance_Tolerance:\t");
            Serial.println(distanceCm - bottomOutDistanceCm);

            // Check if isStop is true
            if (isStop)
            {
                // Stop the actuators
                digitalWrite(relay1_LA1LHP_Pin, HIGH);
                digitalWrite(relay2_LA1LHM_Pin, HIGH);
                digitalWrite(relay3_LA2RHP_Pin, HIGH);
                digitalWrite(relay4_LA2RHM_Pin, HIGH);

                // template for boolChecker function is:
                // bool rt11Working
                // int pinstate1
                // int pinState2
                isLeftActuator = "STOP";
                isRightActuator = "STOP";
                isStop = true;
                isMovingUp = false;
                isMovingDown = false;
                isStationary = true;
                isTableLevel = false;
                isTableMax = false;
                isBottomOut = true;

                return; // Exit the function
                // break; // Exit the loop
            }

            // if the table is lower than the bottom out position
            if (abs(distanceCm - bottomOutDistanceCm) <= dist_tolerance_level)
            {
                // ALL STOP
                digitalWrite(relay1_LA1LHP_Pin, HIGH);
                digitalWrite(relay2_LA1LHM_Pin, HIGH);
                digitalWrite(relay3_LA2RHP_Pin, HIGH);
                digitalWrite(relay4_LA2RHM_Pin, HIGH);

                // Serial.print("From uncalibrated. Table is stopping now. It is now at the bottom out position."); // used for testing purposes only

                // template for boolChecker function is:
                // bool rt11Working
                // int pinstate1
                // int pinState2
                isLeftActuator = "STOP";
                isRightActuator = "STOP";
                isStop = true;
                isMovingUp = false;
                isMovingDown = false;
                isStationary = true;
                isTableLevel = false;
                isTableMax = false;
                isBottomOut = true;

                // return; // Exits the function
                break; // Exits the loop
            }
            else
            {
                // if the table is distance is greater than the bottom out position

                // Serial.println("Table is moving down..."); // used for testing purposes only

                moveActuators(false, false);
                // Activate the relay controlling actuator #2 positive
                // digitalWrite(relay1_LA1LHP_Pin, HIGH); // redundant protection
                // digitalWrite(relay2_LA1LHM_Pin, LOW);
                // Activate the relay controlling actuator #2 negative
                // digitalWrite(relay3_LA2RHP_Pin, HIGH); // redundant protection
                // digitalWrite(relay4_LA2RHM_Pin, LOW);

                displayPosition(); // displays the position of the table // from LEDeffects.h

                // template for boolChecker function is:
                // bool rt11Working
                // int pinstate1
                // int pinState2
                isLeftActuator = "DOWN";
                isRightActuator = "DOWN";
                isStop = false;
                isMovingUp = false;
                isMovingDown = true;
                isStationary = false;
                isTableLevel = false;
                isTableMax = false;
                isBottomOut = false;

                // previousDistanceCm = distanceCm; // if we use a previousDistanceCm variable

                // delay(500); // wait for 1/2 second
                // changed to every 10ms
                // currentMillis = millis();
                // if (currentMillis - previousMillis >= timePeriod010)
                // {
                //     // delay(2); // Needed so we don't get repeated measures
                //     previousMillis = currentMillis;
                // }
            }
        }
    }

    // template for boolChecker function is:
    // bool rt11Working
    // int pinstate1
    // int pinState2
    isLeftActuator = "DOWN";
    isRightActuator = "DOWN";
    isStop = true;
    isMovingUp = false;
    isMovingDown = false;
    isStationary = true;
    isTableLevel = false;
    isTableMax = false;
    isBottomOut = true;

    Serial.println("moveToBottomOutPositon function ends"); // used for testing purposes only
}; // end moveToBottomOutPosition

// 
// THIS PART BELOW IS FOR THE RT-11 CONTROLLER
// 
// ================
// From: https://www.devmire.com/2021/10/03/reverse-engineering-a-standing-desk-for-fun-and-profit/
// #include <SoftwareSerial.h>
// 
// #define MAX_MILLIS_TO_WAIT 1000 // or whatever
// #define DOWN_PIN 12 // original code. this was to tell the controller to go down
// #define UP_PIN 13 // original code. this was to tell the controller to go up
// #define UPOP 1
// #define DNOP 2
// #define NOOP 0
// #define ACTIVATE LOW
// #define DEACTIVATE HIGH
// #define DESKMOVEDDURINGREAD 9 // refers to the inches the desk moves during the time it takes to read the height
// 
// SoftwareSerial(uint8_t receivePin, uint8_t transmitPin, bool inverse_logic = false);
// SoftwareSerial deskSerial(5, 4); // SoftwareSerial deskSerial(Rx GPIO (SCL), Tx GPIO (SDA)); // original code
// 
// #ifndef STASSID
// #define STASSID "Your SSID"
// #define STAPSK  "Your pass"
// #endif
// 
// const int led = LED_BUILTIN;
// int currentOperation = NOOP;
// unsigned long starttime;
// const char* ssid = STASSID;
// const char* password = STAPSK;
// unsigned long value;
// int RFin_bytes[4]; // The message 4 bytes long
// int currentHeight = 0;
// int requestedHeight = 0;
// int heightMultiplier = 1;
// 
// Initialize the webserver
// ESP8266WebServer server(80); // original code
// 
// void handleRoot()
// {
//     digitalWrite(led, 1);
//     server.send(200, "text/plain", "hello from esp32!\r\n");
//     digitalWrite(led, 0);
// }; // end of handleRoot
// 
/* This method decodes the serial data*/
int decodeSerial()
{
    while (deskSerial.available() < 4)
    {
        // hang in this loop until we get 4 bytes of data
    }
    if (deskSerial.available() < 4)
    {
        // the data didn't come in - handle that problem here
        Serial.println("ERROR - Didn't get 4 bytes of data!");
    }
    else
    {
        for (int n = 0; n < 4; n++)
        {
            RFin_bytes[n] = deskSerial.read(); // Then: Get them.
            Serial.println(RFin_bytes[n]);
        }
    }

    Serial.println("==========");

    uint16_t myInt1 = (RFin_bytes[2] << 8) + RFin_bytes[3]; // Convert Big Endian to Unit16
    Serial.println(myInt1);
    if (myInt1 > 1259 || myInt1 < 600)
    {
        decodeSerial(); // Wrong value, try again
    }
    return myInt1;
}; // end of decodeSerial
// 
// the function below is already included in this library (located above)
/* This method stops moving and sets the requested height to current height */
// void stopMoving(void)
// {
//     digitalWrite(DOWN_PIN, DEACTIVATE);
//     digitalWrite(UP_PIN, DEACTIVATE);
//     currentHeight = requestedHeight;
//     currentOperation = NOOP;
// }; // end of stopMoving
// 
void setHeight(int height)
{
    operation = 0;
    requestedHeight = height;
    if (currentHeight > requestedHeight)
    {
        operation = DNOP;
    };
    if (currentHeight < requestedHeight)
    {
        operation = UPOP;
    };

    if (currentHeight == requestedHeight)
    {
        operation = NOOP;
    };
    switch (operation)
    {
    case UPOP:
        moveUp();
        digitalWrite(UP_PIN, ACTIVATE);
        currentOperation = UPOP;
        Serial.println("RT-11_going_up"); // used for testing purposes only
        
        // template for boolChecker function is:
        // bool rt11Working
        // int pinState1 = PIN_UP
        // int pinState2 = PIN_DOWN
        isLeftActuator = "UP";
        isRightActuator = "UP";
        isStop = false;
        isMovingUp = true;
        isMovingDown = false;
        isStationary = false;
        // bool isTableLevel
        // bool isTableMax
        // bool isBottomOut

        break; // exit the loop
    case DNOP:
        moveDown();
        digitalWrite(DOWN_PIN, ACTIVATE);
        currentOperation = DNOP;
        Serial.println("RT-11_going_down"); // used for testing purposes only

        // template for boolChecker function is:
        // bool rt11Working
        // int pinstate1 = PIN_DOWN
        // int pinState2 = PIN_UP
        isLeftActuator = "DOWN";
        isRightActuator = "DOWN";
        isStop = false;
        isMovingUp = false;
        isMovingDown = true;
        isStationary = false;
        // bool isTableLevel
        // bool isTableMax
        // bool isBottomOut

        break; // exit the loop
    case NOOP:
        currentOperation = NOOP;
        // stopMoving(); // original code
        stopMovement();
        break; // exit the loop
    default:
        // stopMoving(); // original code
        stopMovement();
        break; // exit the loop
    };
}; // end of setHeight

// THIS PART IS ALREADY INCLUDED IN THE SETUP FUNCTION
// void rt11_setup(void)
// {
//     deskSerial.begin(9600); // The controller uses 9600 bauds for serial communication
//     Serial.begin(115200);   // Use the in built UART for debugging
//     // Disable pullups turned on by espSoftwareSerial library
//     pinMode(5, INPUT); // GPIO 5
//     pinMode(led, OUTPUT); // led = 2 = GPIO 2
//     pinMode(DOWN_PIN, OUTPUT); // Default is HIGH
//     pinMode(UP_PIN, OUTPUT);   // Default is high
// 
//     pinMode(led, OUTPUT);
//     stopMoving(); // Make sure there is no movement on startup
// 
//     digitalWrite(led, 0);
// 
//     WiFi.mode(WIFI_STA);
//     WiFi.begin(ssid, password);
//     Serial.println("");
// 
//     // Wait for connection
//     while (WiFi.status() != WL_CONNECTED) {
//       delay(500);
//       Serial.print(".");
//     }
//     Serial.println("");
//     Serial.print("Connected to ");
//     Serial.println(ssid);
//     Serial.print("IP address: ");
//     Serial.println(WiFi.localIP());
// 
//     if (MDNS.begin("esp8266"))
//     {
//         Serial.println("MDNS responder started");
//     }
// 
//     server.on("/", handleRoot);
// 
//     server.on("/height", []()
//               {
//     char hstr[4];
//     sprintf(hstr, "%d", currentHeight);
//     server.send(200, "text/plain", hstr); });
// 
//     server.on("/abort", []()
//               {
//     stopMoving();
//     server.send(200, "text/plain", "Aborted current operation. please wait for some time before sending a new command"); });
// 
//     server.on("/setheight", []()
//               {
//     String message;
//     // Stop all ongoing operations 
//     if(currentOperation != NOOP) {
//       // 409 is conflict status code
//       server.send(409, "text/plain", "A height adjustment operation is goin on, please try later");
//     }else{
//       message = server.arg(0);
//       setHeight(atoi(message.c_str()));
//       server.send(200, "text/plain", message);  
//     } });
// 
//     server.begin();
//     Serial.println("HTTP server started");
// }; // end of setup
// 
// 
// Height should be set in bursts of x ms pulses, as it takes 4ms to read the current height
void rt11_loop(void)
{
    // the decodeSerial should be only called once in the loop
    currentHeight = decodeSerial();

    // These values come in either 3-4 digits , the last one always being the fractional value
    // i.e 1206 means 120.6 while 655 means 65.5
    if (currentHeight > 999)
    {
        heightMultiplier = 10;
    }
    else
    {
        heightMultiplier = 1;
    }

    Serial.println("CURR/REQ");
    Serial.println("current Height: " + currentHeight);
    Serial.println("requested height: " + requestedHeight);
    Serial.println("============");

    // This number is not precise, so compare in ball park
    // Based on current height
    // we assume that in the time taken to update the current height
    // the desk has moved by DESKMOVEDDURINGREAD inches
    if (currentOperation == UPOP)
    {
        if (currentHeight >= requestedHeight - DESKMOVEDDURINGREAD * heightMultiplier)
        {
            stopMovement();
            // stopMoving();
        }

        // added to function to display the position of the table
        displayPosition(); // displays the position of the table // from LEDeffects.h
    }
    if (currentOperation == DNOP)
    {
        if (currentHeight <= requestedHeight + DESKMOVEDDURINGREAD * heightMultiplier)
        {
            stopMovement();
            // stopMoving();
        }

        // added to function to display the position of the table
        displayPosition(); // displays the position of the table // from LEDeffects.h
    }
    // server.handleClient();
    // MDNS.update(); // no need for this when using ESPmDNS.h
}; // end of rt11_loop

// ================

#endif // LINEARACTUATOR_H

// =========================================================
// END OF PROGRAM
// =========================================================