// +-------------------------------------------------------------
//
// Equipment:
// DOIT ESP32 Dev Kit V1, MPU-6050, OLED SSD1306, WS2812B Addressable LEDS x3,
// 4-relay module, 2 linear actuators, RT-11(JCHT35K9) remote/handset,
// and 650W 12V Power supply
//
// File: dumpData.h
//
// Description:
//
// Provides the dump of gyro information through the Serial Monitor for the
// MPU-6050, and the linear actuators to control the movement of the standup
// portion of the workstation.
//
// History:     6-Mar-2024     Scarecrow1965   Created
//
// +-------------------------------------------------------------

#ifndef DUMPDATA_H
#define DUMPDATA_H

#include <Arduino.h>
#include <Wire.h>

// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class
// using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

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
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
// #include "dataLogger.h" // included in case you want to log the data to a file

// #define USE_LIBRARY1
#ifdef USE_LIBRARY1
#include <MPU6050_6Axis_MotionApps20.h>

// from main.cpp variables
// extern int16_t ax, ay, az, gx, gy, gz, temp;
// extern uint16_t accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z;
// extern double tempC;
// extern String fileName;
// extern File dataFile;

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
// #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
// #include "Wire.h"
// #endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
// MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.

   For the Galileo Gen1/2 Boards, there is no INT pin support. Therefore
   the INT pin does not need to be connected, but you should work on getting
   the timing of the program right, so that there is no buffer overflow.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
#define OUTPUT_READABLE_REALACCEL

// Uncomment if you are using an Arduino-Style Board
// #define ARDUINO_BOARD

// Uncomment if you are using a Galileo Gen1 / 2 Board
// #define GALILEO_BOARD

// #define LED_PIN 13 // (Galileo/Arduino is 13)
// bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
VectorFloat gravity; // [x, y, z]            gravity vector
Quaternion q;        // [w, x, y, z]         quaternion container
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

// This function is not required when using the Galileo
volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
    mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void dumpDataSetup()
{
    // join I2C bus (I2Cdev library doesn't do this automatically)
    // #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    //     Wire.begin();
    // #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    //     Fastwire::setup(400, true);
    // #endif // I2CDEV_IMPLEMENTATION

    // Serial.begin(115200);
    // while (!Serial)
    //     ;

    // initialize device
    // Serial.println(F("Initializing I2C devices..."));
    // mpu.initialize();

    // verify connection
    // Serial.println(F("Testing device connections..."));
    // Serial.println(F("MPU6050 connection "));
    // Serial.print(mpu.testConnection() ? F("successful") : F("failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read())
        ; // empty buffer
    while (!Serial.available())
        ; // wait for data
    while (Serial.available() && Serial.read())
        ; // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    // mpu.setXGyroOffset(220);
    // mpu.setYGyroOffset(76);
    // mpu.setZGyroOffset(-85);
    // mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
};

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void dumpDataLoop()
{
    // if programming failed, don't try to do anything
    if (!dmpReady)
        return; // exit the function

    // wait for MPU interrupt or extra packet(s) available

    // #ifdef ARDUINO_BOARD
    //     while (!mpuInterrupt && fifoCount < packetSize)
    //     {
    //     }
    // #endif

    // #ifdef GALILEO_BOARD
    // delay(10);
    // #endif

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

        // otherwise, check for DMP data ready interrupt (this should happen frequently)
    }
    else if (mpuIntStatus & 0x02)
    {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize)
            fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

#ifdef OUTPUT_READABLE_YAWPITCHROLL

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        // =============================== //
        // display Euler angles in degrees
        // =============================== //
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        Serial.print("ypr\t");
        Serial.print(ypr[0] * 180 / M_PI);
        Serial.print("\t");
        Serial.print(ypr[1] * 180 / M_PI);
        Serial.print("\t");
        Serial.println(ypr[2] * 180 / M_PI);
#endif // OUTPUT_READABLE_YAWPITCHROLL

#ifdef OUTPUT_READABLE_EULER
        // =============================== //
        // display Euler angles in degrees
        // =============================== //
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetEuler(euler, &q);
        Serial.print("euler\t");
        Serial.print(euler[0] * 180 / M_PI);
        Serial.print("\t");
        Serial.print(euler[1] * 180 / M_PI);
        Serial.print("\t");
        Serial.println(euler[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_READABLE_REALACCEL
        // display real acceleration, adjusted to remove gravity
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        Serial.print("areal\t");
        Serial.print(aaReal.x);
        Serial.print("\t");
        Serial.print(aaReal.y);
        Serial.print("\t");
        Serial.println(aaReal.z);
#endif
    }
};

void dumpData(void)
{
    dumpDataSetup();
    Serial.println("Dump Data Setup Completed");
    // dumpDataLoop();
    // Serial.println("Dump Data Start Completed");
};

// ================================================================
// ===              DIGITAL LEVEL PROGRAM                       ===
// ================================================================
//  FROM: https://github.com/DesignBuildDestroy/digital_spirit_level/blob/main/Digital_Level_v1.ino or
// https://www.youtube.com/watch?v=232jer4HIZc&t=13s
// 
// #include "I2Cdev.h"
// #include "Wire.h"
// #include "MPU6050_6Axis_MotionApps20.h" //Must include for DMP holds firmware hex to push to MPU on init
// #include <Adafruit_SSD1306.h>
// #include <Adafruit_GFX.h>
// #include <EEPROM.h>
// #include <Fonts/FreeMono9pt7b.h> //Font for menus only
// 
// // SSD1306 OLED
// const byte SCREEN_WIDTH = 128; // OLED display width, in pixels
// const byte SCREEN_HEIGHT = 64; // OLED display height, in pixels
// Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
// 
// //  Menu & menu button definitions
// const byte MENU_BTN = 3;
// const byte ENTER_BTN = 4;
// unsigned long menuStartTime = 0; // To use with menuTimeout to track if we should auto exit
// byte menuItem = 0;               // Hold the current menu's item selected
// const int menuTimeout = 10000;   // Time out value for menus to auto exit
// bool precisionMode = false;      // Menu option to show 1 decimal places instead of rounding to whole.
// 
// // MPU Address for I2C
// byte devAddr = 0x68;
// MPU6050 mpu(devAddr);
// 
// VectorFloat QtoEulerAngle(Quaternion qt)
// {
//     VectorFloat ret;
//     double sqw = qt.w * qt.w;
//     double sqx = qt.x * qt.x;
//     double sqy = qt.y * qt.y;
//     double sqz = qt.z * qt.z;

//     ret.x = atan2(2.0 * (qt.x * qt.y + qt.z * qt.w), (sqx - sqy - sqz + sqw));
//     ret.y = asin(2.0 * (qt.x * qt.z - qt.y * qt.w) / (sqx + sqy + sqz + sqw)); // Adafruit uses -2.0 *..
//     ret.z = atan2(2.0 * (qt.y * qt.z + qt.x * qt.w), (-sqx - sqy + sqz + sqw));

//     // Added to convert Radian to Degrees
//     ret.x = ret.x * 180 / PI;
//     ret.y = ret.y * 180 / PI;
//     ret.z = ret.z * 180 / PI;
//     return ret;
// }; // end of QtoEulerAngle
// 
// // Write a 2 byte word starting at the startAddr provided
// void epromWriteWord(int startAddr, int value)
// {
//     EEPROM.update(startAddr, value);
//     EEPROM.update(startAddr + 1, value >> 8);
// }; // end of epromWriteWord
// 
// // Return a 2 byte word read from a starting EEPROM address
// int epromReadWord(int startAddr)
// {
//     int value = EEPROM.read(startAddr);
//     value = value | (EEPROM.read(startAddr + 1) << 8);
//     return value;
// }; // end of epromReadWord
// 
// void levelgetCalibration()
// {
//     // Get the saved calibration values from EEPROM and update MPU
//     // Address Always starts at 0 ends at 11, 2 bytes per axis
// 
//     // Future Check if we have anything saved (look for all FF)
//     // if not assume calibration not run and skip setting just MPU default
//     // Eventually prompt user to calibrate! if there's enough space left in memory of this sketch!!!
//     mpu.setXAccelOffset(epromReadWord(0));
//     mpu.setYAccelOffset(epromReadWord(2));
//     mpu.setZAccelOffset(epromReadWord(4));
//     mpu.setXGyroOffset(epromReadWord(6));
//     mpu.setYGyroOffset(epromReadWord(8));
//     mpu.setZGyroOffset(epromReadWord(10)); // last address read would be 11 decimal in eeprom
// }; // end of getCalibration
// 
// void levelsetCalibration()
// {
//     // Run DMP auto calibration and then get those values and save to EEprom
//     // This should only be called when Auto Calibrate option is selected
//     // to preserve EEPROM life use update instead of write
// 
//     // Run autocalibration 6 times
//     mpu.CalibrateAccel(6);
//     mpu.CalibrateGyro(6);
// 
//     // Get the final values that were saved and move them to EEPROM
//     int Data[3];
//     // Accel Offsets
//     // static int8_t I2Cdev::readWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data, uint16_t timeout = I2Cdev::readTimeout, void *wireObj = (void *)0)
//     I2Cdev::readWords(devAddr, 0x06, 3, (uint16_t *)Data);
//     epromWriteWord(0, Data[0]);
//     epromWriteWord(2, Data[1]);
//     epromWriteWord(4, Data[2]);
//     // Gyro Offsets
//     // static int8_t I2Cdev::readWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data, uint16_t timeout = I2Cdev::readTimeout, void *wireObj = (void *)0)
//     I2Cdev::readWords(devAddr, 0x13, 3, (uint16_t *)Data);
//     epromWriteWord(6, Data[0]);
//     epromWriteWord(8, Data[1]);
//     epromWriteWord(10, Data[2]); // Last byte written is eeprom address 11 decimal
// }; // end of setCalibration
// 
// void levelsetup()
// {
//    // From JRowber sample to setup MPU6050 connection
// #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
//     Wire.begin();
//     Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
// #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
//     Fastwire::setup(400, true);
// #endif
// 
//     Serial.begin(115200); // Only needed for Debug otherwise can comment out
// 
//     // Set menu buttons WITH PULLUPS
//     pinMode(MENU_BTN, INPUT_PULLUP);
//     pinMode(ENTER_BTN, INPUT_PULLUP);
// 
//     // Init OLED Display
//     display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
// 
//     display.clearDisplay();
//     display.setTextSize(2);
//     display.setTextColor(SSD1306_WHITE); // not really necessary for this display
//     display.println();
//     display.println(F("Starting!!"));
//     display.display();
// 
//     // Init MPU6050
//     mpu.initialize();
//     devStatus = mpu.dmpInitialize();
// 
//     // Get stored EEPROM Calibration values and send to MPU
//     // Otherwise default to predefined and display Calibration needed!
//     getCalibration();
// 
//     // make sure it worked - Because we are pushing firmware on startup of DMP
//     // we need to check that everything actually went as planned devStatus 0 is success.
//     if (devStatus == 0)
//     {
//         mpu.setDMPEnabled(true);
//         mpuIntStatus = mpu.getIntStatus();
//         // set our DMP Ready flag so the main loop() function knows it's okay to use it
//         dmpReady = true;
// 
//         // get expected DMP packet size for later comparison
//         packetSize = mpu.dmpGetFIFOPacketSize();
//     }
//     else
//     {
//         // IMU Failed for some reason show error on OLED
//         display.clearDisplay();
//         display.setCursor(0, 40);
//         display.println("IMU FAIL");
//         display.display();
//         Serial.println ("IMU FAIL"); // used for testing puposes only
//     }
// }; // end of setup
// 
// // andles display routine for Main menu
// void dispMenu(byte itemSelect)
// {
//     display.clearDisplay();
//     display.setRotation(0);
//     display.setFont(&FreeMono9pt7b);
//     display.setCursor(0, 14);
//     display.setTextColor(WHITE);
//     display.setTextSize(1);
//     display.println(F("   Normal"));
//     display.println(F(" Precision"));
//     display.println(F(" Calibrate"));
// 
//     if (itemSelect == 0)
//     {
//         display.drawRect(5, 2, 120, 17, SSD1306_WHITE); // Draw box around item 1
//     }
//     if (itemSelect == 1)
//     {
//         display.drawRect(5, 18, 120, 20, SSD1306_WHITE); // Draw box around item 2
//     }
//     if (itemSelect == 2)
//     {
//         display.drawRect(5, 36, 120, 20, SSD1306_WHITE); // Draw box around item 3
//     }
// 
//     // Display everything
//     display.display();
// }; // end of dispMenu
// 
// // Handles display routine for Calibration sub menu
// void dispCalibrate(byte itemSelect)
// {
//     display.clearDisplay();
//     display.setFont(&FreeMono9pt7b);
//     display.setCursor(2, 14);
//     display.setTextColor(WHITE);
//     display.setTextSize(1);
//     display.println(F("Lay me down"));
//     display.println(F("  face up"));
//     display.setCursor(0, 55);
//     display.println(F(" START   X"));
// 
//     if (itemSelect == 0)
//     {
//         display.drawRect(6, 41, 65, 20, SSD1306_WHITE); // Draw box around START
//     }
//     else
//     {
//         display.drawRect(95, 41, 20, 20, SSD1306_WHITE); // Draw box around X
//     }
// 
//     // Display everything
//     display.display();
// }; // end of dispCalibrate
// 
// // Handle user button interactions for the main menu options
// void menuMainWait()
// {
//     // Show the main Menu text on screen
//     menuItem = 0;
//     dispMenu(menuItem);
// 
//     // Wait for key press to go to selected item
//     menuStartTime = millis(); // Reset menu timout
// 
//     // Wait for next user action in the menu, if no buttons pressed
//     // after menuTimeout limit, auto exit the menu
//     while (millis() - menuStartTime <= menuTimeout)
//     {
//         if (digitalRead(MENU_BTN) == LOW)
//         {
//             delay(200);               // Debounce
//             menuStartTime = millis(); // Reset menu timout
//             menuItem++;
//             if (menuItem >= 3)
//             {
//                 // We reached menu item limit, roll around to start
//                 menuItem = 0;
//                 dispMenu(menuItem);
//             }
//             else
//             {
//                 // Draw rect around next menu item selected
//                 dispMenu(menuItem);
//             }
//         }
//         if (digitalRead(ENTER_BTN) == LOW)
//         {
//             delay(200);               // Debounce
//             menuStartTime = millis(); // Reset menu timout
//             if (menuItem == 0)
//             {
//                 // Normal mod,e Angle is rounded to whole number
//                 precisionMode = false;
//                 break; // exit the loop
//             }
//             if (menuItem == 1)
//             {
//                 // Precision mode, Angle rounded to 1st decimal place
//                 precisionMode = true;
//                 break; // exit the loop
//             }
//             if (menuItem == 2)
//             {
//                 // Show calibration sub menu
//                 menuCalibrateWait();
//                 break; // exit the loop
//             }
//         }
//     }
// }; // end of menuMainWait
// 
// // Auto Calibration menu, will use MPU auto calibration feature
// // and save the values to EEPROM that are pulled on startup
// // Calibration only needs to be run one time and values should stay
// // acurate from then on.
// // NOTE: Calibration must be done on an already known LEVEL surface that
// // is level front to back and side to side like a level table top with the device laying down
// // screen facing up.
// 
// void menuCalibrateWait()
// {
//     // Show the sub menu for CALIBRATION
//     menuItem = 1; // Default to EXIT
//     dispCalibrate(menuItem);
// 
//     // Wait for next user action in the menu, if no buttons pressed
//     // after menuTimeout limit, auto exit the menu
//     menuStartTime = millis(); // Reset menu start timer
//     while (millis() - menuStartTime <= menuTimeout)
//     {
//         if (digitalRead(MENU_BTN) == LOW)
//         {
//             delay(300); // Debounce
//             if (menuItem == 1)
//             {
//                 menuItem = 0;
//                 dispCalibrate(menuItem);
//             }
//             else
//             {
//                 menuItem = 1;
//                 dispCalibrate(menuItem);
//             }
//         }
//         if (digitalRead(ENTER_BTN) == LOW)
//         {
//             delay(200); // Debounce
//             if (menuItem == 0)
//             {
//                 // Start Calbiration process takes a few seconds to run
//                 // So display something to let user know it's working
//                 display.clearDisplay();
//                 display.setCursor(0, 40);
//                 display.println("CALIBRATING");
//                 display.display();
// 
//                 setCalibration(); // This runs the actual calibration
// 
//                 display.clearDisplay();
//                 display.setCursor(0, 40);
//                 display.println("COMPLETE!");
//                 display.display();
//                 delay(500); // Give user time to see complete message before exit
//                 break; // exit the loop
//             }
//             if (menuItem == 1)
//             {
//                 // X selected - Exit Menus;
//                 break; // exit the loop
//             }
//         }
//     }
// }; // end of menuCalibrateWait
// 
// // For OLED, show the angle at the correct orientation based on
// // How the level is positioned, whether Precision mode is set
// // changes decimal display and font size to fit characters
// void formatDisplay(double angleVal, byte dispRotate)
// {
// 
//     display.clearDisplay();
//     display.setCursor(0, 0);
//     display.setFont();
//     display.setRotation(dispRotate);
// 
//     // Set font size, text position and angle value based on MPU positions and precision setting
//     // Precision mode shows 1 decimal place value, otherwise round to whole number
//     if (precisionMode == false)
//     {
//         if (dispRotate == 0 || dispRotate == 2)
//         {
//             // Horizontal
//             display.setCursor(40, 10);
//         }
//         else
//         {
//             // Vertical
//             display.setCursor(3, 30);
//         }
// 
//         display.setTextSize(5);
//         display.println(round(abs(angleVal)));
//     }
//     else
//     {
//         // Precision Mode show 1 decimal place smaller font when vertical
//         if (dispRotate == 0 || dispRotate == 2)
//         {
//             // Horizontal
//             display.setTextSize(4);
//             display.setCursor(20, 10);
//         }
//         else
//         {
//             // Vertical
//             display.setTextSize(2);
//             display.setCursor(6, 40);
//         }
// 
//         display.println(abs(angleVal), 1);
//     }
// 
//     // If we are level or plumb +-1 degree Display it under the angle
//     // again format for rotation position use LEVEL or PLUMB text indicator
//     // Do this inside a white rectangle with black font so it stands out
//     // since it will be very small font size
//     if (round(abs(angleVal)) <= 1)
//     {
//         display.setTextSize(1);
//         display.setTextColor(BLACK, WHITE);
//         if (dispRotate == 0 || dispRotate == 2)
//         {
//             // Horizontal
//             display.setCursor(37, 50);
//             display.println(F("   LEVEL  "));
//         }
//         else
//         {
//             // Vertical
//             display.setCursor(0, 90);
//             display.println(F("   PLUMB  "));
//         }
//     }
// 
//     // Reset text color
//     display.setTextColor(WHITE, BLACK);
//     // Show the text
//     display.display();
// }; // end of formatDisplay
// 
// // MAIN PROGRAM LOOP!!
// void levelloop()
// {
//     if (digitalRead(MENU_BTN) == LOW)
//     {
//         delay(500);
//         menuMainWait();
//     }
//     // if programming failed, don't try to do anything
//     if (!dmpReady)
//     {
//         return; // exit the function
//     }
//     // Get the Quaternion values from DMP buffer
//     if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
//     {
//         mpu.dmpGetQuaternion(&q, fifoBuffer);
// 
//         // Calc angles converting Quaternion to Euler this was giving more stable acurate results compared to
//         // getting Euler directly from DMP. I think Quaternion conversion takes care of gimble lock.
//         VectorFloat ea = QtoEulerAngle(q);
// 
//         // DEBUG ONLY COMMENT OUT UNLESS NEEDED
//         /*  Serial.print("quat\t");
//           Serial.print(ea.x);
//           Serial.print("\t");
//           Serial.print(ea.y);
//           Serial.print("\t");
//           Serial.print(ea.z);
//           Serial.println("\t");
//         */
// 
//         float angVal = 0;
//         float dispRotate = 0;
// 
//         // Figure out how to display the data on OLED at the right rotation
//         // Like flipping a phone around rotating the display - trial and error...took a while
//         // TOP IS TOP means TOP of OLED is the TOP of the screen, RIGHT IS TOP Right side of OLED is now the top, etc.
// 
//         // TOP IS TOP angling Right side (LEVEL)
//         if (ea.x > 0 && ea.y > 35 && ea.y <= 90)
//         {
//             angVal = (90 - ea.y);
//             dispRotate = 0;
//         }
//         // Angling right side up RIGHT is TOP (toward PLUMB)
//         if (ea.x > 0 && ea.y <= 35 && ea.y > -35)
//         {
//             angVal = ea.y;
//             dispRotate = 1;
//         }
//         // LEFT IS TOP (PLUMB)
//         if (ea.x > 0 && ea.y <= -35 && ea.y > -90)
//         {
//             angVal = ea.y + 90;
//             dispRotate = 2;
//         }
//         // TOP IS TOP angling Left side (LEVEL)
//         if (ea.x < 0 && ea.y > 35 && ea.y <= 90)
//         {
//             angVal = (90 - ea.y);
//             dispRotate = 0;
//         }
//         // Upside down - BOTTOM IS TOP (LEVEL)
//         if (ea.x < 0 && ea.y <= 35 && ea.y > -35)
//         {
//             angVal = ea.y;
//             dispRotate = 3;
//         }
//         // Upside down - BOTTOM IS TOP
//         if (ea.x < 0 && ea.y <= -35 && ea.y > -90)
//         {
//             angVal = ea.y + 90;
//             dispRotate = 2;
//         }
//         // laying down face up - this is also Calibration position
//         // need to also check Z here or it can get confused with another position
//         if (ea.x < 5 && ea.x > -20 && ea.y <= 35 && ea.y > -35 && ea.z < 5)
//         {
//             angVal = 90 - ea.y;
//             if (angVal > 50)
//             {
//                 angVal -= 90; // bandaid fix from being laid flat...
//             }
//             dispRotate = 0;
//         }
// 
//         // Display the data on OLED formatted as we need for the position
//         formatDisplay(angVal, dispRotate);
//     }
// }; // END OF MAIN LOOP

#endif // USE_LIBRARY1

#endif // DUMPDATA_h

// =========================================================
// END OF PROGRAM
// =========================================================
