// +-------------------------------------------------------------
//
// Equipment:
// DOIT ESP32 Dev Kit V1, MPU-6050, OLED SSD1306, WS2812B Addressable LEDS x3,
// 4-relay module, 2 linear actuators, RT-11(JCHT35K9) remote/handset,
// and 650W 12V Power supply
//
// File: ultrasonic.h
//
// Description:
//
// Provides the one way to find the calibration requirements for the
// MPU-6050, and the linear actuators to control the movement of the standup
// portion of the workstation.
//
// History:     2-Apr-2024     Scarecrow1965   Created
//
// +-------------------------------------------------------------

#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <Arduino.h>
#include <Wire.h>

#include <U8g2lib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <Ultrasonic.h>

extern int16_t ax, ay, az, gx, gy, gz;
extern double tempC;
extern Adafruit_SSD1306 display;
extern U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2;
extern uint8_t oled_LineH;

#include "filterData.h"
void filterData(void);

// delay variables
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

/*********
    Rui Santos
    Complete project details at https://RandomNerdTutorials.com/esp32-hc-sr04-ultrasonic-arduino/

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files.

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.
*********/
//
// const int trigPin = 5;
// const int echoPin = 18;
//
// define sound speed in cm/uS
// #define SOUND_SPEED 0.034
// #define CM_TO_INCH 0.393701
//
// long duration;
// float distanceCm;
// float distanceInch;
//
// void setup()
// {
//     Serial.begin(115200);     // Starts the serial communication
//     pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
//     pinMode(echoPin, INPUT);  // Sets the echoPin as an Input
// }
//
// void loop()
// {
//     // Clears the trigPin
//     digitalWrite(trigPin, LOW);
//     delayMicroseconds(2);
//     // Sets the trigPin on HIGH state for 10 micro seconds
//     digitalWrite(trigPin, HIGH);
//     delayMicroseconds(10);
//     digitalWrite(trigPin, LOW);
//
//     // Reads the echoPin, returns the sound wave travel time in microseconds
//     duration = pulseIn(echoPin, HIGH);
//
//     // Calculate the distance
//     distanceCm = duration * SOUND_SPEED / 2;
//
//     // Convert to inches
//     distanceInch = distanceCm * CM_TO_INCH;
//
//     // Prints the distance in the Serial Monitor
//     Serial.print("Distance (cm): ");
//     Serial.println(distanceCm);
//     Serial.print("Distance (inch): ");
//     Serial.println(distanceInch);
//
//     delay(1000);
// }

// Ultrasonic sensor variables
#define TRIG_PIN 32 // GPIO 32 on the ESP32
#define ECHO_PIN 35 // GPIO 35 on the ESP32 // receive only

// define sound speed in cm/uS
#define SOUND_SPEED 0.034
#define CM_TO_INCH 0.393701

long duration;

// variable for the base distance which will be determined during calibration
// measured with Reekon Tomahawk tape measure = 64.50 cm
// const float bottomOutDistanceCm = 58.41; // aka baseDistanceCm // according to displayAllGyroandDistance function
const float bottomOutDistanceCm = 61.43;
// const float bottomOutDistanceCm = 61.60;
const float bottomOutDistanceInch = 24.25; // aka baseDistanceInch // according to displayAllGyroandDistance function

// measured with Reekon Tomahawk tape measure = 73.29 cm
// const float normalDistanceCm = 58.2975; // +/- 1cm // according to displayAllGyroandDistance function
const float normalDistanceCm = 63.03;
// const float normalDistanceInch = 22.9513; // =/- .5 inch // according to displayAllGyroandDistance function

const float maxDistanceCm = 102.00; // according to displayAllGyroandDistance function
// const float maxDistanceInch = 39.37; // according to displayAllGyroandDistance function

// float baseDistanceCm = 58.41;  // according to displayAllGyroandDistance function
// float baseDistanceInch = 23.14; // according to displayAllGyroandDistance function

float baseDistanceCm;
float baseDistanceInch;
float totalDistanceCm;
float totalDistanceInch;
float distanceCm;
float distanceInch;

void getRawDistance(void)
{
    distanceCm = 0;
    distanceInch = 0;

    // Clears the trigPin
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    // Reads the echoPin, returns the sound wave travel time in microseconds
    // duration = pulseIn(ECHO_PIN, HIGH);
    duration = pulseIn(ECHO_PIN, HIGH, 25000);

    // Calculate the distance
    distanceCm = duration * SOUND_SPEED / 2;

    // Convert to inches
    distanceInch = distanceCm * CM_TO_INCH;

    // Prints the distance in the Serial Monitor
    // used for testing purposes only
    Serial.print("Distance (cm):\t");
    Serial.println(distanceCm);
    // Serial.print("Distance (inch):\t");
    // Serial.println(distanceInch);

    // delay for 5 ms to ensure we don't get repeated measures
    // delay(10);
    currentMillis = millis();
    if (currentMillis - previousMillis >= timePeriod005)
    {
        // delay(2); // Needed so we don't get repeated measures
        previousMillis = currentMillis;
    }
}; // end getRawDistance

void getFilteredDistance(void)
{
    float dist = 0;
    int count = 0;
    int filtersize = 300;
    float dist_buffer = 0;
    float dist_mean = 0;

    while (count < (filtersize + 101))
    {
        // Execute the command
        // Clears the trigPin
        digitalWrite(TRIG_PIN, LOW);
        delayMicroseconds(2);
        // Sets the trigPin on HIGH state for 10 micro seconds
        digitalWrite(TRIG_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG_PIN, LOW);

        // Reads the echoPin, returns the sound wave travel time in microseconds
        // duration = pulseIn(ECHO_PIN, HIGH);
        duration = pulseIn(ECHO_PIN, HIGH, 25000);

        // Calculate the distance
        dist = duration * SOUND_SPEED / 2;

        if (count > 100 && count <= (filtersize + 100))
        {
            // First 100 measures are discarded
            dist_buffer = dist_buffer + dist;
        }

        if (count == (filtersize + 100))
        {
            dist_mean = dist_buffer / filtersize;
        }
        count++;

        // delay for 2 ms to ensure we don't get repeated measures
        currentMillis = millis();
        if (currentMillis - previousMillis >= timePeriod002)
        {
            // delay(2); // Needed so we don't get repeated measures
            previousMillis = currentMillis;
        }
    }

    distanceCm = dist_mean;

    // Convert to inches
    distanceInch = distanceCm * CM_TO_INCH;

    // Prints the distance in the Serial Monitor
    // used for testing purposes only
    Serial.print("Current Distance (cm):\t");
    Serial.println(distanceCm);
    // Serial.print("Current Distance (inch):\t");
    // Serial.println(distanceInch);
}; // end getFilteredDistance

// // function to display information on the OLED screen
// void displayAllGyroandDistance(void)
// {
// #ifdef USE_LIBRARY2
//     // #include <MPU6050.h>
//     // extern MPU6050 MPU2;
//     // this will get the filtered gyro information
//     filterData();
//     double temp = MPU2.getTemperature();
//     tempC = (temp / 340.00) + 36.53; // convert to Celsius using MPU6050.h library
//     // this will get the distance from the ultrasonic sensor
//     getFilteredDistance();
// #endif
//     display.clearDisplay();
//     u8g2.clearBuffer();
//     u8g2.home();
//     u8g2.drawFrame(0, 0, u8g2.getWidth(), u8g2.getHeight()); // Draw a border around the display
//     u8g2.setFont(u8g2_font_profont10_tf);
//     u8g2.setCursor(3, oled_LineH * 1 + 2);
//     u8g2.println("Distance(cm)= ");
//     u8g2.print(distanceCm);
//     // u8g2.setCursor(3, oled_LineH * 2 + 2);
//     // u8g2.println("Distance(inch)= ");
//     // u8g2.print(distanceInch);
//     u8g2.setCursor(3, oled_LineH * 3 + 2);
//     u8g2.println("Accel(m/s^2)");
//     u8g2.setCursor(3, oled_LineH * 4 + 2);
//     u8g2.print("X:");
//     u8g2.print(ax);
//     u8g2.print(", Y:");
//     u8g2.print(ay);
//     u8g2.print(", Z:");
//     u8g2.print(az);
//     u8g2.setCursor(3, oled_LineH * 5 + 2);
//     u8g2.println("Gyro(rps)");
//     u8g2.setCursor(3, oled_LineH * 6 + 2);
//     u8g2.print("X:");
//     u8g2.print(gx, 1);
//     u8g2.print(", Y:");
//     u8g2.print(gy);
//     u8g2.print(", Z:");
//     u8g2.print(gz);
//     u8g2.setCursor(3, oled_LineH * 7 + 2);
//     u8g2.print("Temp= ");
//     u8g2.print(tempC);
//     u8g2.print(" ");
//     u8g2.print(char(176));
//     u8g2.setFont(u8g2_font_profont10_tf);
//     u8g2.print("C");
//     u8g2.sendBuffer(); // Send it out
// }; // end displaying the MPU-6050 info

#endif // ULTRASONIC_H

// =========================================================
// END OF PROGRAM
// =========================================================