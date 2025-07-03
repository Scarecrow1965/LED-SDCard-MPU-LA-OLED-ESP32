// +-------------------------------------------------------------
//
// Equipment:
// DOIT ESP32 Dev Kit V1, MPU-6050, OLED SSD1306, WS2812B Addressable LEDS x3,
// 4-relay module, 2 linear actuators, RT-11(JCHT35K9) remote/handset,
// and 650W 12V Power supply
//
// File: main.cpp
//
// Description:
//
// Provides the computer workstation the ability to use the remote/handset,
// MPU-6050, and the linear actuators to control the movement of the standup
// portion of the workstation.
//
// History:     11-Mar-2024     Scarecrow1965   Created
//
// +-------------------------------------------------------------

#include <Arduino.h>
#include <Wire.h>

// ================
// libraries for the SD card reader
// #include <SPI.h>
#include <FS.h>
#include <SD.h>
#include <vector>    // used to carry out alphabetical sorting of the files
#include <algorithm> // used to carry out alphabetical sorting of the files
// these variables are to ensure the data.csv does not get overwritten
const char *root = "/";
const char *baseName = "/data";
const char *extension = ".csv";
int fileNumber = 0;
String fileName;
File dataFile;   // for the data.csv file
File configFile; // for the config.csv file

// adding the SD card reader for the ESP32 using the SPI library
// from: https://www.youtube.com/watch?v=e1xOgZsnAuw
// if you want to use the SD_Data GPIOs, you have to use the HSPI or VSPI, and use the ESP32 MMC library
// which means that the pins allocation would have to be 14, 13, 4, 12, and 15 on a normal SD Card
// data1= 4, data2= 12, data3= 13, CMD= 15, CLK= 14
// this would greatly speed up the SD card reading and writing
#define SCK 18  // GPIO 18 = VSPI_CLK = Pin 22 on ESP32 DEVKIT V1
#define MISO 19 // GPIO 19 = VSPI_MISO = pin 21 on ESP32 DEVKIT V1
#define MOSI 23 // GPIO 23 = VSPI_MOSI = pin 16 on ESP32 DEVKIT V1
#define CS 5    // GPIO 5 = VSPI_CS = pin 23 on ESP32 DEVKIT V1
// ================

// ================
// if using MMC library for the SD Card, the following line must be commented out
bool bLED = false; // LED state
// ================

// ================
// start our fps tracking
float fps = 0;
// ================

// ================
// libraries for the OLED
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <U8g2lib.h>
// #ifdef U8X8_HAVE_HW_SPI
// #include <SPI.h>
// #endif

// #ifdef U8X8_HAVE_HW_I2C
// #include <Wire.h>
// #endif
// This will enable for the OLED screen to display information using the U8G2 library
// definition of OLED display SSD1306 for ESP32
#define OLED_CLOCK 22 // SCA pin on Display = pin 17 (I2C_SCL) on ESP32 DEVKIT V1 = GPIO 22
#define OLED_DATA 21  // SDL pin on display = pin 20 (I2C_SDA) on ESP32 DEVKIT V1 = GPIO 21
// U8G2 SSD1306 Driver here to run OLED Screen
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, OLED_CLOCK, OLED_DATA, U8X8_PIN_NONE); // This works but according to the function, it shouldn't
// uint8_t oled_LineH = 0;

// ADAFRUIT SSD1306 Driver here to run animation
#define SCREEN_I2C_ADDR 0x3C // or 0x3C // also should not be the same address as the MPU6050 which causes problems
// #define SCREEN_I2C_ADDR 0x68 // alternate addr for the OLED
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RST_PIN -1  // Reset pin (-1 if not available)

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST_PIN);
// ================

// ================
// the followwing code is for the Wifi
// and NTP, and datestamping for datalogging purposes,
// and lastly for the display onto the web interfaace to see the MPU-6050 in action
#include <Wifi.h>
#include <time.h>
#include <esp_sntp.h>
const char *ssid = "Sc4r3Cr0w245";
const char *password = "R0b0tech_1";
const char *ntpServer1 = "pool.ntp.org";
const char *ntpServer2 = "time.nist.gov";
const long gmtOffset_sec = 3600;
const int daylightOffset_sec = 3600;
//
// const char *time_zone = "CET-1CEST,M3.5.0,M10.5.0/3"; // TimeZone rule for Europe/Rome including daylight adjustment rules (optional)
const char *time_zone = "CST6CDT,M3.2.0,M11.1.0"; // TimeZone rule for America/Winnipeg including daylight adjustment rules (TZ_America_Winnipeg)
// ================

// ================
// RT-11 setup variables
// #include "rt11.h"
//
// removed for RT-11 interface
// interrupt pin is now used for the RT-11 interface
// #define INTERRUPT_PIN 4
// interrupt connected to MPU-6050
// GPIO 4 on the ESP32
//
// changed to hardware serial since it was conflicting with serial monitor and had to use 2 UARTs
// #include <SoftwareSerial.h>
#include <HardwareSerial.h>
//
bool rt11Working = false; // will need some sort of flag to see if RT11 is working
//
// UART setup
// for the DOIT ESP32 V1 Kit
// the following pins are used for UART2 but variables were used for the SoftwareSerial library
// hence commented out
const int uart2rxPin = 16; // for GPIO 16 for UART2 Rx
const int uart2txPin = 17; // for GPIO 17 for UART2 Tx
//
HardwareSerial deskSerial(2); // 2 = UART2
// confirrmed through HardwareSerial.cpp line 78 (RX2 16) and 87 (TX2 17)
// SoftwareSerial(uint8_t receivePin, uint8_t transmitPin, bool inverse_logic = false);
// SoftwareSerial deskSerial(Rx GPIO (SCL), Tx GPIO (SDA));
// SoftwareSerial deskSerial(5, 4);  // original code
// SoftwareSerial deskSerial(22, 23); // for the DOIT ESP32 V1 Kit, SCL = GPIO 22, SDA = GPIO 23
// ================

// ================
// libraries for MPU-6050

// additional libraries for the MPU-6050
// #include <MPU6050.h>
// #include <Adafruit_MPU6050.h>
// #include <MPU6050_6Axis_MotionApps20.h>
// variables for the MPU-6050
#include <I2Cdev.h>

const int MPU6050_addr = 0x68; // to initialize the address of the MPU-6050
int16_t ax, ay, az, gx, gy, gz;
double tempC;
bool isCalibrationLevel = false;

// ================
#define USE_LIBRARY1
#ifdef USE_LIBRARY1
#include <MPU6050_6Axis_MotionApps20.h>
#endif
// this library does not use MPU6050.h
#include "dumpData.h"
// #include "selfBalancing.h" // not used at this time
// ================

// ================
#define USE_LIBRARY2
#ifdef USE_LIBRARY2
#include <MPU6050.h>
MPU6050 MPU2;
#endif

// these libraries uses MPU6050.h
#include "filterData.h"
#include "calibrateData.h"
void calibratingData(uint8_t saved);
extern int error;

// libraries for the addressable LEDs
#define FASTLED_INTERNAL // Suppress build banner
#include <FastLED.h>
#include "LEDeffects.h"
// for LEDs setup
// void ledStripTest(CRGB *leds, int numLEDs);

// definitions of assets and parameters for ESP32
#define LED_PIN0 13 // Test Bench = option #1: GPIO25(pin 8), option #2:GPIO13(pin 3) for the ESP32
#define LED_PIN1 12 // Standup Desk = option #1: GPIO26(pin 7), option #2:GPIO12(pin 4) for the ESP32
#define LED_PIN2 14 // Storage Bench = option #1: GPIO27(pin 6), option #2:GPIO14(pin 5) for the ESP32

// Define the number of LEDs in each strip
#define NUM_LED_PIN0 278 // Test Bench
#define NUM_LED_PIN1 222 // Standup Desk
#define NUM_LED_PIN2 242 // Storage Bench

// #define BRIGHTNESS 32    // 0 to 255 of the LEDs brightness
int BRIGHTNESS = 32;     // 0 to 255 of the LEDs brightness
#define LED_TYPE WS2812B // type works for the WS2815 which I am using
#define COLOR_ORDER RGB

// Define the LED arrays for each strip
CRGB led_Strip0[NUM_LED_PIN0]; // Test Bench
CRGB led_Strip1[NUM_LED_PIN1]; // Standup Desk
CRGB led_Strip2[NUM_LED_PIN2]; // Storage Bench

// these libraries do not involve the MPU6050.h but need filterData.h
#include "dataLogger.h"
#include "LinearActuator.h"
void checkTableMovement(bool &dataChanged);
extern bool dataChanged;

#include "standupTableMovement.h"
// this library uses MPU6050.h and is for the ultrasonic sensor
#include "ultrasonic.h"
// void getDistance(void);
#include "seeOLED.h"
// ================

// ================
#define USE_LIBRARY3
#ifdef USE_LIBRARY3
#include <MPU6050.h>
#include <Adafruit_MPU6050.h>
#endif

#include "IMU_Zero.h"
// ================

// ================
// All from https://randomnerdtutorials.com/esp32-mpu-6050-web-server/
/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-mpu-6050-web-server/

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <Arduino_JSON.h>

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// Create an Event Source on /events
AsyncEventSource events("/events");

// Json Variable to Hold Sensor Readings
JSONVar readings;

sensors_event_t a, g, temp;
// ================

// ================
// functions for the web interface
// ================
// Retrieves MPU data and placess it into the readings JSON object
String getMPUReadings()
{
    filterData();

    readings["gyroX"] = String(gyro_x);
    readings["gyroY"] = String(gyro_y);
    readings["gyroZ"] = String(gyro_z);

    readings["accX"] = String(accel_x);
    readings["accY"] = String(accel_y);
    readings["accZ"] = String(accel_z);

    readings["temp"] = String(tempC);

    String jsonString = JSON.stringify(readings);

    return jsonString;
}; // end of the getGyroReadings function

// Initialize WiFi
void initWiFi(void)
{
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    Serial.println("");
    Serial.print("Connecting to WiFi...");
    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print(".");
        delay(1000);
    }
    Serial.println("");
    Serial.println(WiFi.localIP());
};

void webServer(void)
{
    // Handle Web Server
    if (MDNS.begin("esp32-mpu6050-web-server"))
    {
        Serial.println("MDNS responder started");
    }

    // AsyncCallbackWebHandler &AsyncWebServer::on(const char *uri, WebRequestMethodComposite method, ArRequestHandlerFunction onRequest, ArUploadHandlerFunction onUpload = nullptr, ArBodyHandlerFunction onBody = nullptr)
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(SD, "/index.html", "text/html"); });
    // inline void AsyncWebServerRequest::send(AsyncWebServerRequest::FS &fs, const String &path, const char *contentType = "", bool download = false, AwsTemplateProcessor callback = nullptr)

    server.serveStatic("/", SD, "/");

    // RT11 based information
    // Correct usage of send method for a route
    // disabled:
    server.on("/height", [](AsyncWebServerRequest *request)
              {
        char hstr[4];
        sprintf(hstr, "%d", currentHeight);
        request->send(200, "text/plain", hstr); });
    // Correct usage of send method for another route
    // disabled:
    server.on("/abort", [](AsyncWebServerRequest *request)
              {
        stopMovement();
        // stopMoving();
        request->send(200, "text/plain", "Aborted current operation. please wait for some time before sending a new command"); });
    // Handling POST requests and accessing POST parameters
    // disabled:
    server.on("/setheight", HTTP_POST, [](AsyncWebServerRequest *request)
              {
                  // version 1
                  // if (request->hasParam("height", true))
                  // {
                  //     String height = request->getParam("height", true)->value();
                  //     // Do something with height
                  //     request->send(200, "text/plain", "Height set");
                  // }
                  // else
                  // {
                  //     request->send(400, "text/plain", "Height parameter missing or A height adjustment operation is goin on, please try later");
                  // }
                  // version 2
                  // bool AsyncWebServerRequest::hasParam(const char *name, bool post = false, bool file = false) const
                  if (request->hasParam("height", true))
                  {
                      if (currentOperation == NOOP) // Assuming currentOperation indicates if an operation is ongoing
                      {
                          String heightStr = request->getParam("height", true)->value();
                          int height = heightStr.toInt(); // Convert height to integer
                          setHeight(height);              // Call the setHeight function with the new height
                          request->send(200, "text/plain", "Height set to " + heightStr);
                      }
                      else
                      {
                          // If an operation is ongoing, inform the client
                          request->send(409, "text/plain", "A height adjustment operation is going on, please try later");
                      }
                  }
                  else
                  {
                      // If the height parameter is missing, inform the client
                      request->send(400, "text/plain", "Height parameter missing");
                  }
                  //   original code
                  // String message;
                  // // Stop all ongoing operations
                  // if (currentOperation != NOOP)
                  // {
                  //     // 409 is conflict status code
                  //     request->send(409, "text/plain", "A height adjustment operation is goin on, please try later");
                  // }
                  // else
                  // {
                  //     message = server.arg(0);
                  //     setHeight(atoi(message.c_str()));
                  //     request->send(200, "text/plain", message);
                  // }
              });
    // end RT11 based information

    server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *request)
              {
        gyro_x = 0;
        gyro_y = 0;
        gyro_z = 0;
        request->send(200, "text/plain", "OK"); });

    server.on("/resetX", HTTP_GET, [](AsyncWebServerRequest *request)
              {
        gyro_x = 0;
        request->send(200, "text/plain", "OK"); });

    server.on("/resetY", HTTP_GET, [](AsyncWebServerRequest *request)
              {
        gyro_y = 0;
        request->send(200, "text/plain", "OK"); });

    server.on("/resetZ", HTTP_GET, [](AsyncWebServerRequest *request)
              {
        gyro_z = 0;
        request->send(200, "text/plain", "OK"); });

    // Handle Web Server Events
    // inline void AsyncEventSource::onConnect(ArEventHandlerFunction cb)
    events.onConnect([](AsyncEventSourceClient *client)
                     {
        if (client->lastId())
        {
            Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
        }
        // send event with message "hello!", id current millis
        // and set reconnect delay to 1 second
        client->send("hello!", NULL, millis(), 10000); });
    // AsyncWebHandler &AsyncWebServer::addHandler(AsyncWebHandler *handler)
    server.addHandler(&events);

    // this part is given by GitHub Co-pilot
    server.on("/post", HTTP_POST, [](AsyncWebServerRequest *request)
              {
        int params = request->params();
        for (int i = 0; i < params; i++)
        {
            const AsyncWebParameter *p = request->getParam(i);
            if (p->isFile())
            {
                Serial.printf("POST[%s]: %s, size: %u\n", p->name().c_str(), p->value().c_str(), p->size());
            }
            else if (p->isPost())
            {
                Serial.printf("POST[%s]: %s\n", p->name().c_str(), p->value().c_str());
            }
            else
            {
                Serial.printf("GET[%s]: %s\n", p->name().c_str(), p->value().c_str());
            }
        }
        request->send(200, "text/plain", "Data received"); });
    // end of part given by GitHub Co-pilot

    server.begin();
    Serial.println("HTTP server started");
}; // end of webServer function
// ================

// ================
// this code is not tested yet
// the preferences library is part of the arduino-ESP32 library
// #include <Preferences.h>
//
// Preferences preferences;
// ================

// ================
// function to identify the I2C devices
// ================
void identifyI2Cdevices(void)
{
    byte error, address;
    int nDevices;

    Serial.println("Scanning...");

    nDevices = 0;
    for (address = 1; address < 127; address++)
    {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0)
        {
            Serial.print("I2C device found at address 0x");
            if (address < 16)
            {
                Serial.print("0");
            }
            Serial.print(address, HEX);
            Serial.println("  !");

            nDevices++;
        }
        else if (error == 4)
        {
            Serial.print("Unknown error at address 0x");
            if (address < 16)
            {
                Serial.print("0");
            }
            Serial.println(address, HEX);
        }
    }

    if (nDevices == 0)
    {
        Serial.println("No I2C devices found\n");
    }
    else
    {
        Serial.println("done\n");
    }

    // for information only on this particular project
    // Serial.println("OLED Display should be found at address 0x3C\n");
    // Serial.println("MPU-6050 should be found at address 0x68\n");
    // Serial.println("SD Card Reader should be found at address 0x76\n");

    Serial.println("End of I2C scan\n");
}; // end of the identifyI2Cdevices function
// ================

// ================
// End of configuration constants.
// ================

// ================================================================
// ===                   FUNCTIONS THAT WORK                    ===
// ================================================================
/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-microsd-card-arduino/

  This sketch can be found at: Examples > SD(esp32) > SD_Test
*/

// these functions listed below are using SD, FS, and SPI libraries
// #include "FS.h"
// #include "SD.h"

void listDir(fs::FS &fs, const char *dirname, uint8_t levels)
{
    Serial.printf("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if (!root)
    {
        Serial.println("Failed to open directory");
        return; // exit the function
    }
    if (!root.isDirectory())
    {
        Serial.println("Not a directory");
        return; // exit the function
    }

    File file = root.openNextFile();
    // std::vector<String> files; // original line
    std::vector<String> entries;
    while (file)
    {
        String entry;
        if (file.isDirectory())
        {
            Serial.print("  DIR : ");
            Serial.println(file.name());
            entry = "DIR: " + String(file.name());
            if (levels)
            {
                // String subDir = String(file.name()) + "/"; // added this line to fix the error of not having a "/" at the end of the directory name
                // listDir(fs, file.name(), levels - 1); // original line
                // listDir(fs, subDir.c_str(), levels - 1); // added this line to fix the error of not having a "/" at the end of the directory name
                String subDir = "/" + String(file.name());
                // listDir(fs, file.name(), levels - 1);
                listDir(fs, subDir.c_str(), levels - 1);
            }
        }
        else
        {
            // files.push_back(String(file.name()) + " - " + String(file.size()) + " bytes"); // original line
            entry = "FILE: " + String(file.name()) + " - " + String(file.size()) + " bytes";
        }
        entries.push_back(entry);
        file = root.openNextFile();
    }
    // Sort the files
    // std::sort(files.begin(), files.end()); // original line
    std::sort(entries.begin(), entries.end());

    // Print the sorted files
    // for (const String &file : files) // original line
    for (const String &entry : entries)
    {
        // Serial.println(file);
        Serial.println(entry);
    }
}; // end listDir function

// original listDir function that can be used to list the files in the SD card on the root directory only
// void listDir2(fs::FS &fs, const char *dirname, uint8_t levels)
// {
//     Serial.printf("Listing directory: %s\n", dirname);
//
//     File root = fs.open(dirname);
//     if (!root)
//     {
//         Serial.println("Failed to open directory");
//         return; // exit the function
//     }
//     if (!root.isDirectory())
//     {
//         Serial.println("Not a directory");
//         return; // exit the function
//     }
//
//     File file = root.openNextFile();
//     while (file)
//     {
//         if (file.isDirectory())
//         {
//             Serial.print("  DIR : ");
//             Serial.println(file.name());
//             if (levels)
//             {
//                 listDir(fs, file.name(), levels - 1);
//             }
//         }
//         else
//         {
//             Serial.print("  FILE: ");
//             Serial.print(file.name());
//             Serial.print("  SIZE: ");
//             Serial.println(file.size());
//         }
//         file = root.openNextFile();
//     }
// };

// void createDir(fs::FS &fs, const char *path)
// {
//     Serial.printf("Creating Dir: %s\n", path);
//     if (fs.mkdir(path))
//     {
//         Serial.println("Dir created");
//     }
//     else
//     {
//         Serial.println("mkdir failed");
//     }
// };

// void removeDir(fs::FS &fs, const char *path)
// {
//     Serial.printf("Removing Dir: %s\n", path);
//     if (fs.rmdir(path))
//     {
//         Serial.println("Dir removed");
//     }
//     else
//     {
//         Serial.println("rmdir failed");
//     }
// };

// void readFile(fs::FS &fs, const char *path)
// {
//     Serial.printf("Reading file: %s\n", path);
//
//     File file = fs.open(path);
//     if (!file)
//     {
//         Serial.println("Failed to open file for reading");
//         return; // exit the function
//     }
//
//     Serial.print("Read from file: ");
//     while (file.available())
//     {
//         Serial.write(file.read());
//     }
//     file.close();
// };

// void writeFile(fs::FS &fs, const char *path, const char *message)
// {
//     Serial.printf("Writing file: %s\n", path);
//
//     File file = fs.open(path, FILE_WRITE);
//     if (!file)
//     {
//         Serial.println("Failed to open file for writing");
//         return; // exit the function
//     }
//     if (file.print(message))
//     {
//         Serial.println("File written");
//     }
//     else
//     {
//         Serial.println("Write failed");
//     }
//     file.close();
// };

// void appendFile(fs::FS &fs, const char *path, const char *message)
// {
//     Serial.printf("Appending to file: %s\n", path);
//
//     File file = fs.open(path, FILE_APPEND);
//     if (!file)
//     {
//         Serial.println("Failed to open file for appending");
//         return; // exit the function
//     }
//     if (file.print(message))
//     {
//         Serial.println("Message appended");
//     }
//     else
//     {
//         Serial.println("Append failed");
//     }
//     file.close();
// };

// void renameFile(fs::FS &fs, const char *path1, const char *path2)
// {
//     Serial.printf("Renaming file %s to %s\n", path1, path2);
//     if (fs.rename(path1, path2))
//     {
//         Serial.println("File renamed");
//     }
//     else
//     {
//         Serial.println("Rename failed");
//     }
// };

// void deleteFile(fs::FS &fs, const char *path)
// {
//     Serial.printf("Deleting file: %s\n", path);
//     if (fs.remove(path))
//     {
//         Serial.println("File deleted");
//     }
//     else
//     {
//         Serial.println("Delete failed");
//     }
// };

// this function will display the write speeds and read speeds of the SD card
void testFileIO(fs::FS &fs, const char *path)
{
    File file = fs.open(path);
    static uint8_t buf[512];
    size_t len = 0;
    uint32_t start = millis();
    uint32_t end = start;
    if (file)
    {
        len = file.size();
        size_t flen = len;
        start = millis();
        while (len)
        {
            size_t toRead = len;
            if (toRead > 512)
            {
                toRead = 512;
            }
            file.read(buf, toRead);
            len -= toRead;
        }
        end = millis() - start;
        Serial.printf("%u bytes read for %u ms\n", flen, end);
        file.close();
    }
    else
    {
        Serial.println("Failed to open file for reading");
    }

    file = fs.open(path, FILE_WRITE);
    if (!file)
    {
        Serial.println("Failed to open file for writing");
        return; // exit the function
    }

    size_t i;
    start = millis();
    for (i = 0; i < 2048; i++)
    {
        file.write(buf, 512);
    }
    end = millis() - start;
    Serial.printf("%u bytes written for %u ms\n", 2048 * 512, end);
    file.close();
};
// ================

// ================================================================
// ===                       MAIN SETUP                         ===
// ================================================================
void setup(void)
{
    // initialize serial communication
    Serial.begin(115200);
    while (!Serial)
    {
        ; // stay in forever loop until the serial monitor is opened
    }
    Serial.println("Serial communication initialized");

    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

    // ================
    esp_reset_reason_t reason = esp_reset_reason(); // used for testing purposes only and to debug
    Serial.printf("Reset Reason: %d\n", reason);    // used for testing purposes only and to debug
    /**
     * Reset reasons
     */
    // esp_reset_reason_t typedef enum {
    //     ESP_RST_UNKNOWN,    // = 0 => !< Reset reason can not be determined
    //     ESP_RST_POWERON,    // = 1 => !< Reset due to power-on event
    //     ESP_RST_EXT,        // = 2 => !< Reset by external pin (not applicable for ESP32)
    //     ESP_RST_SW,         // = 3 => !< Software reset via esp_restart
    //     ESP_RST_PANIC,      // = 4 => !< Software reset due to exception/panic
    //     ESP_RST_INT_WDT,    // = 5 => !< Reset (software or hardware) due to interrupt watchdog
    //     ESP_RST_TASK_WDT,   // = 6 => !< Reset due to task watchdog
    //     ESP_RST_WDT,        // = 7 => !< Reset due to other watchdogs
    //     ESP_RST_DEEPSLEEP,  // = 8 => !< Reset after exiting deep sleep mode
    //     ESP_RST_BROWNOUT,   // = 9 => !< Brownout reset (software or hardware)
    //     ESP_RST_SDIO,       // = 10 => !< Reset over SDIO
    // }
    // ================

    // ================
    pinMode(LED_BUILTIN, OUTPUT); // this is the LED on the ESP32 board as output
    // ================

    // ================
    // Set relay pins as outputs
    // information is located in LinearActuator.h
    // const int relay1_LA1LHP_Pin PROGMEM = 27;
    // const int relay2_LA1LHM_Pin PROGMEM = 26;
    // const int relay3_LA2RHP_Pin PROGMEM = 25;
    // const int relay4_LA2RHM_Pin PROGMEM = 33;
    pinMode(relay1_LA1LHP_Pin, OUTPUT);
    pinMode(relay2_LA1LHM_Pin, OUTPUT);
    pinMode(relay3_LA2RHP_Pin, OUTPUT);
    pinMode(relay4_LA2RHM_Pin, OUTPUT);
    // ================

    // ================
    // Serial.println("Starting relay activation"); // used for testing purposes only
    // Set initial relay states (HIGH to deactivate all relays)
    digitalWrite(relay1_LA1LHP_Pin, HIGH);
    digitalWrite(relay2_LA1LHM_Pin, HIGH);
    digitalWrite(relay3_LA2RHP_Pin, HIGH);
    digitalWrite(relay4_LA2RHM_Pin, HIGH);
    // relays should not engage at all
    // ================

    // ================
    // Set up the interrupt pin, and configure the interrupt to be triggered on a rising edge
    // pinMode(INTERRUPT_PIN, INPUT_PULLUP);
    // attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), isr, RISING);
    // ================

    // ================
    // pins for ultrasonic sensor
    // information is found in ultrasonic.h
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    // ================

    // ================
    // initialize the LED strip
    pinMode(LED_PIN0, OUTPUT); // This is the Test Bench LEDs as output
    pinMode(LED_PIN1, OUTPUT); // This is the Stand Up Desk LEDs as output
    pinMode(LED_PIN2, OUTPUT); // This is the Storage bench LEDS as output

    // Set up the data pin and LED arrays
    FastLED.addLeds<LED_TYPE, LED_PIN0, COLOR_ORDER>(led_Strip0, NUM_LED_PIN0).setCorrection(TypicalLEDStrip);
    FastLED.addLeds<LED_TYPE, LED_PIN1, COLOR_ORDER>(led_Strip1, NUM_LED_PIN1).setCorrection(TypicalLEDStrip);
    FastLED.addLeds<LED_TYPE, LED_PIN2, COLOR_ORDER>(led_Strip2, NUM_LED_PIN2).setCorrection(TypicalLEDStrip);
    // Setup LEDs brightness and ensure that they start not lit (off)
    FastLED.setBrightness(BRIGHTNESS);
    FastLED.clear();
    // ================

    // ================
    // to test the LEDs
    Serial.println("Starting LED Strip test");
    ledStripTest(led_Strip0, NUM_LED_PIN0); // this works
    ledStripTest(led_Strip1, NUM_LED_PIN1); // this works
    ledStripTest(led_Strip2, NUM_LED_PIN2); // this works

    FastLED.clear();

    // used for testing purposes only
    // testing the balance of the table with LED display
    // Serial.println("Testing the level LED Strip");
    // for (int i = 0; i < 2; i++)
    // {
    //     testLEDsLevel(); // used for testing purposes only
    //     delay(500);        // used for testing purposes only
    // }
    // Serial.println("Testing Level LEDs"); // used for testing purposes only
    // for (int i = 0; i < 3; i++)
    // {
    //     setLEDsLevel(); // used for testing purposes only
    //     delay(500);        // used for testing purposes only
    // }
    // Serial.println("Testing LEDs Tilted Left"); // used for testing purposes only
    // for (int i = 0; i < 3; i++)
    // {
    //     setLEDsTiltedLeft(1); // used for testing purposes only
    //     delay(500);           // used for testing purposes only
    // }
    // Serial.println("Testing LEDs More Tilted Left"); // used for testing purposes only
    // for (int i = 0; i < 3; i++)
    // {
    //     setLEDsTiltedLeft(2); // used for testing purposes only
    //     delay(500);           // used for testing purposes only
    // }
    // Serial.println("Testing LEDs Tilted Right"); // used for testing purposes only
    // for (int i = 0; i < 3; i++)
    // {
    //     setLEDsTiltedRight(1); // used for testing purposes only
    //     delay(500);    // used for testing purposes only
    // }
    // Serial.println("Testing LEDs More Tilted Right"); // used for testing purposes only
    // for (int i = 0; i < 3; i++)
    // {
    //     setLEDsTiltedRight(2); // used for testing purposes only
    //     delay(500);           // used for testing purposes only
    // }
    // Serial.println("Testing LEDs Tilted Forwards"); // used for testing purposes only
    // for (int i = 0; i < 3; i++)
    // {
    //     setLEDsTiltedForward(1); // used for testing purposes only
    //     delay(500);           // used for testing purposes only
    // }
    // Serial.println("Testing LEDs More Tilted Forwards"); // used for testing purposes only
    // for (int i = 0; i < 3; i++)
    // {
    //     setLEDsTiltedForward(2); // used for testing purposes only
    //     delay(500);           // used for testing purposes only
    // }
    // Serial.println("Testing LEDs Tilted Backwards"); // used for testing purposes only
    // for (int i = 0; i < 3; i++)
    // {
    //     setLEDsTiltedBackward(1); // used for testing purposes only
    //     delay(500);           // used for testing purposes only
    // }
    // Serial.println("Testing LEDs More Tilted Backwards"); // used for testing purposes only
    // for (int i = 0; i < 3; i++)
    // {
    //     setLEDsTiltedBackward(2); // used for testing purposes only
    //     delay(500);           // used for testing purposes only
    // }
    // FastLED.clear();
    // ================

    // ================
    // RT-11 remote/handset pins

    pinMode(DOWN_PIN, OUTPUT); // since DOWN_PIN = 15 = GPIO 15 // need to listen to whatever the remote is sending
    pinMode(UP_PIN, OUTPUT);   // since UP_PIN = 4 = GPIO 4 // need to listen to whatever the remote is sending

    digitalWrite(DOWN_PIN, DEACTIVATE); // simulate that the down button is not pressed at startup
    digitalWrite(UP_PIN, DEACTIVATE);   // simulate that the up button is not pressed at startup

    // Disable pullups turned on by espSoftwareSerial library

    // pinMode(16, INPUT); // GPIO 16 // GPIO 16 is pin 26 on the ESP32 DOIT and set for Rx since the Tx of the RT11 is connected to pin 26
    // pinMode(26, INPUT); // Pin 26 // Pin 26 is the Tx of the RT11 which means that GPIO 16 on the ESP32 DOIT is set for Tx
    // pinMode(17, OUTPUT); // GPIO 17 // GPIO 17 is pin 25 on the ESP32 DOIT is set for Tx since the Rx of the RT11 is connected to pin 25
    // pinMode(25, INPUT); // Pin 25 // Pin 25 is the Rx of the RT11 which means that GPIO 17 on the ESP32 DOIT is set for Rx

    // pinMode(5 , INPUT); // GPIO 5 // original code, pin 5 was the Tx of the RT11 which means that pin 5 on the ESP8266 was set for Rx
    // pinMode(3, INPUT); // GPIO 3 is the ESP32 Rx pin connected to the Tx pin of the RT11
    // pinMode(1, OUTPUT); // GPIO 1 is the ESP32 Tx pin connected to the Rx pin of the RT11

    // disabled:
    // pinMode(led, OUTPUT); // led = 2 = GPIO 2 = LED_BUILTIN // already defined above
    //
    // stopMoving(); // Make sure there is no movement on startup // not needed here as I will lower the table to the lowest position first
    //
    // digitalWrite(led, 0); // do not need to turn off the LED as it is already off // original code
    //
    // RT-11 remote/handset setup
    // for use in SoftwareSerial only
    // deskSerial.begin(9600); // The controller uses 9600 bauds for serial communication

    // Initialize UART2 (deskSerial2) on GPIO 16 (RX) and GPIO 17 (TX)
    // void HardwareSerial::begin(unsigned long baud, uint32_t config = 134217756U, int8_t rxPin = (int8_t)(-1), int8_t txPin = (int8_t)(-1), bool invert = false, unsigned long timeout_ms = 20000UL, uint8_t rxfifo_full_thrhd = (uint8_t)112U)
    deskSerial.begin(9600, SERIAL_8N1, uart2rxPin, uart2txPin); // GPIO 16 (RX) and GPIO 17 (TX) for UART 2// pins 25 and 24 on the ESP32 DOIT DevKit V1
    // from esp32-hal-uart.h
    // enum SerialConfig {SERIAL_5N1 = 0x8000010, SERIAL_6N1 = 0x8000014, SERIAL_7N1 = 0x8000018, SERIAL_8N1 = 0x800001c, SERIAL_5N2 = 0x8000030, SERIAL_6N2 = 0x8000034,
    // SERIAL_7N2 = 0x8000038, SERIAL_8N2 = 0x800003c, SERIAL_5E1 = 0x8000012, SERIAL_6E1 = 0x8000016, SERIAL_7E1 = 0x800001a, SERIAL_8E1 = 0x800001e, SERIAL_5E2 = 0x8000032,
    // SERIAL_6E2 = 0x8000036, SERIAL_7E2 = 0x800003a, SERIAL_8E2 = 0x800003e, SERIAL_5O1 = 0x8000013, SERIAL_6O1 = 0x8000017, SERIAL_7O1 = 0x800001b, SERIAL_8O1 = 0x800001f,
    // SERIAL_5O2 = 0x8000033, SERIAL_6O2 = 0x8000037, SERIAL_7O2 = 0x800003b, SERIAL_8O2 = 0x800003f };
    // SERIAL_8N1 = 0x800001c = 134217756U??

    Serial.println("UART2 initialized");

    // added this little bit to see if the RT-11 is working

    if (deskSerial.available())
    {
        Serial.println("RT-11 is working");
        int height = 0;
        setHeight(height);
        decodeSerial();
        rt11Working = true;
    }
    else
    {
        Serial.println("RT-11 is not working");
        rt11Working = false;
        
        // decodeSerial(); // this code works and does demonstrate that information is being released from the RT-11
        
        // for (;;)
        // {
        //     // don't proceed, loop forever
        // }
    }

    // if (deskSerial.available() >= 4) { // Check if at least 4 bytes are available
    //     byte data[4];
    //     for (int i = 0; i < 4; i++) {
    //         data[i] = deskSerial.read(); // Read each byte
    //     }
    //     // Process the 4 bytes (32 bits) as needed
    //     Serial.print("Received 32 bits: ");
    //     for (int i = 0; i < 4; i++) {
    //         Serial.print(data[i], HEX);
    //         Serial.print(" ");
    //     }
    //     Serial.println();
    //     rt11Working = true;
    // }
    // else {
    //     Serial.println("RT-11 is not working");
    //     rt11Working = false;
    //     for (;;)
    //     {
    //         // don't proceed, loop forever
    //     }
    // }
    //
    // deskSerial.println("Test");
    //
    // Wait for a response within a timeout period
    // unsigned long startTime = millis();
    // bool responseReceived = false;
    // while (millis() - startTime < 1000) { // 1 second timeout
    //     if (deskSerial.available()) {
    //         rt11Working = true;
    //         decodeSerial(); // decode the serial data
    //         // String data = deskSerial.readString();
    //         // if (data.length() > 0) {
    //         //     responseReceived = true;
    //         //     Serial.println("Response from RT-11: " + data);
    //         //     break; // exit the loop
    //         // }
    //     }
    // }
    // if (!responseReceived) {
    //     rt11Working = false;
    //     Serial.println("No response from RT-11.");
    //     for (;;)
    //     {
    //         // don't proceed, loop forever
    //     }
    // }
    //
    // original code
    // if (deskSerial.available())
    // {
    //     deskSerial.println("RT-11 is working");
    //     rt11Working = true;
    // }
    // else
    // {
    //     deskSerial.println("RT-11 is not working");
    //     rt11Working = false;
    // }
    // ================

    // ================
    // from SimpleTime.ino example
    // link to the example in datalogger.h
    // connect to WiFi
    Serial.printf("Connecting to %s ", ssid);
    // added this function to connect to the WiFi
    // ================
    // commented out for testing purposes only
    initWiFi(); // initializes the WiFi connection // from ESP32-MPU-6050-Web-Server
    // ================
    //
    WiFi.begin(ssid, password);
    //
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println(" CONNECTED");
    //
    // set notification call-back function
    sntp_set_time_sync_notification_cb(timeavailable);
    /**
     * NTP server address could be aquired via DHCP,
     *
     * NOTE: This call should be made BEFORE esp32 aquires IP address via DHCP,
     * otherwise SNTP option 42 would be rejected by default.
     * NOTE: configTime() function call if made AFTER DHCP-client run
     * will OVERRIDE aquired NTP server address
     */
    // ================
    // commented out for testing purposes only
    sntp_servermode_dhcp(1); // (optional) set NTP server address acquisition mode to DHCP
    // ================
    //
    /**
     * This will set configured ntp servers and constant TimeZone/daylightOffset
     * should be OK if your time zone does not need to adjust daylightOffset twice a year,
     * in such a case time adjustment won't be handled automagicaly.
     */
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer1, ntpServer2);
    //
    /**
     * A more convenient approach to handle TimeZones with daylightOffset
     * would be to specify a environmnet variable with TimeZone definition including daylight adjustmnet rules.
     * A list of rules for your zone could be obtained from https://github.com/esp8266/Arduino/blob/master/cores/esp8266/TZ.h
     */
    configTzTime(time_zone, ntpServer1, ntpServer2);

    webServer(); // initializes the webServer layout // from ESP32-MPU-6050-Web-Server
    // ================

#ifdef USE_LIBRARY2
    // initialize devices
    Serial.println("Initializing I2C devices..."); // used for testing purposes only

    // verify connection
    // Serial.println("Testing device connections..."); // used for testing purposes only

    // ================
    // setup for the OLED display
    if (!u8g2.begin())
    {
        Serial.println(F("SSD1306 allocation failed! using the U8G2 library"));
        for (;;)
        {
            // don't proceed, loop forever
        }
    };

    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_I2C_ADDR))
    {
        Serial.println(F("SSD1306 allocation failed! using the Adafruit library"));
        for (;;)
        {
            // don't proceed, loop forever
        }
    };

    u8g2.begin();
    u8g2.clear();
    u8g2.setFont(u8g2_font_profont10_tf);
    oled_LineH = u8g2.getFontAscent() - u8g2.getFontDescent();
    display.clearDisplay();

    // to test the OLED Screen
    Serial.println("OLED TESTING");
    // infinite loop to test the OLED screen
    // for (;;)
    // {
    //     DrawLinesAndGraphicsFrame();
    // };

    // used for testing purposes only
    for (int v = 0; v < 100; v++)
    {
        if (v ? (v % 5 == 0) : (v % 5 == 1))
        {
            Serial.println(v);
        }
        DrawLinesAndGraphicsFrame();
    }

    Serial.println("OLED Display Setup Complete"); // used for testing purposes only
    // ================

    // ================
    MPU2.initialize();
    Serial.print("MPU Connection ");
    Serial.println(MPU2.testConnection() ? "successful" : "failed");
    // ================

    // ================
    // for testing purposes only
    // to identify all the I2C connected devices
    // identifyI2Cdevices();
    // this functions works and it will display the devices connected to the I2C bus
    // ================

    // setup for the SD card reader with libraries SD, FS, and SPI
    // ================
    if (!SD.begin(CS))
    {
        Serial.println("Card Mount Failed");
        return; // exit the function
    }
    uint8_t cardType = SD.cardType();

    if (cardType == CARD_NONE)
    {
        Serial.println("No SD card attached");
        return; // exit the function
    }

    Serial.print("SD Card Type: ");
    if (cardType == CARD_MMC)
    {
        Serial.println("MMC");
    }
    else if (cardType == CARD_SD)
    {
        Serial.println("SDSC");
    }
    else if (cardType == CARD_SDHC)
    {
        Serial.println("SDHC");
    }
    else
    {
        Serial.println("UNKNOWN");
    }

    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("SD Card Size: %lluMB\n", cardSize);
    Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));

    // function calling list using functions either active or commented out above
    // listDir(SD, "/", 0); // original line
    // listDir2(SD, "/", 0);
    // listDir(SD, root, 1); // works now // this would be my default to see if all the files are registering
    // createDir(SD, "/mydir");
    // listDir(SD, "/", 0);
    // removeDir(SD, "/mydir");
    // listDir(SD, "/", 2);
    // writeFile(SD, "/hello.txt", "Hello ");
    // appendFile(SD, "/hello.txt", "World!\n");
    // readFile(SD, "/hello.txt");
    // deleteFile(SD, "/foo.txt");
    // renameFile(SD, "/hello.txt", "/foo.txt");
    // readFile(SD, "/foo.txt");
    testFileIO(SD, "/test.txt"); // this will display the speed at which the test file is written and read
    // this test displays the write speed as approx 2.8 secs and read as approx 2.4 secs
    // ================

    // ================
    // DATA LOGGING SETUP
    // Now you can open the file with the unique filename
    // Check if the file exists and if so, increment the file number
    if (SD.exists("/data.csv"))
    {
        // Serial.println("data.csv exists");
        while (SD.exists(String(baseName) + String(fileNumber) + String(extension)))
        {
            fileNumber++;
        }
        fileName = String(baseName) + String(fileNumber) + String(extension);
        dataFile = SD.open(fileName, FILE_WRITE); // Use the global dataFile variable
        if (!dataFile)
        {
            Serial.println("Error opening " + fileName);
            return; // exit the function
        }
        else
        {
            Serial.println("New File " + fileName + " created successfully"); // used for testing purposes only
            dataFile.close();
            // Serial.println("File now closed"); // used for testing purposes only
        }
    }
    else
    {
        // Serial.println("data.csv does not exist");
        dataFile = SD.open("/data.csv", FILE_WRITE); // Use the global dataFile variable
        if (!dataFile)
        {
            Serial.println("Error opening data.csv");
            return; // exit the function
        }
        else
        {
            // Serial.println("File opened successfully"); // used for testing purposes only
            dataFile.close();
            // Serial.println("data.csv now closed"); // used for testing purposes only
        }
    }
    logDataHeader(); // this function is used to create log the header information to the SD card when data logging
    // ================

    // ================
    IMUInitialization(); // this function is used to initialize the MPU6050 and possibly calibrate depending in the function data
    delay(1000);         // delay 1 second to stabilize the MPU

    // ================
    // this code is not tested yet
    // Retrieve the offset values
    // preferences.begin("MPU_calibration", false);
    // gx_offset = preferences.getLong("gyroXOffset");
    // gy_offset = preferences.getLong("gyroYOffset");
    // gz_offset = preferences.getLong("gyroZOffset");
    // ax_offset = preferences.getLong("accelXOffset");
    // ay_offset = preferences.getLong("accelYOffset");
    // az_offset = preferences.getLong("accelZOffset");
    // preferences.end();
    // ================

    // now we have to ensure we either start at bottom out or save the information before we reinitialize
    moveToBottomOutPosition(); // moving table to bottom out position

    // Functions to test out the MPU6050 or to initialize it
    // this one works
    // dumpDataLoop(); // this function is used to dump the data from the MPU6050

    // ================
    // CONFIG FILE SETUP
    if (SD.exists("/config.csv"))
    {
        // Serial.println("config.csv exists"); // used for testing purposes only
        isCalibrationLevel = false;
        // this is one of the functions used to calibrate the MPU6050 using the calibrateData.h library
        // 1 is for initial gyro calibration
        // 2 is for saved gyro calibration
        calibratingData(2);

        if (error == 1)
        {
            Serial.println("Error in calibration. Redoing calibration");
            calibratingData(2);
        }
        else
        {
            // Serial.println("Calibration successful"); // used for testing purposes only
        }
    }
    else
    {
        // Serial.println("config.csv does not exist"); // used for testing purposes only

        configFile = SD.open("/config.csv", FILE_WRITE); // creating the file config.csv

        if (!configFile)
        {
            Serial.println("Error opening config.csv");
            return; // exit the function
        }
        else
        {
            // Serial.println("File opened successfully"); // used for testing purposes only

            configFile.close();
            configDataHeader(); // this function is used to log the header information to the SD card when data logging
            // this is one of the functions used to calibrate the MPU6050 using the calibrateData.h library
            // 1 is for initial gyro calibration
            // 2 is for saved gyro calibration
            calibratingData(1);
        }
    }
    // ================
#endif

    Serial.println("Setup complete"); // used for testing purposes only
}; // end setup function

// ================================================================
// ===                        MAIN LOOP                         ===
// ================================================================
void loop(void)
{
    Serial.println("Starting loop"); // used for testing purposes only
    currentMillis = millis();        // this is used to keep track of the current time in milliseconds

    // ================
    // if using MMC library for the SD Card, the following lines must be commented out
    // blink LED to indicate activity
    // used for testing purposes only
    // bLED = !bLED; // toggle LED State
    // digitalWrite(LED_BUILTIN, bLED);
    // ================

#ifdef USE_LIBRARY2

    // ================
    // indefinite loop for ultrasonic sensor testing
    // while (true)
    // {
    //     bLED = !bLED; // toggle LED State
    //     digitalWrite(LED_BUILTIN, bLED);
    //     // getRawDistance();
    //     displayAllGyroandDistance();
    //     delay(1000);
    // }
    // ================

    // ================
    FastLED.clear(); // clear whatever is on the LED strips whenever the loop starts and/or is restarted

    // variable for the LEDs animation
    // MUST REMAIN HERE
    startIndex = startIndex + 1; /* motion speed */
    // ================

    // ================
    // following lines are used to relay the information from the MPU6050 to the OLED screen
    // read raw accel/gyro measurements from device
    MPU2.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    //
    // used for testing purposes only
    // display tab-separated accel/gyro x/y/z values
    // Serial.print("MPU RAW Output: AccX:\t");
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
    // Serial.print("\t Temp: ");
    double temp = MPU2.getTemperature();
    tempC = (temp / 340.00) + 36.53; // convert to Celsius using MPU6050.h library
    // Serial.println(tempC);
    //
    // used for testing purposes only
    // write the data to the SD card
    // version 1
    // dataFile.print("MPU RAW Output:\nAccX:\t");
    // dataFile.print(ax);
    // dataFile.flush(); // Immediately write all data in the cache to the disk
    // dataFile.print("\tAccY:\t");
    // dataFile.print(ay);
    // dataFile.flush(); // Immediately write all data in the cache to the disk
    // dataFile.print("\tAccZ:\t");
    // dataFile.print(az);
    // dataFile.flush(); // Immediately write all data in the cache to the disk
    // dataFile.print("\tGyroX:\t");
    // dataFile.print(gx);
    // dataFile.flush(); // Immediately write all data in the cache to the disk
    // dataFile.print("\tGyroY:\t");
    // dataFile.print(gy);
    // dataFile.flush(); // Immediately write all data in the cache to the disk
    // dataFile.print("\tGyroZ:\t");
    // dataFile.print(gz);
    // dataFile.flush(); // Immediately write all data in the cache to the disk
    // dataFile.print("\tTemp:\t");
    // dataFile.println(tempCelsius);
    // dataFile.flush(); // Immediately write all data in the cache to the disk
    // if (dataFile)
    // {
    //     Serial.println("Data written to " + fileName);
    // }
    // else
    // {
    //     Serial.println("Error writing to " + fileName);
    // }
    // version 2
    // logSensorData(2); // this function is used to log the RAW SENSOR DATA to the SD card
    //
    // used for testing purposes only
    // logSensorData(3); // this function is used to log the sensor data to the SD card
    // logSensorData(7); // this function is used to log ALL the SENSOR DATA to the SD card
    // ================

    // Functions to test out the MPU6050 or to initialize it
    // this one works
    // dumpDataLoop(); // this function is used to dump the data from the MPU6050
    // ensure the Serial.print lines above are commented out as well as the getMotion6 function

    // ================
    // This if statement is used to gather data for the webpage visual representation of the MPU6050
    // delay equivalence of 10ms found in LinearActuator.h
    if ((millis() - previousMillis) > timePeriod010)
    {
        // Send Events to the Web Server with the Sensor Readings
        events.send(getMPUReadings().c_str(), "MPU_readings", millis());
        // for testing purposes only
        // String gyroData = getMPUReadings();
        // Serial.println("MPU data sent to " + WiFi.localIP().toString() + ": " + gyroData);
        previousMillis = millis();
    }
    // ================

    // ================
    // RT-11 remote/handset loop
    //
    // for testing RT-11 and serial monitor purposes only
    //     if (Serial.available()) {
    //     String input = Serial.readString();
    //     Serial.print("Received from VSCode Serial Monitor: ");
    //     Serial.println(input);
    //   }
    //
    //   if (deskSerial.available()) {
    //     String input = deskSerial.readString();
    //     deskSerial.print("Received from RT-11 remote controller: ");
    //     deskSerial.println(input);
    //   }
    // ================

    // ================
    // MAIN LOOP AREA
    if (!rt11Working)
    {
        tableCommands(); // this function is used to control the table movement
        checkTableMovement(dataChanged);
        // Reset dataChanged after checking
        dataChanged = false;
    }
    else
    {
        // identical to within the tableCommands function
        // currentMillis = millis(); // Get the current time
        if (currentMillis - previousMillis >= timePeriod100)
        {
            // Save the last time the display was updated
            previousMillis = currentMillis;

            // displayFilteredGyro(); // this function is used to display the filtered gyro information on the OLED screen
            Serial.println("Inside_Table_Commands_delay_loop"); // used for testing purposes only
            displayAllGyroandDistance();                        // this function is used to display the filtered gyro information and the distance on the OLED screen
        }

        rt11_loop();
    }
    // ================

    // ================
    // Check if it's time to execute tableCommands every 50ms
    // if (currentMillis - commandsPreviousMillis >= timePeriod050) {
    //     commandsPreviousMillis = currentMillis;
    //     tableCommands();
    // }

    // Check if it's time to execute displayAllGyroandDistance every 10ms
    // if (currentMillis - displayPreviousMillis >= timePeriod010) {
    //     displayPreviousMillis = currentMillis;
    //     // displayFilteredGyro(); // this function is used to display the filtered gyro information on the OLED screen
    //     displayAllGyroandDistance();
    // }
    // displayAllGyroandDistance(); // display the gyro and distance information on the OLED screen
    // displayFilteredGyro(); // display the gyro infomration on the OLED screen

    // checkTableMovement(dataChanged); // this function is used to check the MPU data for changes
    // if the data has not changed, then this if statement should work
    // if (!dataChanged)
    // {
    //     // If table is not moving, execute LED effect and update OLED
    //     randomizedLEDEffect();
    // }
    // ================

#endif

}; // end loop function

// =========================================================
// END OF PROGRAM
// =========================================================
