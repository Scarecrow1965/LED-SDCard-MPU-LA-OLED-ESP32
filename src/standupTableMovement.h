// +-------------------------------------------------------------
//
// Equipment:
// DOIT ESP32 Dev Kit V1, MPU-6050, OLED SSD1306, WS2812B Addressable LEDS x3,
// 4-relay module, 2 linear actuators, RT-11(JCHT35K9) remote/handset,
// and 650W 12V Power supply
//
// File: standupTableMovement.h
//
// Description:
//
// Provides the computer workstation the ability to use the remote/handset,
// MPU-6050, and the linear actuators to control the movement of the standup
// portion of the workstation.
//
// History:     20-Mar-2024     Scarecrow1965   Created
//
// +-------------------------------------------------------------

#ifndef STANDUPTABLEMOVEMENT_H
#define STANDUPTABLEMOVEMENT_H

#include <Arduino.h>
#include <Wire.h>
// #include <SPI.h>
#include <SD.h>

// from main.cpp variables
extern File dataFile;
extern String fileName;

// #include "calibrateData.h"
#include "filterData.h"

#include "LinearActuator.h"
extern bool isStop;
extern bool isTableMax;
extern bool isBottomOut;
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
extern bool dataChanged;
void boolChecker(void);

#include "dataLogger.h"

#include "ultrasonic.h"
#include "seeOLED.h"

#include "FastLED.h"
#include "LEDeffects.h"
void randomizedLEDEffect(void);

// these variables are used to put the ESP32 into a sleep mode
#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 60       /* Time ESP32 will go to sleep (in seconds) */
// ================
char command = 0;
char lastCommand = 0;
bool isCommandBeingProcessed = false;

void processCommand(char command)
{
    currentMillis = millis(); // Record the start time

    switch (command)
    {
    case 'Q':
    case 'q':
        Serial.println(F("Quitting Program"));
        // creating a timer to put the ESP32 to sleep before I shut it down
        esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
        Serial.println("Going to sleep now");
        esp_deep_sleep_start();
        // exit(0);
        break; // exit the loop

    case 'M':
    case 'm':
        Serial.println(F("Constant Gyro Monitoring"));
        while (Serial.available() == 0 && millis() - currentMillis < timePeriod5) // Check if there's no new command and the timeout hasn't been reached
        {
            filterData();          // function to display the filtered gyro information on the OLED screen
            displayFilteredGyro(); // this function is used to display the filtered gyro information on the OLED screen

            // delay for 100 ms
            currentMillis = millis();
            if (currentMillis - previousMillis >= timePeriod100)
            {
                previousMillis = currentMillis;
                // delay(100);
            }
        }
        break; // exit the loop

    case 'D':
    case 'd':
        // used for testing purposes only
        Serial.println("lastCommand_from_down_was:\t" + String(lastCommand));
        // Serial.println(bool(isStop));
        // if (isStop)
        // {
        //     Serial.println("isStop is true");
        // }
        // else
        // {
        //     Serial.println("isStop is false");
        // }
        boolChecker(); // used for testing purposes only

        Serial.println("isBottomOut: " + String(isBottomOut ? "True" : "False")); // print the value of isBottomOut

        if (!isBottomOut)
        {
            Serial.println(F("Moving Down"));
            moveDown();
        }
        else if (isBottomOut == true && isStop == true)
        {
            Serial.println("Table_at_bottom"); // used for testing purposes only
            // stopMovement(); // already called/declared in the moveDown function
            command = 'S'; // stop the table from moving
            lastCommand = command;
        }
        else if (isStop == true)
        {
            Serial.println(F("Stopping Movement"));
            command = 'S'; // stop the table from moving
            lastCommand = command;
            stopMovement();
        }
        Serial.println("lastCommand_from_down_is_now:\t" + String(lastCommand)); // used for testing purposes only
        break; // exit the loop

    case 'U':
    case 'u':
        // used for testing purposes only
        Serial.println("lastCommand_from_up_was:\t" + String(lastCommand));
        // Serial.println(bool(isTableMax));
        // if (isTableMax)
        // {
        //     Serial.println("isTableMax is true");
        // }
        // else
        // {
        //     Serial.println("isTableMax is false");
        // }
        // Serial.println(bool(isStop));
        // if (isStop)
        // {
        //     Serial.println("isStop is true");
        // }
        // else
        // {
        //     Serial.println("isStop is false");
        // }
        boolChecker();

        if (!isTableMax)
        {
            Serial.println(F("Moving Up"));
            moveUp();
        }
        else if (isTableMax)
        {
            Serial.println("Table_at_MAX_Height"); // used for testing purposes only
            Serial.println(F("Stopping Movement"));
            // stopMovement(); // already called/declared in the moveUp function
            command = 'S'; // stop the table from moving
            lastCommand = command;
        }

        Serial.println("lastCommand_from_up_is_now:\t" + String(lastCommand)); // used for testing purposes only
        break; // exit the loop

    case 'S':
    case 's':
        // used for testing purposes only  
        // Serial.println("old oldCommand: " + String(oldCommand));
        // Serial.println("old lastCommand: " + String(lastCommand));
        // oldCommand = lastCommand;
        // Serial.println("new oldCommand: " + String(oldCommand));
        // Serial.println("new lastCommand: " + String(lastCommand));
        Serial.println("from_stop,_new_command:\t" + String(command)); // used for testing purposes only
        Serial.println("lastCommand_from_stop_was:\t" + String(lastCommand)); // used for testing purposes only
        boolChecker(); // used for testing purposes only

        if (lastCommand == 'S' || lastCommand == 's')
        {
            Serial.println(F("Already Stopped"));
            // run the LEDs
            randomizedLEDEffect(); // time to run the LED effects // from LEDeffects.h
            // 
        }
        else
        {
            Serial.println(F("Stopping Movement"));
            // stop the LEDs
            FastLED.clear(); // clears the position of the table // from LEDeffects.h
            // 
            stopMovement();
        }
        // lastCommand = command; // save the last command
        Serial.println("lastCommand_from_stop_is_now:\t" + String(lastCommand)); // used for testing purposes only

        // original lines of code commented out
        // Serial.println(F("Stopping Movement"));
        // stopMovement();
        break; // exit the loop

    default:
        Serial.println(F("Invalid Command"));
        break; // exit the loop
    }

}; // end process command function

void tableCommands(void)
{
    Serial.println(F("\nSend 'U'/'u' for Up, 'D'/'d' for Down, 'S'/'s' for STOP \n'Q'/'q' to Quit Program, 'M'/'m' for Constant Gyro Monitoring.\n"));
    bool isFirstStart = true;
    dataChanged = false;

    while (true)
    {
        // delay for 100ms and display on OLED
        currentMillis = millis(); // Get the current time
        if (currentMillis - previousMillis >= timePeriod100)
        {
            // Save the last time the display was updated
            previousMillis = currentMillis;

            // displayFilteredGyro(); // this function is used to display the filtered gyro information on the OLED screen
            Serial.println("Inside_Table_Commands_delay_loop"); // used for testing purposes only
            displayAllGyroandDistance();                        // this function is used to display the filtered gyro information and the distance on the OLED screen
        }

        // Check if a command is available from the Serial Monitor
        if (Serial.available())
        {
            char command = Serial.read();

            // Clear the serial buffer
            // if (command == '\n' || command == '\r')
            // {
            //     // Skip the rest of the loop and continue to the next iteration
            //     return; // Exit the function
            // }

            // while (Serial.available() > 0)
            // {
            //     Serial.read();
            // }

            // // Consume any additional characters from the Serial buffer
            while (Serial.available())
            {
                Serial.read();
            }

            // Debug output to display the received command
            Serial.print(F("Received Command: "));
            Serial.println(command);

            // Save the command to the data file
            String commandString;
            if (command == 'D' || command == 'd')
            {
                commandString = "Moving Down";
                dataChanged = true;
            }
            else if (command == 'U' || command == 'u')
            {
                commandString = "Moving Up";
                dataChanged = true;
            }
            else if (command == 'S' || command == 's')
            {
                commandString = "Stopping Movement";
                dataChanged = true;
            }
            else if (command == 'Q' || command == 'q')
            {
                commandString = "Quitting Program";
            }
            else if (command == 'M' || command == 'm')
            {
                commandString = "Constant Gyro Monitoring";
            }
            else
            {
                commandString = "Invalid Command";
            }
            // Save the command to the data file
            File dataFile = SD.open(fileName, FILE_APPEND);
            dataFile.println("Command: " + commandString);
            dataFile.flush(); // Immediately write all data in the cache to the disk
            dataFile.close();

            // Process the command
            isCommandBeingProcessed = true; // Set the flag to true before processing the command
            processCommand(command);
            isCommandBeingProcessed = false; // Set the flag back to false after the command is processed
            
            // Save the command for later use
            lastCommand = command;
            Serial.println("command processed."); // used for testing purposes only
        }
        else
        {
            // If no new command is received, use the last command
            if (lastCommand != 0)
            {
                Serial.print(F("Repeating last command: "));
                Serial.println(lastCommand);

                // Save the command to the data file
                String lastCommandString;
                if (lastCommand == 'D' || lastCommand == 'd')
                {
                    lastCommandString = "Repeating Moving Down";
                    dataChanged = true;
                }
                else if (lastCommand == 'U' || lastCommand == 'u')
                {
                    lastCommandString = "Repeating Moving Up";
                    dataChanged = true;
                }
                else if (lastCommand == 'S' || lastCommand == 's')
                {
                    lastCommandString = "Repeating Stopping Movement";
                }
                else if (lastCommand == 'Q' || lastCommand == 'q')
                {
                    lastCommandString = "Repeating Quitting Program";
                }
                else if (lastCommand == 'M' || lastCommand == 'm')
                {
                    lastCommandString = "Repeating Constant Gyro Monitoring";
                }
                else
                {
                    lastCommandString = "Invalid Command again";
                }
                // Save the command to the data file
                File dataFile = SD.open(fileName, FILE_APPEND);
                dataFile.println("Last Command: " + lastCommandString);
                dataFile.flush(); // Immediately write all data in the cache to the disk
                dataFile.close();

                // Process the last command
                isCommandBeingProcessed = true; // Set the flag to true before processing the command
                processCommand(lastCommand);
                isCommandBeingProcessed = false; // Set the flag back to false after the command is processed

                Serial.println("last command processed."); // used for testing purposes only
            }
            else
            {
                Serial.println(F("No command received"));
                Serial.println("command:\t" + (command == 0 ? String("blank") : String(command))); // used for testing purposes only
                Serial.println("lastCommand:\t" + (lastCommand == 0 ? String("blank") : String(lastCommand))); // used for testing purposes only
                // time to turn on the LEDs since there is nothing going on
                randomizedLEDEffect(); // time to run the LED effects // from LEDeffects.h
                // 

                if (isFirstStart)
                {
                    Serial.println("First_Start_stop_and_leveling"); // used for testing purposes only
                    stopMovement();
                    checkTableAfterStop();
                    isFirstStart = false;
                    isStop = false;
                }
                else
                {
                    Serial.println("Still_no_commands_received"); // used for testing purposes only
                }
            }
        }
    }
}; // end table commands function

#endif // STANDUPTABLEMOVEMENT_H

// =========================================================
// END OF PROGRAM
// =========================================================