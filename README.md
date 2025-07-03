## My Personal Computer Workstation

I decided to build my very own custom U-shaped computer workstation.
It would have an epoxied hardwood butcher style top with e notch at the bottom for LED strips.

It would have three distinct parts to it.
The first section would be a wide stand big enoung to house a computer repair platform and would have an open-frame chassis with power supply sitting on top of an ESD mat located right beside me.
The next section would be a stand-up desk that would house all the standard things: 3 monitors, mouse, keyboard, sound bar, stream deck, lighting, microphone, etc. It would also have a control to raise and lower the top.
The last section would be a small stand which would have the other peripherals like printer, scanner, external hard drives, reference books, etc.

On each of the sections, there would be an addressable LED strip which would surround the entire sections.
Of course, since these section have different sizes, so would be the number of LEDs.

## History
Since I have made this custom, I had originally thought that the stand-up table would be powered by 2 linear actuators and an Arduino board and three addressable LED strips.
As I soon discovered that it needed a 4-relay module. So a little more code had to be added... and learning.
Then I found out that, if I reversed polarity, it would reset the arduino since the power fluctuations would too much for it.
So I had to get a bi-directional diode to compensate for this.
Looking for something that my addrerssable LEDs would do led me down a huge rabbit hole.. way more code... and a way more testing...
I soon found out that the Arduino Uno would not be able to deal with all of this code; out of memeory and things didn't seem right, and waas running out of pins to work with.
Hence the DOIT V1 devkit ESP32... and a huge change of code.
Since my linear actuators were not matched, they climbed or descended at different speeds; This meant my table was going crooked, unstable, and scary really quickly.
This meant I needed something to keep the table level; hence the MPU-5060 gyro and accelerometer module.
That involved a whole hell of a lot more code... and learning.
I also found out that the table needed a reference point to start and another to stop at any given time.
The gyro had problems trying to decipher where it was at and using the serial monitor was getting me nowhere.
So I thought "Hey what about another screeen to see what is going on!"
That is when the SSD1306 came into play... and more learning.. and more code.
I soon found out that I could do more on this tiny little screen which led me down a rabbit hole... ugh, more learning... and a huge amount of code.
Still, the screen could only show me as much as it could display, but there was much more that I neede to see.
That is when the micro SD Card module came into play... again, more learning.. more code.
Now the problem came to light that I needed some sort of reference height to use as a base but would be a default value.
So I came up with the Ultrasonic sensor module... I'm sure you guessed it; more learning... more code...
Now things were getting way out of hand.

## Contents:
This is almost the full series of upgrades to my computer wokstation.
This program inside this repository includes the following modules:
- DOIT V1 Devkit ESP32;
- 4-relay module;
- Ultrasonic distance module;
- Micro SDCard module with 8Gb SD Card;
- OLED SSD1306;
- MPU-5060 gyro and accelerometer module;
- 3x WS2818A LED RGB Addressable strips; and
- 2x Linear Actuators.

One last piece of the puzzle has yet to be installed: the RT-11 or more commonly named JCHT35K9... now there's a major problem to be had!!
 
I am using PlatformIO in VSCode for various reasons (library download/upgrades, series output, uploading to ESP32).

## Diagram:
Below is the wiring diagram currently used to attach all to my workstation:
<img height=500 width=750 alt="Wiring diagram" src="https://github.com/Scarecrow1965/LED-SDCard-MPU-LA-OLED-ESP32/blob/main/ESP32-deskstand-wiring2.png">
You can download it to get more precise information.

##Problems:
#1 - Since this house is well built; the gyro is always having issues stabilizing. This sometime leads the ESP to reset and/or simply enable one of the linear actuators.
#2 - Had to find a way to create a way to ensure the standup table would work through the serial monitor; think I have it down pat but not sure.
#3 - Trying to get this RT-11 to work is like trying to pull my own teeth; very painful and very non-productive. 
  There are only a few websites that detail some work done, but the code in them simply does not work for my setup.
