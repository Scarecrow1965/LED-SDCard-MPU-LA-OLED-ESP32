## My Personal Computer Workstation

I decided to build my very own custom U-shaped computer workstation so i can learn how to code and even do some computewr building as a side hobby/job.
It would have an epoxied hardwood butcher style top with a notch at the bottom for LED strips and the rest would be parts of an old computer server mainframe I got a while ago.

It would have three distinct parts to it.
The first section would be a wide stand big enoung to house a computer repair platform and would have an open-frame chassis with power supply sitting on top of an ESD mat located right beside me.
The second section would be a stand-up desk that would house all the standard things: 3 monitors, mouse, keyboard, sound bar, stream deck, lighting, microphone, etc. It would also have a control to raise and lower the top.
The third section would be a small stand which would have the other peripherals like printer, scanner, external hard drives, reference books, etc.

On each of the sections (at the bottom of the hardwood), there would be an addressable WS2818A LED strip which would surround the entire section.
Of course, since these section have different sizes, so would be the number of LEDs.

## Contents:
This is almost the full series of upgrades to my computer wokstation.
This program inside this repository includes the following modules:
- 1x DOIT V1 Devkit ESP32 (<a href="https://randomnerdtutorials.com/getting-started-with-esp32/" target="_blank">link here</a>);
- 1x 4-relay module (example of it : <a href="https://www.amazon.ca/ELEGOO-Channel-Optocoupler-Arduino-Raspberry/dp/B06XCKQ1M9?th=1" target="_blank">link here</a>);
- 1x Ultrasonic distance module (<a href="https://www.hackster.io/csw1/ultrasonic-sensor-with-arduino-uno-f33ca1" target="_blank">link here</a>);
- 1x Micro SDCard module with 8Gb SD Card (<a href="https://www.amazon.ca/Storage-Memory-Shield-Module-Arduino/dp/B01IPCAP72" target="_blank">link here</a>);
- 1x OLED SSD1306 (example of it: <a href="https://www.amazon.ca/UCTRONICS-SSD1306-Self-Luminous-Display-Raspberry/dp/B072Q2X2LL" target="_blank">link here</a>);
- 1x MPU-6050 gyro and accelerometer module (<a href="https://howtomechatronics.com/tutorials/arduino/arduino-and-mpu6050-accelerometer-and-gyroscope-tutorial/" target="_blank">link here</a>);
- 3x WS2818B LED RGB Addressable strips (<a href="https://www.superlightingled.com/ws2818-ic-programmable-led-strips-c-5_488_183.html" target="_blank">link here</a>);
- 2x Linear Actuators (<a href="https://www.windynation.com/products/linear-actuators" target="_blank">link here</a>); and
- 2x Bi-directional Diodes (TVS) (<a href="https://www.rohm.com/electronics-basics/diodes/di_what8" target="_blank">link here</a>).

One last piece of the puzzle yet to be installed is the RT-11 (<a href="https://www.progressiveautomations.ca/products/rt-11" target="_blank">link here</a>) or more commonly named JCHT35K9... now there's a major problem to be had!!

## History
Since I wanted to build this custom workstation, I had originally thought that the stand-up table section would be powered by 2 linear actuators and an Arduino board and the three addressable LED strips which would surround each section.
I soon discovered that I needed a 4-relay module. So a little more code had to be added... and learning.
Then I found out that, if I reversed polarity, it would reset the arduino since the power fluctuations would too much for it.
So I had to get bi-directional diodes to compensate for this. (That took a long time to figure that out.)
Looking for something that my addrerssable LEDs would do led me down a huge rabbit hole.. way more code... and a way more testing...
I soon found out that the Arduino Uno would not be able to deal with all of this code; out of memory and things didn't seem right, and was running out of pins to work with.
Hence the DOIT V1 devkit ESP32... and a huge learning curve... and change of code.
Since my linear actuators were not matched, they climbed or descended at different speeds; This meant my table was going crooked, unstable, and scary really quickly.
This meant I needed something to keep the table level; hence the MPU-6050 gyro and accelerometer module.
That involved a whole hell of a lot more code... and learning.
Plus I did manage to code a webpage representation of what the gyro was doing (Thank you internet and fellow programmers!).
I also found out that the table needed a reference point to start and another to stop at any given time since the MPU could not keep the base height and level.
The gyro had problems trying to decipher where it was at and using the serial monitor was getting me nowhere.
So I thought "Hey what about another screeen to see what is going on!"
That is when the SSD1306 came into play... and more learning.. and more code.
I soon found out that I could do more on this tiny little screen which led me down a rabbit hole... ugh, more learning... and a huge amount of code.
Still, the screen could only show me as much as it could display and in real time, but some of the problems I was encountering couldn't be displayed on the screeen or the responses were too fast for me to decipher.
I knew there was much more that I needed to see. Plus I thought if we could save the base level and height and all this extra code I have been slowly accumulating, it might make things easier.
That is when the micro SD Card module came into play... again, more learning.. more code. But it didn't make things easier on my brain...
And I also thought that we could save the data and responses to the SD Card and then I could look into it after the fact.
I think I ended up doing more exercises retrieving and putting that SD Card back into the system during those times then I need for physical activities.
Now the problem came to light that I needed some sort of reference height to use as a base but would be a default value.
So I came up with the Ultrasonic sensor module... I'm sure you guessed it; more learning... more code...
Now, I think I have all the parts necessary for this workstation to work independently.

## Personal/Coding Notes
As a very new/junior programmer, this project has gotten way out of hand real quick/over my head. But stubborn as I am, I trudged on.
I am using PlatformIO in VSCode for various reasons (library download/upgrades, series output, uploading to ESP32), although I have not used PlatformIO to its full potential.

## Diagram:
Below is the wiring diagram currently used to attach all to my workstation:
<img height=500 width=750 alt="Wiring diagram" src="https://github.com/Scarecrow1965/LED-SDCard-MPU-LA-OLED-ESP32/blob/main/ESP32-deskstand-wiring2.png">
You can download it to get more precise information.

## Problems:
Problems that I have encountered and fixed(??):
#1- Hmm. With only the basic parts, I can only energize the linear actuator one way. Guess I need a 4-relay module to switch polarities on the linear Actuators. (one problem down)
#2- Why does this Arduino keep reseting if I engage the Linear actuators up or even down? Ugh, the bi-directional doides... That took a long time to figure it out... (one more problem down)
#3- What will my LEDs do? oh boy... The sky's the limit on this one... Let's see if I can maybe create a dozen of so ways for it to do things... No more problems I hope...
#4- Uh oh... out of memory? 4K is simply not enough for what I need it to do and I'm no where close to being done... Which arduino can I use. So I tried the Arduino Mega...  Geez, still not enough memory. Time for an ESP32 and dev board!!
#5- Why are my linear actuators not moving at the same speed? ugh, now what? How the hell am I going to get this fixed? Guess I need some sort of gyro system. Does it come in a micro size? Yep, sure does, welcome the MPU-6050! (problem solved)
#6- still running into issues that I can't see through serial monitor...
#7- How am I going to see what the hell is going on in real time... SSD1306 to the rescue... Ooops incoming Rabit hole!!! (Problem solved)
#8- Since this house is well built, the gyro is always having issues stabilizing...
  How am I going to see what is going on without my doing calestenics (bending over an looking underneath my workstation to look at the SSD1306 while looking at the serial monitor's outputs)?
  This sometime leads the ESP to reset and/or simply enable one of the linear actuators and then reset in the process. (still a problem)
#9- Uh oh.. out of memory again? Got way too much code and need to modularize things. Saving it to files? Need an external memory storage (aka external harddrive?) to do that...
#10- how can I find out how everything works if the tiny little SSD1306 can't show it all to me. Ah, what about logging the information?
#11- maybe I can kill two birds with one stone? An SD Card module? Do they make them that small, you bet!... Now I can call the animation on the screen, and save the datalogs...
#12- Had to find a way to create a way to ensure the standup table would work through the serial monitor; think I have it down pat but not sure but what about the remote control (RT-11). (still a problem, a timing issue?)
#13- Alright, the random LEDs program is supposed to be active if the table is not moving... think it's working? only sometimes though. (still a problem)
#14- The LEDs should display the level of the table while it is moving.... hmmm. that doesn't seem to work. (still a problem)
#15 - 
#??- Trying to get this RT-11 to work is like trying to pull my own teeth; very painful and very non-productive. 
 There are only a few websites that detail some work done, but the code in them simply does not work for my setup.
 
 Refs:
 1) <a href="https://embedded-elixir.com/post/2019-01-18-nerves-at-home-desk-controller/" target="_blank">https://embedded-elixir.com/post/2019-01-18-nerves-at-home-desk-controller/</a>; and
 2) <a href="https://hackaday.io/project/4173-uplift-desk-wifi-link/log/13628-remote-connector" target="_blank">https://hackaday.io/project/4173-uplift-desk-wifi-link/log/13628-remote-connector</a>;
 
 Found out that I have power to it, but that's it... no LEDs on in the RT-11 and the up and down switches don't work....
