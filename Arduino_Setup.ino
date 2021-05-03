/*
By: Sharwin Patil
Contact me at: patil.sha@northeastern.edu

Here we'll cover how to get started with Arduino
for the NURobotics Intro Class.

The following documentation can be used to read
more about specifics of getting started with Arduino: 
https://create.arduino.cc/projecthub/Arduino_Genuino/getting-started-with-arduino-web-editor-on-various-platforms-4b3e4a

The Arduino IDE is a very common IDE(Integrated Developer Environment) for coding with Arduino.
If you are interested in using another IDE such as VSCode, here is a useful link:
https://maker.pro/arduino/tutorial/how-to-use-visual-studio-code-for-arduino

Arduino programming is done in C++, along with many libraries
that help you use various sensors/motors/etc. We can include 
libraries incredibly easily in the Arduino IDE by using the
'include' keyword like so:
#include <LibraryName.h>

The .h at the end of the Library denotes that it is a
'header' file, which is a file written in C++ that includes
declarations and setup for a Library or Class.
*/

#include <Servo.h>

// Setup Function
void setup() {
    // This function runs once upon starting the program
    // We'll define variables and setup Inputs/Outputs here
}

// Loop Function
void loop() {
    // This function loops over again and runs the code
    // within the function forever
    // LED Blink, without delay, using the millis timer
    digitalWrite(LED_BUILTIN, millis() % 512 > 256);
}