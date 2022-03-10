// Line Tracker Sensor I/O setup
// Remove Inverse operator to track bright lines on dark backgrounds
#define LT_R !digitalRead(10)  // Right sensor
#define LT_M !digitalRead(4)   // Middle sensor
#define LT_L !digitalRead(2)   // Left sensor

// Drivebase Motor Setup
#define ENA 5
#define ENB 6
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 11

// Constants for program
#define carSpeed 250
#define stopTime 3000

// Variables for tracking the state of the robot, timers, etc.
unsigned long startTime;

/*
    Copy over the movement functions from RobotMovement.ino
*/

// Setup Pinmodes for sensor
void setup() {
    Serial.begin(9600);  // Start Serial monitor
    // Input Pins for Line Tracker
    pinMode(10, INPUT);
    pinMode(4, INPUT);
    pinMode(2, INPUT);
    startTime = millis();  // Start a running timer
}

// Track and follow a dark line
void loop() {
    // If middle sensor reads line, move forward
    // If the right sensor reads line, turn right until sensor no longer reads line
    // if the left sensor reads line, turn left until sensor no longer reads line

    // Stop the car after 30,000 ms (30s)
    if (startTime > stopTime) {
        stop();
    }
}
