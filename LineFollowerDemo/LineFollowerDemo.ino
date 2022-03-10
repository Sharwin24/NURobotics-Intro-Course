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
#define stopTime 3000

// Variables for tracking the state of the robot, timers, etc.
unsigned long startTime;

// Movement functions from RobotMovement

/**
 * @brief Moves the robot forward with the given speed
 *
 * @param carSpeed an integer representing the car's speed.
 * This value is capped at 255 and passing in a higher value will
 * cap at 255
 */
void forward(int carSpeed)  // Forward
{
    // Setting Direction and Power Pins
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    // Write speed to 'A' and 'B' MotorGroups
    analogWrite(ENA, carSpeed);
    analogWrite(ENB, carSpeed);
}

/**
 * @brief Moves the robot backwards with the given speed
 *
 * @param carSpeed an integer representing the car's speed.
 * This value is capped at 255 and passing in a higher value will
 * cap at 255
 */
void back(int carSpeed) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, carSpeed);
    analogWrite(ENB, carSpeed);
}

/**
 * @brief Turns the robot left with the given speed
 *
 * @param carSpeed an integer representing the car's speed.
 * This value is capped at 255 and passing in a higher value will
 * cap at 255
 */
void turnLeft(int carSpeed) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, carSpeed);
    analogWrite(ENB, carSpeed);
}

/**
 * @brief Turns the robot right with the given speed
 *
 * @param carSpeed an integer representing the car's speed.
 * This value is capped at 255 and passing in a higher value will
 * cap at 255
 */
void turnRight(int carSpeed) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, carSpeed);
    analogWrite(ENB, carSpeed);
}

/**
 * @brief Stops the robot's drive motors.
 *
 */
void stop() {
    digitalWrite(ENA, 0);
    digitalWrite(ENB, 0);
}

// End movement functions from RobotMovement

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
