#define ENA 5
#define ENB 6
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 11

#include <Servo.h>

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
    // TODO: Complete this function
}

/**
 * @brief Turns the robot left with the given speed
 *
 * @param carSpeed an integer representing the car's speed.
 * This value is capped at 255 and passing in a higher value will
 * cap at 255
 */
void turnLeft(int carSpeed) {
    // TODO: Complete this function
}

/**
 * @brief Turns the robot right with the given speed
 *
 * @param carSpeed an integer representing the car's speed.
 * This value is capped at 255 and passing in a higher value will
 * cap at 255
 */
void turnRight(int carSpeed) {
    // TODO: Complete this function
}

void stop() {
    digitalWrite(ENA, 0);
    digitalWrite(ENB, 0);
}

void setup() {
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
}

void loop() {
    // put your code here!

    // here, we're moving the bot forward at speed 250...
    forward(250);
    // and continuing to move for 2 seconds
    delay(2000);
    // stopping the bot...
    stop();

    // try writing code to move the bot in a square and stop!
    // make it go fast on two sides and slow on the other two
    // and challenge yourselves to make it a square and not a weirdly shaped rectangle!

    delay(20);
}