#define ENA 5
#define ENB 6
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 11

#include <Servo.h>
Servo myservo;  // create servo object to control servo

int Echo = A4;
int Trig = A5;

// Variables for saved ultrasonic distance measurements
int rightDistance = 0, leftDistance = 0, middleDistance = 0;

// Ultrasonic distance measurement Sub function
int getDistance() {
    // trigger an ultrasonic wave for 20 microseconds
    digitalWrite(Trig, LOW);
    delayMicroseconds(2);
    digitalWrite(Trig, HIGH);
    delayMicroseconds(20);
    digitalWrite(Trig, LOW);
    // calculation converting time until signal returned to distance
    float Fdistance = pulseIn(Echo, HIGH);
    Fdistance = Fdistance / 58;
    // returns an integer as the distance (in cm) sensed
    return (int)Fdistance;
}

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
    digitalWrite(IN4, LOW);
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

void setup() {
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(Echo, INPUT);
    pinMode(Trig, OUTPUT);
    myservo.attach(3, 700, 2400);  // attach servo on pin 3 to servo object with min/max: 700/2400
    Serial.begin(9600);
    stop();
}

void loop() {
    myservo.write(90);  // setservo position according to scaled value (forward)
    delay(500);
    middleDistance = getDistance();  // grab the distance with the servo forwards

    // if we sense an object <= 40 cm away...
    if (middleDistance <= 40) {
        stop();

        // turn the servo/sensor right and take a distance measurement
        delay(500);
        myservo.write(10);
        delay(1000);
        rightDistance = getDistance();

        // turn the servo/sensor left and take a distance measurement
        delay(500);
        myservo.write(90);
        delay(1000);
        myservo.write(180);
        delay(1000);
        leftDistance = getDistance();

        delay(500);
        myservo.write(90);
        delay(1000);

        // go right if there's more space to the right
        if (rightDistance > leftDistance) {
            right();
            delay(360);
        }
        // go left if there's more space to the left
        else if (rightDistance < leftDistance) {
            left();
            delay(360);
        }
        // go back if we're too close
        else if ((rightDistance <= 40) || (leftDistance <= 40)) {
            back();
            delay(180);
        }
        // otherwise we're good to go forwards
        else {
            forward();
        }
    }
    // go forwards if no object within 40 cm
    else {
        forward();
    }
    // Program delay
    delay(20);
}