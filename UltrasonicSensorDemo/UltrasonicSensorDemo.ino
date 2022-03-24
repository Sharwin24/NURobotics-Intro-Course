// www.elegoo.com

#include <Servo.h>  //servo library
Servo myservo;      // create servo object to control servo

int Echo = A4;
int Trig = A5;

// Drivebase Motor Setup
#define ENA 5
#define ENB 6
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 11

// Constant for car's speed, adjust as needed
#define carSpeed 250

// Variables for tracking the state of the robot, timers, etc.
int rightDistance = 0, leftDistance = 0, middleDistance = 0;

// Move Forward
void forward() {
    analogWrite(ENA, carSpeed);
    analogWrite(ENB, carSpeed);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    Serial.println("Forward");
}

// Move Back
void back() {
    analogWrite(ENA, carSpeed);
    analogWrite(ENB, carSpeed);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    Serial.println("Back");
}

// Move Left
void left() {
    analogWrite(ENA, carSpeed);
    analogWrite(ENB, carSpeed);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    Serial.println("Left");
}

// Move Right
void right() {
    analogWrite(ENA, carSpeed);
    analogWrite(ENB, carSpeed);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    Serial.println("Right");
}

// Stop the car
void stop() {
    digitalWrite(ENA, LOW);
    digitalWrite(ENB, LOW);
    Serial.println("Stop!");
}

// Ultrasonic distance measurement Sub function
int Distance_test() {
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

void setup() {
    myservo.attach(3, 700, 2400);  // attach servo on pin 3 to servo object
    Serial.begin(9600);
    pinMode(Echo, INPUT);
    pinMode(Trig, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
    stop();
}

void loop() {
    myservo.write(90);  // setservo position according to scaled value (forward)
    delay(500);
    middleDistance = Distance_test();  // grab the distance with the servo forwards

    // if we sense an object <= 40 cm away...
    if (middleDistance <= 40) {
        stop();

        // turn the servo/sensor right and take a distance measurement
        delay(500);
        myservo.write(10);
        delay(1000);
        rightDistance = Distance_test();

        // turn the servo/sensor left and take a distance measurement
        delay(500);
        myservo.write(90);
        delay(1000);
        myservo.write(180);
        delay(1000);
        leftDistance = Distance_test();

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
}
