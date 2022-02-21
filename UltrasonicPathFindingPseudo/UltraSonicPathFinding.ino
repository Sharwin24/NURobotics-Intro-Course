#include <Servo.h>  //servo library
Servo myservo;      // create servo object to control servo

// Ultrasonic pins
int Echo = A4;
int Trig = A5;
int rightDistance = 0;
int leftDistance = 0;
int forwardDistance = 0;

//define L298n module IO Pin
#define ENA 5
#define ENB 6
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 11

// Ultrasonic functions ----------------------------

// Ultrasonic distance measurement Sub function
int getDistance() {
    // Create a square wave to send out and read
    digitalWrite(Trig, LOW);
    delayMicroseconds(2);
    digitalWrite(Trig, HIGH);
    delayMicroseconds(20);
    digitalWrite(Trig, LOW);
    float Fdistance = pulseIn(Echo, HIGH);
    Fdistance = Fdistance / 58; // 58 for cm, 148 for in
    return (int)Fdistance;
}

// -------------------------------------------------

// Move functions ----------------------------------------
void forward() {
    digitalWrite(ENA, HIGH);  //enable L298n A channel
    digitalWrite(ENB, HIGH);  //enable L298n B channel
    digitalWrite(IN1, HIGH);  //set IN1 hight level
    digitalWrite(IN2, LOW);   //set IN2 low level
    digitalWrite(IN3, LOW);   //set IN3 low level
    digitalWrite(IN4, HIGH);  //set IN4 hight level
}

void back() {
    digitalWrite(ENA, HIGH);
    digitalWrite(ENB, HIGH);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

void left() {
    digitalWrite(ENA, HIGH);
    digitalWrite(ENB, HIGH);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}

void right() {
    digitalWrite(ENA, HIGH);
    digitalWrite(ENB, HIGH);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

void stop() {
    digitalWrite(ENA, LOW);
    digitalWrite(ENB, LOW);
}
// -----------------------------------------------

void setup() {
    Serial.begin(9600);
    myservo.attach(3, 700, 2400);  // attach servo on pin 3 to servo object and force min/max to [700,2400]
    pinMode(Trig, OUTPUT);
    pinMode(Echo, INPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
    stop();  // Optional: Ensures car is stopped during setup and before loop()
}

void loop() {
    // If close to a wall looking forward {
    // Stop the car
    // Look right and take measurement
    // Look left and take measurement
    // If right wall is farther than left wall, move right
    // If left wall is farther than right wall, move left
    // If both left and right walls are close to the robot, then back up
    // If right and left are equal, move forward
    // }
    // else move forward
}

void loop() {
    myservo.write(90);  //setservo position according to scaled value
    delay(500);
    forwardDistance = getDistance();

    if (forwardDistance <= 40) {
        stop();
        delay(500);
        myservo.write(10);
        delay(1000);
        rightDistance = getDistance();

        delay(500);
        myservo.write(90);
        delay(1000);
        myservo.write(180);
        delay(1000);
        leftDistance = getDistance();

        delay(500);
        myservo.write(90);
        delay(1000);
        if (rightDistance > leftDistance) {
            right();
            delay(360);
        } else if (rightDistance < leftDistance) {
            left();
            delay(360);
        } else if ((rightDistance <= 40) || (leftDistance <= 40)) {
            back();
            delay(180);
        } else {
            forward();
        }
    } else {
        forward();
    }
}
