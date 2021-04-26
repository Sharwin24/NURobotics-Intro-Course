/*
 * modified 25 April 2021
 * by Sharwin Patil
 * purpose: to use the HC-SR501 PIR motion sensor to turn an LED on 
 *          if motion is detected
 * 
 * Note: looking at the bottom of the sensor with the pins along the top,
 *       the pins go GND, output, VCC from left to right. Also, the 
 *       potentiometers on the bottom can be used to adjust the SENSITIVITY
 *       for the maximum distance of motion detected, and also the TIME for
 *       how long the output will remain HIGH after detection
 */

int ledPin = LED_BUILTIN;  // choose the pin for the LED
int inputPin = 8;          // choose the input pin (for PIR sensor)
int pirState = LOW;        // we start, assuming no motion detected
int val = 0;               // variable for reading the pin status

void setup() {
    pinMode(ledPin, OUTPUT);   // declare LED as output
    pinMode(inputPin, INPUT);  // declare sensor as input

    Serial.begin(9600);
}

void loop() {
    val = digitalRead(inputPin);  // read input value

    if (val == HIGH)  // check if the input is HIGH
    {
        digitalWrite(ledPin, HIGH);  // turn LED ON

        if (pirState == LOW) {
            Serial.println("Motion detected!");  // print on output change
            pirState = HIGH;
        }
    } else {
        digitalWrite(ledPin, LOW);  // turn LED OFF

        if (pirState == HIGH) {
            Serial.println("Motion ended!");  // print on output change
            pirState = LOW;
        }
    }
}