// Line Tracker Sensor I/O setup
// Remove Inverse operator to track bright lines on dark backgrounds
#define LT_R !digitalRead(10) // Right sensor
#define LT_M !digitalRead(4) // Middle sensor
#define LT_L !digitalRead(2) // Left sensor

// Drivebase Motor Setup
#define ENA 5
#define ENB 6
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 11

// Constants for program
#define carSpeed 250
#define stopTime 30000

// Variables for tracking the state of the robot, timers, etc.
unsigned long startTime;


// Move Forward
void forward() {
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println("go forward!");
}

// Move Left
void left(){
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println("go left!");
}

// Move Right
void right(){
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW); 
  Serial.println("go right!");
} 

// Stop the car
void stop(){
   digitalWrite(ENA, LOW);
   digitalWrite(ENB, LOW);
   Serial.println("Stop!");
} 

// Setup Pinmodes for sensor
void setup() {
  Serial.begin(9600); // Start Serial monitor
  pinMode(10,INPUT);
  pinMode(4,INPUT);
  pinMode(2,INPUT);
  startTime = millis();
}

// Track and follow a dark line
void loop() {
  if (LT_M) { // If middle sensor reads line, move forward
    forward();
  }
  else if (LT_R) {
    right();
    while(LT_R); // Turn right until sensor no longer reads line
  }
  else if (LT_L) {
    left();
    while(LT_L); // Turn left until sensor no longer reads line
  }
  // Stop the car after 30,000 ms (30s)
  if (startTime > stopTime) {
    stop();
  }
  
}

