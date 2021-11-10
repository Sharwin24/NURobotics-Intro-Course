//define our pin numbers
#define ENA 5
#define ENB 6
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 11

//set a car speed within 0-255
#define carSpeed 250

void setup() {
  //set our pins as output since we'll write commands to it
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);
}

void loop() {
  //we set the direction here as forwards with the INX pins
  digitalWrite(IN1,HIGH); 
  digitalWrite(IN2,LOW);  
  digitalWrite(IN3,LOW);  
  digitalWrite(IN4,HIGH);
  
  //with our direction set, we send a signal controlling the speed
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);

  //try adding some other directions based on the table in the slides
  
  //don't forget to add some delays so your bot knows how long to 
  //go in each direction!
}
