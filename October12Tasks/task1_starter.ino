#define ENA 5
#define ENB 6
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 11

//function to move forward at a given car speed
void forward(int carSpeed){
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

//function to move back at a given car speed
void back(int carSpeed){
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

//function to move left at a given car speed
void left(int carSpeed){
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

//function to move right at a given car speed
void right(int carSpeed){
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
} 

void stop(){
   digitalWrite(ENA, LOW);
   digitalWrite(ENB, LOW);
} 

void setup(){
  //set our pins as output since we'll write commands to it
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);
}

void loop() {
  //put your code here!

  //here, we're moving the bot forward at speed 250...
  forward(250);
  //and continuing to move for 3 seconds
  delay(3000);
  //stopping the bot...
  stop();
  //for 3 seconds
  delay(3000);

  //try writing code to move the bot in a square and stop!
  //make it go fast on two sides and slow on the other two
  //and challenge yourselves to make it a square and not a weirdly shaped rectangle!
}
