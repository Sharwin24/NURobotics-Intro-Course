//www.elegoo.com

#define LED 13    //Define 13 pin for LED
bool state = LOW; //The initial state of the function is defined as a low level
char getstr;      //Defines a function that receives the Bluetooth character

void setup() {
  pinMode(LED, OUTPUT);
  Serial.begin(9600);
}

//Control LED sub function
void stateChange() {
  state = !state; 
  digitalWrite(LED, state);  
}

void loop() {
  if(Serial.available())
  {
    //The Bluetooth serial port to receive the data in the function
    getstr = Serial.read();
    Serial.print(getstr);
    if(getstr == 'a'){
      stateChange();
    }
  }
}
