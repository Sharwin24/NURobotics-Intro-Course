/*
 * @Description: Rocker_Control
 * @Author: HOU Changhua
 * @Date: 2019-08-12 18:00:25
 * @LastEditTime: 2019-08-27 10:45:29
 * @LastEditors: Please set LastEditors
 */
#include <Servo.h>
#include <stdio.h>
#include "HardwareSerial.h"
#include "ArduinoJson-v6.11.1.h" //ArduinoJson

/*Driving Interface for Ultrasound Ranging*/
#define ECHO_PIN A4
#define TRIG_PIN A5

/*Motor Drive Interface*/
#define ENA 5
#define ENB 6
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 11

/*Driving Interface for Infrared Pipeline Patrol*/
#define LineTeacking_Pin_Right 10
#define LineTeacking_Pin_Middle 4
#define LineTeacking_Pin_Left 2
#define LineTeacking_Read_Right !digitalRead(10)
#define LineTeacking_Read_Middle !digitalRead(4)
#define LineTeacking_Read_Left !digitalRead(2)

#define carSpeed 180 //PWM(That is: motor speed/vehicle speed)

Servo servo;
unsigned long IR_PreMillis;
unsigned long LT_PreMillis;

enum FUNCTIONMODE
{
  IDLE,
  LineTeacking,
  ObstaclesAvoidance,
  Bluetooth,

} func_mode = IDLE; /*Functional model*/

enum MOTIONMODE
{
  STOP,
  FORWARD,
  BACK,
  LEFT,
  RIGHT
} mov_mode = STOP; /*Motion model*/

void delays(unsigned long t)
{

  for (unsigned long i = 0; i < t; i++)
  {
    getBTData_Plus();
    delay(1);
  }
}

/*Acquisition Distance: Ultrasound*/
unsigned int getDistance(void)
{
  static unsigned int tempda = 0;
  unsigned int tempda_x = 0;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  tempda_x = ((unsigned int)pulseIn(ECHO_PIN, HIGH) / 58);
  if (tempda_x < 150)
  {
    tempda = tempda_x;
  }
  else
  {
    tempda = 30;
  }
  return tempda;
}
/*Control motor: */
void forward(bool debug, int16_t in_carSpeed)
{

  analogWrite(ENA, in_carSpeed);
  analogWrite(ENB, in_carSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  if (debug)
    Serial.println("Go forward!");
}

/*Control motor: */
void back(bool debug, int16_t in_carSpeed)
{

  analogWrite(ENA, in_carSpeed);
  analogWrite(ENB, in_carSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  if (debug)
    Serial.println("Go back!");
}
/*Control motor:*/
void left(bool debug, int16_t in_carSpeed)
{
  analogWrite(ENA, 250);
  analogWrite(ENB, 250);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  if (debug)
    Serial.println("Go left!");
}
/*Control motor:*/
void right(bool debug, int16_t in_carSpeed)
{
  analogWrite(ENA, 250);
  analogWrite(ENB, 250);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  if (debug)
    Serial.println("Go right!");
}
/*Control motor:*/
void stop(bool debug = false)
{
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
  if (debug)
    Serial.println("Stop!");
}
/*
  Bluetooth serial port data acquisition and control command parsing
*/
void getBTData_Plus(void)
{
  String comdata = "";

  while ((Serial.available() > 0) && (false == comdata.endsWith("}")))
  {
    comdata += char(Serial.read());
    delay(3);
  }
  if ((comdata.length() > 0) && (comdata != "") && (true == comdata.endsWith("}")))
  {
    StaticJsonDocument<200> doc;                                //Create a JsonDocument object
    DeserializationError error = deserializeJson(doc, comdata); //Deserialize JSON data
    if (!error)                                                 //Check if deserialization is successful
    {
      int control_mode_N = doc["N"];
      char buf[3];
      uint8_t temp = doc["H"];
      sprintf(buf, "%d", temp);
      String CommandSerialNumber = buf; //Get the serial number of the new command

      switch (control_mode_N)
      {
      case 5: /*Clear mode  processing <command：N 5>*/
        func_mode = Bluetooth;
        mov_mode = STOP;
        Serial.print('{' + CommandSerialNumber + "_ok}");
        break;
      case 3:               /*Remote switching mode  processing <command：N 3>*/
        if (1 == doc["D1"]) // Line Teacking Mode
        {
          func_mode = LineTeacking;
          Serial.print('{' + CommandSerialNumber + "_ok}");
        }
        else if (2 == doc["D1"]) //Obstacles Avoidance Mode
        {
          func_mode = ObstaclesAvoidance;
          Serial.print('{' + CommandSerialNumber + "_ok}");
        }
        break;
      case 2: /*Remote switching mode  processing <command：N 2>*/

        if (1 == doc["D1"])
        {
          func_mode = Bluetooth;
          mov_mode = LEFT;
          Serial.print('{' + CommandSerialNumber + "_ok}");
        }
        else if (2 == doc["D1"])
        {
          func_mode = Bluetooth;
          mov_mode = RIGHT;
          Serial.print('{' + CommandSerialNumber + "_ok}");
        }
        else if (3 == doc["D1"])
        {
          func_mode = Bluetooth;
          mov_mode = FORWARD;
          Serial.print('{' + CommandSerialNumber + "_ok}");
        }
        else if (4 == doc["D1"])
        {
          func_mode = Bluetooth;
          mov_mode = BACK;
          Serial.print('{' + CommandSerialNumber + "_ok}");
        }
        else if (5 == doc["D1"])
        {
          func_mode = Bluetooth;
          mov_mode = STOP;
          Serial.print('{' + CommandSerialNumber + "_ok}");
        }
        break;
      default:
        break;
      }
    }
  }
}

/*Bluetooth remote control mode*/
void bluetooth_mode()
{
  if (func_mode == Bluetooth)
  {
    switch (mov_mode)
    {
    case FORWARD:
      forward(false, carSpeed);
      break;
    case BACK:
      back(false, carSpeed);
      break;
    case LEFT:
      left(false, carSpeed);
      break;
    case RIGHT:
      right(false, carSpeed);
      break;
    case STOP:
      stop();
      break;
    default:
      break;
    }
  }
}
/*
  Line Teacking Mode
*/
void line_teacking_mode(void)
{
  if (func_mode == LineTeacking)
  {
    if (LineTeacking_Read_Middle)
    {                           //Detecting in the middle infrared tube
      forward(false, carSpeed); //Control motor：the car moving forward
      LT_PreMillis = millis();
    }
    else if (LineTeacking_Read_Right)
    {                         //Detecting in the right infrared tube
      right(false, carSpeed); //Control motor：the car moving right
      while (LineTeacking_Read_Right)
      {
        getBTData_Plus(); //Bluetooth data acquisition
      }
      LT_PreMillis = millis();
    }
    else if (LineTeacking_Read_Left)
    {                        //Detecting in the left infrared tube
      left(false, carSpeed); //Control motor：the car moving left
      while (LineTeacking_Read_Left)
      {
        getBTData_Plus(); //Bluetooth data acquisition
      }
      LT_PreMillis = millis();
    }
    else
    {
      if (millis() - LT_PreMillis > 150)
      {
        stop(); //Stop motor control：Turn off motor drive mode
      }
    }
  }
}
/*f(x) int */
static boolean function_xxx(long xd, long sd, long ed) //f(x)
{
  if (sd <= xd && xd <= ed)
    return true;
  else
    return false;
}
/*Obstacle avoidance*/
void obstacles_avoidance_mode(void)
{
  static uint16_t ULTRASONIC_Get = 0;
  static unsigned long ULTRASONIC_time = 0;
  static boolean stateCar = false;
  static boolean CarED = false;
  static uint8_t switc_ctrl = 0x00;
  static boolean timestamp = true;

  if (func_mode == ObstaclesAvoidance)
  {
    servo.attach(3);
    if (millis() - ULTRASONIC_time > 100)
    {
      ULTRASONIC_Get = getDistance(); //Measuring obstacle distance
      ULTRASONIC_time = millis();
      if (function_xxx(ULTRASONIC_Get, 0, 25)) //If the distance is less than Xcm obstacles
      {
        stateCar = true;
        stop(); //stop
      }
      else
      {
        stateCar = false;
      }
    }
    if (false == CarED)
    {
      if (stateCar == true)
      {
        timestamp = true;
        CarED = true;
        switc_ctrl = 0x00;
        stop();          //stop
        servo.write(30); //sets the servo position according to the  value
        delays(500);
        if (function_xxx(getDistance(), 0, 25)) //How many cm in the front have obstacles?
        {
          switc_ctrl |= 0x01;
          //goto
        }
        else
        {
          switc_ctrl &= (~0x01);
        }
        servo.write(150); //sets the servo position according to the  value
        delays(500);
        if (function_xxx(getDistance(), 0, 25)) //How many cm in the front have obstacles?
        {
          switc_ctrl |= 0x02;
          //goto
        }
        else
        {
          switc_ctrl &= (~0x02);
        }
        servo.write(90); //tell servo to go to position in variable 30
        delays(500);
        if (function_xxx(getDistance(), 0, 25)) //How many cm in the front have obstacles?
        {
          switc_ctrl |= 0x04;
          //goto
        }
        else
        {
          switc_ctrl &= (~0x04);
        }
      }
      else
      {
        forward(false, 180); //Control motor：the car moving forwar
        CarED = false;
      }
    }

    if (true == CarED)
    {
      // Get cpu time
      static unsigned long MotorRL_time;
      if (timestamp == true || millis() - MotorRL_time > 420)
      {
        timestamp = false;
        MotorRL_time = millis();
      }
      if (function_xxx((millis() - MotorRL_time), 0, 400))
      {
        switch (switc_ctrl)
        {
        case 0 ... 1:
          left(false, 150); //Control motor：The car moves forward and left
          break;
        case 2:
          right(false, 150); //Control motor：The car moves forward and right
          break;
        case 3:
          forward(false, 150); //Control motor：the car moving forwar
          break;
        case 4 ... 5:
          left(false, 150); //Control motor：The car moves forward and left
          break;
        case 6:
          right(false, 100); //Control motor：The car moves forward and right
          break;
        case 7:
          back(false, 150); //Control motor：Car backwards
          break;
        }
      }
      else
      {
        CarED = false;
      }
    }
  }
  else
  {
    servo.detach();
    ULTRASONIC_Get = 0;
    ULTRASONIC_time = 0;
  }
}
void setup(void)
{
  Serial.begin(9600);         //initialization
  servo.attach(3, 500, 2400); //500: 0 degree  2400: 180 degree
  servo.write(90);            //sets the servo position according to the 90（middle）

  pinMode(ECHO_PIN, INPUT); //Ultrasonic module initialization
  pinMode(TRIG_PIN, OUTPUT);

  pinMode(IN1, OUTPUT); //Motor-driven port configuration
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(LineTeacking_Pin_Right, INPUT); //Infrared tracking module port configuration
  pinMode(LineTeacking_Pin_Middle, INPUT);
  pinMode(LineTeacking_Pin_Left, INPUT);
}

void loop(void)
{
  getBTData_Plus();           //Bluetooth data acquisition
  bluetooth_mode();           //Bluetooth remote mode
  line_teacking_mode();       //Line Teacking Mode
  obstacles_avoidance_mode(); //Obstacles Avoidance Mode
}
