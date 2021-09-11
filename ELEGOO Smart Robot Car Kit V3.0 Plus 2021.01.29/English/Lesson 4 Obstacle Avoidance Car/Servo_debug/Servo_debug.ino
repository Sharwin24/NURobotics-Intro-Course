//www.elegoo.com
#include <Servo.h>
Servo myservo;

int minAngle = 700;//the pulse width, in microseconds, corresponding to the minimum (0-degree) angle on the servo (defaults to 700)
int maxAngle = 2400;//the pulse width, in microseconds, corresponding to the maximum (180-degree) angle on the servo (defaults to 2400)

void setup(){
  myservo.attach(3,minAngle,maxAngle);//setting the servo IO pin and the steering range.
  myservo.write(90);// move servos to center position -> 90°
} 
void loop(){

}
