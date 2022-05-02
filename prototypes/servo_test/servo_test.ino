#include <Servo.h>

#define SERVO_PIN 9

Servo shifter;

void setup(){
  shifter.attach(SERVO_PIN);
}

void loop(){
  shifter.write(0);
  delay(1000);
  shifter.write(180);
  delay(1000);
}
