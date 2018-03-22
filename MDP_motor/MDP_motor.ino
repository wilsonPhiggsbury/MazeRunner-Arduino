#include "Motor.h"

Motor *motor;

void setup() {
  // put your setup code here, to run once:
  motor = new Motor();
  Serial.begin(9600);
}

void loop() {
  delay(1000);
  motor->moveForward(105, 4);
}
