#include "IR.h"
#include "Motor.h"

#define SPEED 105
IR *sensors[6];
Motor *motor;
void setup() {
  Serial.begin(9600);
  
  for(int i=0; i<6; i++)
  {
    sensors[i] = new IR(i);
  }
  motor = new Motor();
  delay(1000);
  motor->moveForward(SPEED,0);
  while(sensors[0]->takeReading(true) >0);
  motor->stopBot();
  delay(200);
  motor->rotateRight(SPEED,90);
  delay(200);
  motor->stopBot();
  delay(200);
  motor->moveForward(SPEED,2);
  delay(200);
  motor->stopBot();
  delay(200);
  motor->rotateLeft(SPEED,90);
  delay(200);
  motor->stopBot();
  delay(200);
  motor->moveForward(SPEED,3);
  delay(200);
  motor->stopBot();
  delay(200);
  motor->rotateLeft(SPEED,90);
  delay(200);
  motor->stopBot();
  delay(200);
  motor->moveForward(SPEED,2);
  delay(200);
  motor->stopBot();
  delay(200);
  motor->rotateRight(SPEED,90);
  delay(200);
  motor->stopBot();
  delay(200);
  motor->moveForward(SPEED,0);
  while(sensors[0]->takeReading(true) > 100);
  motor->stopBot();
}

void loop() {
  
}
