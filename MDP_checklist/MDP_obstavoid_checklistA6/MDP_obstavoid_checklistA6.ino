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
}

void loop() {
  delay(1000);
  Serial.println(sensors[FL]->takeReading(true));
  if(isClear()){
    motor->moveForward(SPEED,1);
  }
  else {
    if(sensors[FL]->takeReading(true) < 40)
    {
      motor->rotateRight(SPEED,90);
      delay(200);
      motor->moveForward(SPEED,1);
      delay(200);
      motor->rotateLeft(SPEED,90);
      delay(200);
      motor->moveForward(SPEED,4);
      delay(200);
      motor->rotateLeft(SPEED,90);
      delay(200);
      motor->moveForward(SPEED,1);
      delay(200);
      motor->rotateRight(SPEED,90);
    }
    else if(sensors[FM]->takeReading(true) < 140) {
      motor->rotateRight(SPEED,45);
      delay(200);
      motor->moveForwardCustom(SPEED,720);
      delay(200);
      motor->rotateLeft(SPEED,45);
      delay(200);
      motor->moveForward(SPEED,3);
      delay(200);
      motor->rotateLeft(SPEED,45);
      delay(200);
      motor->moveForwardCustom(SPEED,720);
      delay(200);
      motor->rotateRight(SPEED,45);
    }
    else {
      motor->rotateLeft(SPEED,90);
      delay(200);
      motor->moveForward(SPEED,1);
      delay(200);
      motor->rotateRight(SPEED,90);
      delay(200);
      motor->moveForward(SPEED,4);
      delay(200);
      motor->rotateRight(SPEED,90);
      delay(200);
      motor->moveForward(SPEED,1);
      delay(200);
      motor->rotateLeft(SPEED,90);
    }
  }  
}

boolean isClear(){
  Serial.println(sensors[FL]->takeReading(true));
  if(sensors[FM]->takeReading(true) >= 140 &&
  sensors[FL]->takeReading(true) >= 140  &&
  sensors[FR]->takeReading(true) >= 140){
    return true;
    
  }
  else{
    return false;
  }
}

