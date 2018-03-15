#include "Motor.h"
//Pins
const int E1A = 3; //right
const int E1B = 5;
const int E2A = 11; //left
const int E2B = 13;

float desired_rpm = 60;
Motor *motor;
// management variables
String inputBuffer = "";

long startTime = 0;
long commandPeriod = 0;

void setup() {
  // put your setup code here, to run once:
  motor = new Motor(E1A, E1B, E2A, E2B);
  Serial.begin(9600);
  pinMode(E1A, INPUT);
  pinMode(E1B, INPUT);
  pinMode(E2A, INPUT);
  pinMode(E2B, INPUT);
}

void loop() {
  delay(1000);
  motor->moveForward(105, 5);
  delay(1000);
  motor->rotateRight(105, 90);
  delay(1000);
  motor->moveForward(105, 8);
  delay(1000);
  motor->rotateLeft(105, 90);
  delay(1000);
  motor->moveForward(105, 12);
  delay(1000);
  motor->rotateRight(105, 90);
  delay(1000);
  motor->moveForward(105, 4);
  delay(60000);
}
