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

const char* commands[] = {
  "ROTATE_LEFT 105 90",
  "ROTATE_RIGHT 105 90",
  "FORWARD 105 3",
  // new command
  "FORWARD 105", // forward without stopping
  "ROTATE_LEFT 105", // rotate left without stopping
  "ROTATE_RIGHT 105", // rotate right without stopping
  "STOP" // stop
  };

unsigned int command_index = 0;

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
  motor->command(commands[command_index]);
  command_index++;
  command_index %= 6;
  delay(1000);
}
