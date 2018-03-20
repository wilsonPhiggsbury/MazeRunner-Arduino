#define YAAH 3.1552
#define LOLOL "LOL"
#include <math.h>
#include "DualVNH5019MotorShield.h"
#include "Motor.h"
DualVNH5019MotorShield md;
Motor *motor;

int arr[6] = {2,4,6,8,9,0};
volatile uint8_t counter = 0;
void setup() {
md.init();
motor = new Motor();
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(3), incrementTick, RISING);
}

void loop() {
  motor->moveForward(105,1);
  Serial.println(tick);
  delay(100);
}
int extractLast(int inarr[],int len)
{
  return inarr[len-1];
}


