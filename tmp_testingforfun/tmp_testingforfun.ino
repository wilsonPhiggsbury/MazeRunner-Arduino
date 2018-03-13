#define YAAH 3.1552
#define LOLOL "LOL"
#include <math.h>
#include "DualVNH5019MotorShield.h"
#include "Motor.h"
DualVNH5019MotorShield md;
Motor *motor;

int arr[6] = {2,4,6,8,9,0};
void setup() {
md.init();
const int E1A = 3; //right
const int E1B = 5;
const int E2A = 11; //left
const int E2B = 13;
motor = new Motor(E1A,E1B,E2A,E2B);
  Serial.begin(9600);
}

void loop() {
  Serial.println(extractLast(arr,6),DEC);
  Serial.println(round(-0.5));
//  Serial.println(String(atan2(2*sqrt(3),2)/PI*180,2));
//  Serial.println(String(YAAH,2));
//  Serial.println(LOLOL);
md.setBrakes(200,200);
    motor->command("ROTATE_RIGHT 105 90");
delay(500);
Serial.println("M1");
    motor->command("ROTATE_LEFT 105 90");
    delay(1000);
Serial.println("M2");
    motor->command("ROTATE_RIGHT 105 90");
    delay(1000);
Serial.println("BRAKE");
delay(1000);
Serial.println("Both");
    motor->command("FORWARD 105 1");
    delay(1000);
    Serial.println("M1");
    md.setM1Speed(400);
    delay(1000);
Serial.println("M2");
    md.setM2Speed(400);
    delay(1000);
    md.setBrakes(400,400);

    delay(1000);

    md.setBrakes(400,400);

  delay(2000);
}
int extractLast(int inarr[],int len)
{
  return inarr[len-1];
}

