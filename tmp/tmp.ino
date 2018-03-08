#define YAAH 3.1552
#define LOLOL "LOL"
#include <math.h>
void setup() {

  Serial.begin(9600);
}

void loop() {
  Serial.println(round(-0.4));
  Serial.println(round(-0.5));
//  Serial.println(String(atan2(2*sqrt(3),2)/PI*180,2));
//  Serial.println(String(YAAH,2));
//  Serial.println(LOLOL);
  delay(500);
}
