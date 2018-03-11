#define YAAH 3.1552
#define LOLOL "LOL"
#include <math.h>

int arr[6] = {2,4,6,8,9,0};
void setup() {

  Serial.begin(9600);
}

void loop() {
  Serial.println(extractLast({0,4,3,2,1,9},6),DEC);
  Serial.println(round(-0.5));
//  Serial.println(String(atan2(2*sqrt(3),2)/PI*180,2));
//  Serial.println(String(YAAH,2));
//  Serial.println(LOLOL);
  
  delay(2000);
}
int extractLast(int inarr[],int len)
{
  return inarr[len-1];
}

