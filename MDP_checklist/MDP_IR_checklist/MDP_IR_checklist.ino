#include "IR.h"

IR *IR_sensors[6];
int i;
bool raw = false;
void setup() {
  Serial.begin(9600);
  for(int i=0; i<6; i++)
  {
    IR_sensors[i] = new IR(i);
  }
}

void loop() {
  
  if(Serial.available()>1)
  {
    char readVal,readVal2;
    readVal = Serial.read();
    readVal2 = Serial.read();
    if(readVal == 'v' && readVal2 == '\n')raw = !raw;
    // print values, seperated by "tab tab newline tab tab newline"
//    for(i=0; i<6; i++)
//    {
//      float fittedDist;
//      fittedDist = IR_sensors[i]->takeReading(!raw);
//      Serial.print(String(fittedDist,0));
//      if(i==2 || i==5)Serial.print("\n");
//      else Serial.print("\t");
//    }
    Serial.println(String(IR_sensors[3]->takeReading(raw),2));
  }
  
  delay(200); // slow down serial port
}


