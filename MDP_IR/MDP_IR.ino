#include "IR.h"

IR *IR_sensors[6];
int i;
bool raw = true;
void setup() {
  Serial.begin(9600);
  for(int i=0; i<6; i++)
  {
    IR_sensors[i] = new IR(i);
  }
}

void loop() {
  if(Serial.available()>1)
  {//Serial.println("HERE");
    char readVal,readVal2;
    readVal = Serial.read();
    readVal2 = Serial.read();
    if(readVal == 'v' && readVal2 == '\n')raw = !raw;
    // print values, seperated by "tab tab newline tab tab newline"
    for(i=0; i<6; i++)
    {
      float fittedDist;
      fittedDist = IR_sensors[i]->takeReading(!raw);
      Serial.print(String(fittedDist,0));
      if(i==2 || i==5)Serial.print("\n");
      else Serial.print("\t");
    }
    Serial.print("\nEOL");
  }
  
  delay(200); // slow down serial port
}


