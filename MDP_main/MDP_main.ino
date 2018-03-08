#include "IR.h"
#include "Motor.h"
#include "Calibration.h"

#define LEFT 0
#define RIGHT 1

#define DEBUGMODE 1
/*
 * Algo set of commands:
 * F
 * L
 * R
 * B
 * ES
 * EE
 * N
 */
const int E1A = 3; //right
const int E1B = 5;
const int E2A = 11; //left
const int E2B = 13;

IR *IR_sensors[6];
Motor *motor;
Calibration *calibration;
int i;
String in_command = "";

bool exploring = false;
void setup() {
  Serial.begin(9600);
  for(int i=0; i<6; i++)
  {
    IR_sensors[i] = new IR(i);
  }
  motor = new Motor(E1A, E1B, E2A, E2B);
  calibration = new Calibration(IR_sensors, motor);
  
  pinMode(E1A, INPUT);
  pinMode(E1B, INPUT);
  pinMode(E2A, INPUT);
  pinMode(E2B, INPUT);
}

void loop() {
  
  // take reading, send integers to algo
  String sendToAlgo = "";
  for(i=0; i<6; i++)
  {
    IR_sensors[i]->takeReading(true);
    int roundedVal;
    if(!DEBUGMODE)
    {
      roundedVal= round(IR_sensors[i]->reading);
      sendToAlgo += String(roundedVal);
    }
    else
    {
      sendToAlgo += String(IR_sensors[i]->reading,2);
      sendToAlgo += "\t";
    }
    
  }
  Serial.println(sendToAlgo);
  delay(500);
  // calibrate
  //calibration->calibrateRotation(true);
  // read command, parse and execute
  while(!readCommand(&in_command)); // block until command comes in
  int start = 0;
  int end = 1;
  while(end <= in_command.length())
  {
    bool isHandled = executeInstruction(in_command.substring(start,end));
    if(isHandled)
    {
      start = end;
      delay(500);
    }
    end++;
  }
  in_command = "";
}
bool executeInstruction(String instr)
{
  int hasRotated;
  
  if(instr == "F")
    motor->command("FORWARD 60 1");
  else if(instr == "B")
    motor->command("BACKWARD 60 1");
  else if(instr == "L")
  {
    motor->command("ROTATE_LEFT 80 90");
    calibration->informTurn(false);
  }
  else if(instr == "R")
  {
    motor->command("ROTATE_RIGHT 80 90");
    calibration->informTurn(true);
  }
  else if(instr == "ES")
  {
    exploring = true;
  }
  else if(instr == "EE")
  {
    exploring = false;
  }
  else if(instr == "CF0")
  {
    hasRotated = calibration->calibrateRotation(true);
    if(hasRotated == 0)
    {
      calibration->calibrateDisplacement_forRotation();
      hasRotated = calibration->calibrateRotation(true);
    }
    
    delay(100);
    calibration->calibrateDisplacement();
  }
  else if(instr == "CS0")
  {
    hasRotated = calibration->calibrateRotation(false);
    delay(100);
    calibration->calibrateDisplacement();
  }
  else if(instr == "CS1")
  {
    hasRotated = calibration->calibrateRotation(false);
    delay(100);
    calibration->calibrateDisplacement();
  }
  else if(instr == "N")
    return true;
  else
    return false;

  return true;
//  switch(instr)
//  {
//
//    return true;
//    case "C":// Calibrate front / Calibrate side
//    
//    return true;
//    case "ES":
//    exploring = true;
//    return true;
//    case "EE":
//    exploring = false;
//    return true;
//    
//    default:
//
//    return false;
//  }
}
bool readCommand(String *readVal)
{
  char tmp;
  while(Serial.available())
  {
    tmp = Serial.read();
    (*readVal) += tmp;
    // append char to readVal until encounter newline
    if(tmp != '\n')
      continue;
    else
      return true; // stop blocking
  }
  // block if nothing comes in
  return false;
}
