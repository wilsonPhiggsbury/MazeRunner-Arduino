#include "IR.h"
#include "Motor.h"
#include "Calibration.h"

#define LEFT 0
#define RIGHT 1

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
bool DEBUGMODE = false;

IR *IR_sensors[6];
Motor *motor;
Calibration *calibration;
int i;

String in_command = "";
char lumpingInstr = 'n';
bool exploring = false;

void setup(){
  Serial.begin(9600);
  for(int i=0; i<6; i++)
  {
    IR_sensors[i] = new IR(i);
  }
  motor = new Motor(E1A, E1B, E2A, E2B);
  calibration = new Calibration(IR_sensors, motor,DEBUGMODE);
  
  pinMode(E1A, INPUT);
  pinMode(E1B, INPUT);
  pinMode(E2A, INPUT);
  pinMode(E2B, INPUT);
  delay(1000);
//  for(i=0;i<15;i++)
//  {
//    motor->command("ROTATE_LEFT 105 "+String(i));
//    delay(500);
//    motor->command("ROTATE_RIGHT 105 "+String(i));
//    delay(500);
//    Serial.println(i);
//  }
//  for(i=15;i>=0;i--)
//  {
//    motor->command("ROTATE_LEFT 105 "+String(i));
//    delay(500);
//    motor->command("ROTATE_RIGHT 105 "+String(i));
//    delay(500);
//    Serial.println(i);
//  }
}

void loop() {
  // take reading, send integers to algo
  String sendToAlgo = "";
  // take dummy reading
  for(i=0; i<6; i++)
  {
    IR_sensors[i]->takeReading(true);
  }
  delay(1000);
  // take real reading
  for(i=0; i<6; i++)
  {
    IR_sensors[i]->takeReading(true);
    int thisReading;
    if(!DEBUGMODE)
    {
      thisReading = IR_sensors[i]->reading;
      if(thisReading<0)
        thisReading = 0;
      sendToAlgo += String(round(thisReading/100.0));
    }
    else
    {
      sendToAlgo += String(IR_sensors[i]->reading);
      sendToAlgo += "\t";
    }
  }
  Serial.println(sendToAlgo);
  // calibrate
  //calibration->calibrateRotation(true);
  // read command, parse and execute
  while(!readCommand(&in_command)); // block until command comes in
  int start = 0;
  int end = 1;
  while(end <= in_command.length())
  {
    bool isHandled = executeInstruction(in_command.substring(start,end),end-start);
    if(isHandled)
    {
      start = end;
      delay(250);
    }
    end++;
  }
  in_command = "";
  if(DEBUGMODE)Serial.print("EOL");
}
bool executeInstruction(String instr, int instr_len)
{  
//   check if i am lumping instruction, and if last char of instr matches
  if(lumpingInstr != 'n')
  {
    
    if(instr.charAt(instr_len-1)==lumpingInstr)
      return false;
    else
    {
      if(lumpingInstr=='F')
        motor->command("FORWARD 60 "+String(instr_len));
      else if(lumpingInstr=='B')
        motor->command("BACKWARD 60 "+String(instr_len));
        
      lumpingInstr = 'n';
      instr = instr.charAt(instr_len-1);
    }
  }
  if(instr == "F")
  {
    if(lumpingInstr == 'n')
      lumpingInstr = 'F';
    return false;
  }
  else if(instr == "B")
  {
    if(lumpingInstr == 'n')
      lumpingInstr = 'B';
    return false;
  }
//  if(instr == "F")
//  {
//    motor->command("FORWARD 105 1");
//  }
//  else if(instr == "B")
//  {
//    motor->command("BACKWARD 105 1");
//  }
  else if(instr == "L")
  {
    motor->command("ROTATE_LEFT 105 90");
    calibration->informTurn(false);
  }
  else if(instr == "R")
  {
    motor->command("ROTATE_RIGHT 105 90");
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
  else if(instr == "N")
  {
    return true;
  }
  else if(instr == "CF0")
  {
    calibration->doCalibrationSet(0,'F');
  }
  else if(instr == "CF1")
  {
    calibration->doCalibrationSet(1,'F');
  }
  else if(instr == "CS0")
  {
    calibration->doCalibrationSet(0,'S');
  }
  else if(instr == "CS1")
  {
    calibration->doCalibrationSet(1,'S');
  }
  else if(instr== "D")
  {
    DEBUGMODE = !DEBUGMODE;
    calibration->toggleDebug();
  }
  else
    return false;
  return true;
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
