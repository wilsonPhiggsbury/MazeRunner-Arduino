// Voltage: 6.3

#include "Calibration.h"
#include "Utilities.h"

#define LEFT 0
#define RIGHT 1

const float DEFAULT_RPM = 105;

/*
   Algo set of commands:
   F
   L
   R
   B
   ES
   EE
   N
*/

bool DEBUGMODE = false;
bool raw = false;

IR *IR_sensors[6];
Motor *motor;
Calibration *calibration;

String in_command = "";
char lumpingInstr = 'n';
bool exploring = false;

void setup() {
  motor = new Motor();
  Serial.begin(9600);
  for (int i = 0; i < 6; i++)
  {
    IR_sensors[i] = new IR(i);
  }
  calibration = new Calibration(IR_sensors, motor, DEBUGMODE);
  delay(500);
}

void loop() {
  // take reading, send integers to algo
  String sendToAlgo = "";
  int i,j;
  delay(150);

  for (i = 0; i < 6; i++)
  {
    if (i == 1)j = 0;
    else if (i == 0)j = 1;
    else j = i;
    IR_sensors[j]->takeReading(!raw);
    int thisReading;
    if (!DEBUGMODE && !raw)
    {
      thisReading = IR_sensors[j]->reading;
      if (thisReading < 0)
        thisReading = 0;
      sendToAlgo += String(round(float(thisReading) / 100.0));
    }
    else
    {
      sendToAlgo += String(IR_sensors[j]->reading);
      sendToAlgo += " ";
    }
  }
  sendToAlgo += "\n";
  Serial.println(sendToAlgo);

  // read command, parse and execute
  while (!readCommand(&in_command)); // block until command comes in
  i=0;
  j=0;
  String sub_command = Utilities::getSubString(in_command, ',', i++);
  while(sub_command != "")
  {
    String instr,parameter;
    while(sub_command.charAt(j)!=' ')
    {
      instr += sub_command.charAt(j);
      j++;
    }
    parameter = sub_command.substring(j+1,sub_command.length());
    executeInstruction(instr, parameter);
    sub_command = Utilities::getSubString(in_command, ',', i++);
  }
  
  in_command = "";
}
bool executeInstruction(String instr, String parameter)
{
  int param1 = parameter.toInt();
  Serial.println(instr+" "+parameter);
  if (instr == "F")
  {
    motor->moveForward(DEFAULT_RPM, param1);
  }
  else if (instr == "B")
  {
    motor->moveBackward(DEFAULT_RPM, param1);
  }
  else if (instr == "L")
  {
    while(param1>0)
    {
      motor->rotateLeft(DEFAULT_RPM, 90);
      calibration->informTurn(false);
      param1--;
      if(param1!=0)delay(250);
    }
  }
  else if (instr == "R")
  {
    while(param1>0)
    {
      motor->rotateRight(DEFAULT_RPM, 90);
      calibration->informTurn(true);
      param1--;
      if(param1!=0)delay(250);
    }
  }
  else if (instr == "ES")
  {
    exploring = true;
  }
  else if (instr == "EE")
  {
    exploring = false;
  }
  else if (instr == "N")
  {
    return true;
  }
  else if (instr == "CF")
  {
    calibration->doCalibrationSet(0, 'F', param1/100, (param1%100)/10, (param1%10));
  }
  else if (instr == "CS")
  {
    calibration->doCalibrationSet(param1, 'S', false, false, false);
  }
  else if (instr == "D")
  {
    DEBUGMODE = !DEBUGMODE;
    calibration->toggleDebug();
  }
  else if (instr == "Q")
  {
    raw = !raw;
  }
  else
    return false;
  return true;
}
bool readCommand(String *readVal)
{
  char tmp;
  while (Serial.available())
  {
    tmp = Serial.read();
    (*readVal) += tmp;
    // append char to readVal until encounter newline
    if (tmp != '\n')
      continue;
    else
      return true; // stop blocking
  }
  // block if nothing comes in
  return false;
}
