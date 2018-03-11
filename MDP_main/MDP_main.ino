#include "IR.h"
#include "Motor.h"
#include "Calibration.h"

#define LEFT 0
#define RIGHT 1

const String DEFAULT_RPM = "105 ";

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
bool DEBUGMODE = true;

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
  calibration = new Calibration(IR_sensors, motor, DEBUGMODE);
  
  pinMode(E1A, INPUT);
  pinMode(E1B, INPUT);
  pinMode(E2A, INPUT);
  pinMode(E2B, INPUT);
  delay(1000);
//  for(int a=0;a<20;a++)
//  {String tmp = "";
//  
//    if(a==5 || a==15)
//    {
//      Serial.println("_____________");
//    }Serial.print(a+String(".\t"));
//    if(a>=5 && a<15)
//    {
//      motor->command("FORWARD 105 1");
//      delay(500);
//      motor->command("BACKWARD 105 1");
//      delay((a-5)*100);
//    }
//    for(i=0; i<6; i++)
//    {
//      int thisReading;
//      tmp += String(IR_sensors[i]->takeReading(true));
//      tmp += "\t";
//    }
//    Serial.println(tmp);
//    if(a==0 || a==19)
//    {
//      delay(200);
//    }
//  }
//____________________________________
//  for(i=0;i<15;i++)
//  {
//    motor->command("ROTATE_LEFT 50 " +String(i));
//    delay(500);
//    motor->command("ROTATE_RIGHT 20 "+String(i));
//    delay(500);
//    Serial.println(i);
//  }
//  for(i=15;i>=0;i--)
//  {
//    motor->command("ROTATE_LEFT 50 "+String(i));
//    delay(500);
//    motor->command("ROTATE_RIGHT 20 "+String(i));
//    delay(500);
//    Serial.println(i);
//  }
}

void loop() {
  // take reading, send integers to algo
  String sendToAlgo = "";
  // take dummy reading
//  for(i=0; i<6; i++)
//  {
//    IR_sensors[i]->takeReading(true);
//  }
  delay(100);
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
      sendToAlgo += " ";
    }
  }
  sendToAlgo += "\n";
  Serial.println(sendToAlgo);
  if(DEBUGMODE)Serial.print("EOL");
  // calibrate
  //calibration->calibrateRotation(true);
  // read command, parse and execute
  while(!readCommand(&in_command)); // block until command comes in
  int start = 0;
  int end = 1;
  while(end <= in_command.length()) // newline included, do not remove (do not replace <= with <)
  {
    if(in_command.charAt(start) == 'F' || in_command.charAt(start) == 'B') // instructions to be lumped
    {
      while(in_command.charAt(end) == in_command.charAt(start)) // expand instruction length until end of lump
      {
        end++;
      }
    }
    bool isHandled = executeInstruction(in_command.substring(start,end),end-start);
    if(isHandled)
    {
      start = end;
      delay(250);
    }
    end++;
  }
  in_command = "";
}
bool executeInstruction(String instr, int instr_len)
{  
  if(instr.charAt(0) == 'F')
  {
    motor->command("FORWARD "+DEFAULT_RPM+String(instr_len));
  }
  else if(instr.charAt(0) == 'B')
  {
    motor->command("BACKWARD "+DEFAULT_RPM+String(instr_len));
  }
  else if(instr == "L")
  {
    motor->command("ROTATE_LEFT "+DEFAULT_RPM+"90");
    calibration->informTurn(false);
  }
  else if(instr == "R")
  {
    motor->command("ROTATE_RIGHT "+DEFAULT_RPM+"90");
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
