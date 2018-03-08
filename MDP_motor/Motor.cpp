#include "Motor.h"
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

Motor::Motor(int E1A, int E1B, int E2A, int E2B)
{
    md.init();
    this->desired_rpm = 0;
    this->input_rpm_e1 = 0;
    this->input_rpm_e2 = 0;
    this->E1A = E1A;
    this->E1B = E1A;
    this->E2A = E2A;
    this->E2B = E1A;
    this->error_e1 = 0;
    this->error_e2 = 0;
    this->last_error_e1 = 0;
    this->last_error_e2 = 0;
    this->last_last_error_e1 = 0;
    this->last_last_error_e2 = 0;
    this->motor_status = "FORWARD";
    this->startTime = 0;
    this->commandPeriod = 0;
}

void Motor::command(String fullCommand)
{
    String command = this->getSubString(fullCommand, ' ', 0);
    this->commandPeriod = getPeriod(fullCommand);
	  this->motor_status = command;
    if(command == "TURN_RIGHT")
    {
        md.setSpeeds(rpmToSpeed(80, false), -1*rpmToSpeed(80, true));
        delay(this->commandPeriod);
        md.setBrakes(rpmToSpeed(this->input_rpm_e2, false), rpmToSpeed(this->input_rpm_e1, true));
    }
    else if(command == "TURN_LEFT")
    {
        md.setSpeeds(-1*rpmToSpeed(80, false), rpmToSpeed(80, true));
        delay(this->commandPeriod);
        md.setBrakes(rpmToSpeed(this->input_rpm_e2, false), rpmToSpeed(this->input_rpm_e1, true));
    }
    else if(command == "FORWARD")
    {
        float rpm =  getSubString(fullCommand, ' ', 1).toFloat();
        this->moveForward(rpm);
        this->startTime = millis();
        while(millis() < this->startTime + this->commandPeriod) {
          this->adjustSpeed(true);
        }
        md.setBrakes(400,400);
        //md.setBrakes(rpmToSpeed(this->input_rpm_e2, false), rpmToSpeed(this->input_rpm_e1, true));
    }
    else if(command == "BACKWARD")
    {
        float rpm =  getSubString(fullCommand, ' ', 1).toFloat();
        this->moveBackward(rpm);
        this->startTime = millis();
        while(millis() < this->startTime + this->commandPeriod) {
          this->adjustSpeed(false);
        }
        md.setBrakes(400,400);
        //md.setBrakes(rpmToSpeed(this->input_rpm_e2, false), rpmToSpeed(this->input_rpm_e1, true));
    }
    else if(command == "ROTATE_RIGHT")
    {
        float rpm =  getSubString(fullCommand, ' ', 1).toFloat();
        md.setSpeeds(rpmToSpeed(rpm, false), -1*rpmToSpeed(rpm, true));
        delay(this->commandPeriod);
        md.setBrakes(400,400);      
    }
    else if(command == "ROTATE_LEFT")
    {
        float rpm =  getSubString(fullCommand, ' ', 1).toFloat();
        md.setSpeeds(-1*rpmToSpeed(rpm, false), rpmToSpeed(rpm, true));
        delay(this->commandPeriod);
        md.setBrakes(400,400);      
    }
} 

void Motor::moveForward(float input_rpm)
{
  this->motor_status = "FORWARD";
  this->desired_rpm = input_rpm;
  this->input_rpm_e1 = input_rpm;
  this->input_rpm_e2 = input_rpm;
  md.setSpeeds(rpmToSpeed(this->input_rpm_e2, false), rpmToSpeed(this->input_rpm_e1, true));
}

void Motor::moveBackward(float input_rpm)
{
  this->motor_status = "BACKWARD";
  this->desired_rpm = input_rpm;
  this->input_rpm_e1 = input_rpm;
  this->input_rpm_e2 = input_rpm;
  md.setSpeeds(-1*rpmToSpeed(this->input_rpm_e2, false), -1*rpmToSpeed(this->input_rpm_e1, true));
}

int Motor::rpmToSpeed(float rpm, boolean isE1)
{
    if(isE1)
    {
        return (rpm-E1C)/E1M;
    }
    else
    {
        return (rpm-E2C)/E2M;
    }
}

void Motor::adjustSpeed(bool isForward)
{
    if(this->motor_status == "FORWARD")
    {
        // get reading
        for (int i=0; i<NUM_SAMPLES; i++) {
          e1a_readings[i] = pulseIn(this->E1A, HIGH, 8000); //timeout 8000 microsecs
          e1b_readings[i] = pulseIn(this->E1B, HIGH, 8000);
          e2a_readings[i] = pulseIn(this->E2A, HIGH, 8000);
          e2b_readings[i] = pulseIn(this->E2B, HIGH, 8000);
        }
        float e1a_reading = getRpm(e1a_readings);
        float e1b_reading = getRpm(e1b_readings);
        float e2a_reading = getRpm(e2a_readings);
        float e2b_reading = getRpm(e2b_readings);

        float e1_reading = ((e1a_reading + e1b_reading) / 2.0) + e1_offset;
        float e2_reading = ((e2a_reading + e2b_reading) / 2.0) + e2_offset;
        
        this->last_last_error_e1 = this->last_error_e1;
        this->last_last_error_e2 = this->last_error_e2;
        this->last_error_e1 = this->error_e1;
        this->last_error_e2 = this->error_e2;
        this->error_e1 = this->desired_rpm - e1_reading;
        this->error_e2 = this->desired_rpm - e2_reading;

        //PID implementation
        this->input_rpm_e1 = this->input_rpm_e1 + (k1_e1*this->error_e1) + (k2_e1*this->last_error_e1) + (k3_e1*this->last_last_error_e1);
        this->input_rpm_e2 = this->input_rpm_e2 + (k1_e2*this->error_e2) + (k2_e2*this->last_error_e2) + (k3_e2*this->last_last_error_e2);

        if(isForward)
        {
          md.setSpeeds(rpmToSpeed(this->input_rpm_e2, false), rpmToSpeed(this->input_rpm_e1, true));
        }
        else {
          md.setSpeeds(-1*rpmToSpeed(this->input_rpm_e2, false), -1*rpmToSpeed(this->input_rpm_e1, true));
        }
    }
}

unsigned int Motor::takeMedian(unsigned int nums[])
{
    int size = sizeof(nums)/sizeof(nums[0]);
        // insertion sort the nums array
    for(int i=1; i<size; i++)
    {
        unsigned int tmp;
        // swap values until previous value is not larger than next value
        int j = i;
        while(j!=0 && nums[j] < nums[j-1])
        {
          tmp = nums[j];
          nums[j] = nums[j-1];
          nums[j-1] = tmp;
          j--;
        }
    }
    // insertion sort done
    // take the middle value
    return nums[(NUM_SAMPLES-1)/2];
}

float Motor::getRpm(unsigned int readings[]) {
  unsigned int median = this->takeMedian(readings);
  if(median == 0) {
    return 0;
  }
  return RPM_CONVERSION/takeMedian(readings);
}

String Motor::getSubString(String data, char separator, int index) {
  int found = 0;
  int strIndex[] = { 0, -1 };
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
      if (data.charAt(i) == separator || i == maxIndex) {
          found++;
          strIndex[0] = strIndex[1] + 1;
          strIndex[1] = (i == maxIndex) ? i+1 : i;
      }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

long Motor::getMoveTime(float rpm, float num_cell) {
  float speed_in_cm_ms = (Pi*WHEEL_DIAMETER*rpm) / 60000;
  long moveTime = (num_cell * CELL_SIZE) / (speed_in_cm_ms);
  long offset = dis_time_m * num_cell + dis_time_c;
  return moveTime + offset;
}

long Motor::getRotateTime(float rpm, float degree, bool isRight) {
  long moveTime = (BASE_DIAMETER * degree * 1000) / (6*rpm*WHEEL_DIAMETER);
  long offset;
  if(isRight)
  {
    offset = rotate_r_m*(degree) + rotate_r_c;
  }
  else
  {
    offset = rotate_l_m*(degree) + rotate_l_c;
  }
  return moveTime + offset;
}

long Motor::getPeriod(String full_command) {
  String command = getSubString(full_command, ' ', 0);
  if(command == "FORWARD" || command == "BACKWARD")
  {
    float rpm =  getSubString(full_command, ' ', 1).toFloat();
    float num_cell =  getSubString(full_command, ' ', 2).toFloat();
    return getMoveTime(rpm, num_cell);
  }
  else if(command == "ROTATE_RIGHT")
  {
    float rpm =  getSubString(full_command, ' ', 1).toFloat();
    float degree =  getSubString(full_command, ' ', 2).toFloat();
    return getRotateTime(rpm, degree, true);
  }
  else if(command == "ROTATE_LEFT")
  {
    float rpm =  getSubString(full_command, ' ', 1).toFloat();
    float degree =  getSubString(full_command, ' ', 2).toFloat();
    return getRotateTime(rpm, degree, false);
  }
  else if(command == "TURN_RIGHT")
  {
    return 620;
  }
  else if(command == "TURN_LEFT")
  {
    return 640;
  }
  else
  {
    return getSubString(full_command, ' ', 1).toInt();
  }
}

