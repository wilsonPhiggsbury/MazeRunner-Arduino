#include "Motor.h"
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

Motor::Motor()
{
    pinMode(E1A, INPUT);
    pinMode(E1B, INPUT);
    pinMode(E2A, INPUT);
    pinMode(E2B, INPUT);
    attachInterrupt(digitalPinToInterrupt(E1A), incrementTick, RISING);
    md.init();
    this->desired_rpm = 0;
    this->input_rpm_e1 = 0;
    this->input_rpm_e2 = 0;
    this->error_e1 = 0;
    this->error_e2 = 0;
    this->last_error_e1 = 0;
    this->last_error_e2 = 0;
    this->last_last_error_e1 = 0;
    this->last_last_error_e2 = 0;
    this->motor_status = COMM_FORWARD;
    tick = 0;
}

void Motor::moveForward(float input_rpm, float cell_num)
{
  this->motor_status = COMM_FORWARD;
  this->desired_rpm = input_rpm;
  this->input_rpm_e1 = input_rpm;
  this->input_rpm_e2 = input_rpm;
  unsigned long correction = this->getCorrection(cell_num);
  tick = 0;
  md.setSpeeds(rpmToSpeed(this->input_rpm_e2, false), rpmToSpeed(this->input_rpm_e1, true));
  for(int i = 0; i < cell_num; i++) {
    while(tick < CPC){
      this->adjustSpeed(true);
    }
    tick = 0;
  }
  if(cell_num > 0) {
    delay(correction);
    md.setBrakes(400,400);
    this->resetError();
  } 
}

void Motor::moveBackward(float input_rpm, float cell_num)
{
  this->motor_status = COMM_BACKWARD;
  this->desired_rpm = input_rpm;
  this->input_rpm_e1 = input_rpm;
  this->input_rpm_e2 = input_rpm;
  unsigned long correction = this->getCorrection(cell_num);
  tick = 0;
  md.setSpeeds(-1*rpmToSpeed(this->input_rpm_e2, false), -1*rpmToSpeed(this->input_rpm_e1, true));
  for(int i = 0; i < cell_num; i++) {
    while(tick < CPC){
      this->adjustSpeed(false);
      if(tick > CPC) {
        tick = 0;
        break;
      }
    } 
  }
  if(cell_num > 0) {
    delay(correction);
    md.setBrakes(400,400);
    this->resetError();
  } 
}

unsigned long Motor::getCorrection(int num_cells)
{
  switch(num_cells) {
    case 1:
      return 0;
    default:
      return 15*num_cells-20;  
  }
}

void Motor::rotateRight(float input_rpm, float degree)
{
  this->motor_status = COMM_ROTATE_R;
  uint8_t tickPeriod = getRotateTime(input_rpm, degree, true);
  tick = 0;
  md.setSpeeds(rpmToSpeed(input_rpm, false), -1*rpmToSpeed(input_rpm, true));
  if(tickPeriod != 0) {
    while(1) {
      if(tick>tickPeriod) {
        break;
      }
    }
    md.setBrakes(400,400);
  }
}

void Motor::rotateLeft(float input_rpm, float degree)
{
  this->motor_status = COMM_ROTATE_L;
  uint8_t tickPeriod = getRotateTime(input_rpm, degree, false);
  tick = 0;
  md.setSpeeds(-1*rpmToSpeed(input_rpm, false), rpmToSpeed(input_rpm, true));
  if(tickPeriod != 0) {
    while(1) {
      if(tick>tickPeriod) {
        break;
      }
    }
    md.setBrakes(400,400);
  }
}

void Motor::stopBot()
{
  md.setBrakes(400,400);
  this->resetError();
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
    if(this->motor_status == COMM_FORWARD)
    {
        // get reading
        for (int i=0; i<NUM_SAMPLES; i++) {
          e1a_readings[i] = pulseIn(E1A, HIGH, 8000); //timeout 8000 microsecs
          e1b_readings[i] = pulseIn(E1B, HIGH, 8000);
          e2a_readings[i] = pulseIn(E2A, HIGH, 8000);
          e2b_readings[i] = pulseIn(E2B, HIGH, 8000);
        }
        float e1a_reading = getRpm(e1a_readings);
        float e1b_reading = getRpm(e1b_readings);
        float e2a_reading = getRpm(e2a_readings);
        float e2b_reading = getRpm(e2b_readings);

        float e1_reading = ((e1a_reading + e1b_reading) / 2.0) + e1_offset;
        float e2_reading = ((e2a_reading + e2b_reading) / 2.0) + e2_offset;

        //Serial.println(String(e1_reading) + "    " + String(e2_reading));
        
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

uint8_t Motor::getRotateTime(float rpm, float degree, bool isRight) {
  if(degree == 0){
    return 0;
  }
  int offset;
  if(isRight)
  {
    offset = 0;
  }
  else
  {
    offset = -2;
  }
  int tickPeriod = (degree/90)*CPR + offset;
  return tickPeriod;
}

void Motor::resetError()
{
  this->last_last_error_e1 = 0;
  this->last_last_error_e2 = 0;
  this->last_error_e1 = 0;
  this->last_error_e2 = 0;
  this->error_e1 = 0;
  this->error_e2 = 0;
}

void incrementTick() {
  half_tick++;
  if(half_tick == 2) {
    tick++;
    half_tick = 0;
  }
}

