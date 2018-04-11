#include "Motor.h"
#include "Utilities.h"
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
  tick_LSB = tick_MSB = 0;
  for (int i = 0; i < 18; i++)
  {
    int offset = sizeof(int) * i;
    EEPROM.get(FORWARD_TICK + offset, CPC_F[i]);
    EEPROM.get(BACKWARD_TICK + offset, CPC_B[i]);
  }
  EEPROM.get(ROTATE_RIGHT_OFFSET, right_offset);
  EEPROM.get(ROTATE_LEFT_OFFSET, left_offset);
  EEPROM.get(PID_RIGHT_OFFSET, e1_offset);
  EEPROM.get(PID_LEFT_OFFSET, e2_offset);

}

void Motor::moveForward(float input_rpm, int cell_num)
{
  this->motor_status = COMM_FORWARD;
  this->desired_rpm = input_rpm;
  this->input_rpm_e1 = input_rpm;
  this->input_rpm_e2 = input_rpm;
  if (cell_num <= 0)
  {
    md.setSpeeds(rpmToSpeed(this->input_rpm_e2, false, true), rpmToSpeed(this->input_rpm_e1, true, true));
  }
  else
  {
    int tick = CPC_F[cell_num - 1];
    uint8_t CPC_MSB = (tick >> 8) & 0xFF;
    uint8_t CPC_LSB = (tick) & 0x00FF;
    md.setSpeeds(rpmToSpeed(this->input_rpm_e2, false, true), rpmToSpeed(this->input_rpm_e1, true, true));
    tick_LSB = tick_MSB = 0;

    while (tick_MSB < CPC_MSB || tick_LSB < CPC_LSB) {
      this->adjustSpeed(true);
    }
    md.setBrakes(400, 400);
    this->resetError();
  }
  delay(200); // AQEDIT: delay after motor movement

}

void Motor::moveForwardTick(float input_rpm, int tick)
{
  this->motor_status = COMM_FORWARD;
  this->desired_rpm = input_rpm;
  this->input_rpm_e1 = input_rpm;
  this->input_rpm_e2 = input_rpm;

  uint8_t CPC_MSB = (tick >> 8) & 0xFF;
  uint8_t CPC_LSB = (tick) & 0x00FF;
  md.setSpeeds(rpmToSpeed(this->input_rpm_e2, false, true), rpmToSpeed(this->input_rpm_e1, true, true));
  tick_LSB = tick_MSB = 0;
  while (tick_MSB < CPC_MSB || tick_LSB < CPC_LSB) {
    this->adjustSpeed(true);
  }
  md.setBrakes(400, 400);
  this->resetError();
}

void Motor::moveBackward(float input_rpm, int cell_num)
{
  this->motor_status = COMM_BACKWARD;
  this->desired_rpm = input_rpm;
  this->input_rpm_e1 = input_rpm;
  this->input_rpm_e2 = input_rpm;
  if (cell_num <= 0)
  {
    md.setSpeeds(rpmToSpeed(this->input_rpm_e2, false, false), rpmToSpeed(this->input_rpm_e1, true, false));
  }
  else
  {
    int tick = CPC_B[cell_num - 1];
    uint8_t CPC_MSB = (tick >> 8) & 0xFF;
    uint8_t CPC_LSB = (tick) & 0x00FF;
    md.setSpeeds(rpmToSpeed(this->input_rpm_e2, false, false), rpmToSpeed(this->input_rpm_e1, true, false));
    tick_LSB = tick_MSB = 0;
    while (tick_MSB < CPC_MSB || tick_LSB < CPC_LSB) {
      this->adjustSpeed(false);
    }
    md.setBrakes(400, 400);
    this->resetError();
  }
  delay(200); // AQEDIT: delay after motor movement

}

void Motor::moveBackwardTick(float input_rpm, int tick)
{
  this->motor_status = COMM_FORWARD;
  this->desired_rpm = input_rpm;
  this->input_rpm_e1 = input_rpm;
  this->input_rpm_e2 = input_rpm;

  uint8_t CPC_MSB = (tick >> 8) & 0xFF;
  uint8_t CPC_LSB = (tick) & 0x00FF;
  md.setSpeeds(rpmToSpeed(this->input_rpm_e2, false, false), rpmToSpeed(this->input_rpm_e1, true, false));
  tick_LSB = tick_MSB = 0;
  while (tick_MSB < CPC_MSB || tick_LSB < CPC_LSB) {
    this->adjustSpeed(false);
  }
  md.setBrakes(400, 400);
  this->resetError();
}

void Motor::rotateRight(float input_rpm, float degree)
{
  this->motor_status = COMM_ROTATE_R;
  int tickPeriod = getRotateTime(input_rpm, degree, true);
  uint8_t tickPeriod_MSB = (tickPeriod >> 8) & 0xFF;
  uint8_t tickPeriod_LSB = (tickPeriod) & 0xFF;
  md.setSpeeds(rpmToSpeed(input_rpm, false, true), rpmToSpeed(input_rpm, true, false));
  tick_MSB = tick_LSB = 0;
  if (tickPeriod != 0) {
    while (tick_MSB < tickPeriod_MSB || tick_LSB < tickPeriod_LSB) {

    }
    md.setBrakes(400, 400);
  }
  delay(200); // AQEDIT: delay after motor movement
}

void Motor::rotateLeft(float input_rpm, float degree)
{
  this->motor_status = COMM_ROTATE_L;
  int tickPeriod = getRotateTime(input_rpm, degree, false);
  uint8_t tickPeriod_MSB = (tickPeriod >> 8) & 0xFF;
  uint8_t tickPeriod_LSB = (tickPeriod) & 0xFF;
  md.setSpeeds(rpmToSpeed(input_rpm, false, false), rpmToSpeed(input_rpm, true, true));
  tick_MSB = tick_LSB = 0;
  if (tickPeriod != 0) {
    while (tick_MSB < tickPeriod_MSB || tick_LSB < tickPeriod_LSB) {

    }
    md.setBrakes(400, 400);
  }
  delay(200); // AQEDIT: delay after motor movement
}

void Motor::stopBot()
{
  md.setBrakes(400, 400);
  this->resetError();
}

int Motor::rpmToSpeed(float rpm, boolean isE1, boolean isForward)
{
  if (isForward)
  {
    if (isE1)
    {
      return (rpm - F_E1C) / F_E1M;
    }
    else
    {
      return (rpm - F_E2C) / F_E2M;
    }
  }
  else
  {
    if (isE1)
    {
      return (rpm - B_E1C) / B_E1M;
    }
    else
    {
      return (rpm - B_E2C) / B_E2M;
    }
  }

}

void Motor::adjustSpeed(bool isForward)
{
  if (this->motor_status == COMM_FORWARD)
  {
    // get reading
    for (int i = 0; i < NUM_SAMPLES; i++) {
      e1a_readings[i] = pulseIn(E1A, HIGH, 8000); //timeout 8000 microsecs
      e1b_readings[i] = pulseIn(E1B, HIGH, 8000);
      e2a_readings[i] = pulseIn(E2A, HIGH, 8000);
      e2b_readings[i] = pulseIn(E2B, HIGH, 8000);
    }
    float e1a_reading = getRpm(e1a_readings);
    float e1b_reading = getRpm(e1b_readings);
    float e2a_reading = getRpm(e2a_readings);
    float e2b_reading = getRpm(e2b_readings);

    float e1_reading = ((e1a_reading + e1b_reading) / 2.0) + e1_offset / 100.0;
    float e2_reading = ((e2a_reading + e2b_reading) / 2.0) + e2_offset / 100.0;
    //Serial.println(String(e1_reading) + "    " + String(e2_reading));

    this->last_last_error_e1 = this->last_error_e1;
    this->last_last_error_e2 = this->last_error_e2;
    this->last_error_e1 = this->error_e1;
    this->last_error_e2 = this->error_e2;
    this->error_e1 = this->desired_rpm - e1_reading;
    this->error_e2 = this->desired_rpm - e2_reading;

    //PID implementation
    this->input_rpm_e1 = this->input_rpm_e1 + (k1_e1 * this->error_e1) + (k2_e1 * this->last_error_e1) + (k3_e1 * this->last_last_error_e1);
    this->input_rpm_e2 = this->input_rpm_e2 + (k1_e2 * this->error_e2) + (k2_e2 * this->last_error_e2) + (k3_e2 * this->last_last_error_e2);

    if (isForward)
    {
      md.setSpeeds(rpmToSpeed(this->input_rpm_e2, false, true), rpmToSpeed(this->input_rpm_e1, true, true));
    }
    else {
      md.setSpeeds(rpmToSpeed(this->input_rpm_e2, false, false), rpmToSpeed(this->input_rpm_e1, true, false));
    }
  }
}


float Motor::getRpm(int readings[]) {
  int median = Utilities::takeMedian(readings, NUM_SAMPLES);
  if (median == 0) {
    return 0;
  }
  return RPM_CONVERSION / Utilities::takeMedian(readings, NUM_SAMPLES);
}

int Motor::getRotateTime(float rpm, float degree, bool isRight) {
  if (degree == 0) {
    return 0;
  }
  int tickPeriod = (degree / 90) * CPR + (isRight ? right_offset : left_offset);
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

void Motor::setForwardTick(int num_cells, int tickChange)
{
  int offset = sizeof(tickChange) * (num_cells - 1);
  CPC_F[num_cells - 1] += tickChange;
  EEPROM.put(FORWARD_TICK + offset, CPC_F[num_cells - 1]);
  Serial.println("Forward "+String(num_cells)+" cells ticks set to "+String(CPC_F[num_cells - 1])+".");
}

void Motor::setBackwardTick(int num_cells, int tickChange)
{
  int offset = sizeof(tickChange) * (num_cells - 1);
  CPC_B[num_cells - 1] += tickChange;
  EEPROM.put(BACKWARD_TICK + offset, CPC_B[num_cells - 1]);
  Serial.println("Backward "+String(num_cells)+" cells ticks set to "+String(CPC_B[num_cells - 1])+".");
}

void Motor::setRotateRightTick(int tickChange)
{
  right_offset += tickChange;
  EEPROM.put(ROTATE_RIGHT_OFFSET, right_offset);
  Serial.println("Rotate Right ticks set to "+String(right_offset)+".");
}

void Motor::setRotateLeftTick(int tickChange)
{
  left_offset += tickChange;
  EEPROM.put(ROTATE_LEFT_OFFSET, left_offset);
  Serial.println("Rotate Left ticks set to "+String(left_offset)+".");
}

void Motor::setPIDRightOffset(int offset)
{
  e1_offset += offset;
  EEPROM.put(PID_RIGHT_OFFSET, e1_offset);
  Serial.println("PID Right offset set to "+String(e1_offset)+".");
}

void Motor::setPIDLeftOffset(int offset)
{
  e2_offset += offset;
  EEPROM.put(PID_LEFT_OFFSET, e2_offset);
  Serial.println("PID Left offset set to "+String(e2_offset)+".");
}

void Motor::printInfo()
{
  int count;
  int offset;
  Serial.println("--------Lookup table for Tick/Cell");
  Serial.println("Cell\tF\tB");
  for (int num_cells = 1; num_cells < 18; num_cells++) {
    offset = sizeof(int) * (num_cells - 1);
    Serial.print(String(num_cells) + "\t");
    Serial.print(CPC_F[num_cells - 1]);
    Serial.print("\t");
    Serial.print(CPC_B[num_cells - 1]);
    Serial.print("\n");
  }
  Serial.println("--------Offsets for Rotation");
  Serial.println("\tL\tR");
  Serial.println("Rotate\t"+String(left_offset)+"\t"+String(right_offset)+"\tIncrease to rotate more.");
  Serial.println("PID\t"+String(e2_offset)+"\t"+String(e1_offset)+"\tIncrease to slow down. May be counter-intuitive.");
  Serial.println("----------------------------");


  //  Serial.println("EEPROM content: ");
  //  for(int i=0; i<100; i++)
  //  {
  //    Serial.println(String(i)+": "+String(EEPROM.read(i)));
  //  }
}

void incrementTick() {
  if (tick_LSB == 255)
  {
    tick_LSB = 0;
    tick_MSB++;
  }
  else
  {
    tick_LSB++;
  }
}
