#include "Calibration.h"

// IR sensor calibrate ROTATION trusted range, in mm
#define ROTATION_RANGE_FLO -10
#define ROTATION_RANGE_FHI 120
#define ROTATION_RANGE_SLO -30 // even -40
#define ROTATION_RANGE_SHI 150
// tolerance in mm
#define ROTATION_ROUGH_TOLERANCE 25//(5~10)
#define ROTATION_FINE_TOLERANCE_SIDE 10//(5~10)
#define ROTATION_FINE_TOLERANCE_FRONT 7
//______________________________________________________
// IR sensor calibrate DISPLACEMENT trusted range, in mm
#define DISPLACEMENT_RANGE_FRONT 210 // SUBJ TO CHANGE (<)
#define DISPLACEMENT_RANGE_SIDE 150
// tolerance in mm
#define DISPLACEMENT_ROUGH_TOLERANCE 30//20//(5~20)
#define DISPLACEMENT_FINE_TOLERANCE 15//8

/*
 * tweak up if jittering occurs (hardware cannot support such fine tuning)
 * tweak down if calibration results not accurate enough
 */

// for atan2 function
#define FRONT_SENSOR_WIDTH 168
#define SIDE_SENSOR_WIDTH 194

const short DEFAULT_RPM = 105;
const short SLOW_RPM = 20;

// public ---------------------------------------------------------------
Calibration::Calibration(IR *IR_sensors[6], Motor *motor, bool debug)
{
	this->displacement_fixLater = 0;
	for (int i=0; i<6; i++)
	{
		(this->IR_sensors)[i] = IR_sensors[i];
	}
	this->motor = motor;
	this->DEBUG = debug;
  sensor1 = sensor2 = -1;
}
int Calibration::doCalibrationSet(char front_or_side, int s1, int s2, int s3)
{
  sensorOffsets[FL] = sensorOffsets[S_FR] = s1*100;
  sensorOffsets[FM] = s2*100;
  sensorOffsets[FR] = sensorOffsets[S_BR] = s3*100;
  const int invalid = 9;
  // sensor choice
  if(front_or_side == 'F')
  {
    if(s1!=invalid && s3!=invalid) // 000 or 090
    {
      sensor1 = FL;
      sensor2 = FR;
      if(s2!=invalid)trustedSensorForDist = FM;
      else trustedSensorForDist = FL;
    }
    else if(s1!=invalid && s2!=invalid) // 009
    {
      sensor1 = FL;
      sensor2 = FM;
      trustedSensorForDist = FL;
    }
    else if(s2!=invalid && s3!=invalid)// 900
    {
      sensor1 = FM;
      sensor2 = FR;
      trustedSensorForDist = FR;
    }
    else // 990 or 909 or 099
    {
      sensor1 = sensor2 = -1;
      if(s1!=invalid) trustedSensorForDist = FL;
      else if(s2!=invalid) trustedSensorForDist = FM;
      else if(s3!=invalid)trustedSensorForDist = FR;
      else trustedSensorForDist = -1;
    }
  }
  else if(front_or_side == 'S')
  {
    sensor1 = (s1!=invalid)?S_FR:-1;
    sensor2 = (s3!=invalid)?S_BR:-1;
    if(s2!=invalid)trustedSensorForDist = FM;
    else trustedSensorForDist = FL;
  }
  else
  {
    return 0;
  }
  bool hasCalibratedRotation = false;
  bool hasCalibratedDisplacement = false;
  uint8_t limit = 0;
  // guard condition: back up if too close
  if(!sensorValid(FL,TOO_CLOSE) || !sensorValid(FM,TOO_CLOSE) || !sensorValid(FR,TOO_CLOSE))// how to read: sensor FL not valid because TOO_CLOSE
  {
    motor->moveBackward(20,0);
    do{
      updateReadings(IMMEDIATE);
    }while(!sensorValid(FL,TOO_CLOSE) || !sensorValid(FM,TOO_CLOSE) || !sensorValid(FR,TOO_CLOSE));
    delay(250);// continue moving backwards for a while even after sensor goes valid
    motor->stopBot();
    updateReadings(WAIT);
  }
  // guard condition: abort if one of the sensors invalid
  if(front_or_side=='F' && (!sensorValid(sensor1,CLOSE_AND_FAR) || !sensorValid(sensor2,CLOSE_AND_FAR)))return -1;
  // main routine: do rotation and displacement calibration until no need anymore (or exceeds limit)
  do
  {
    // check rotation
    if(sensor1!=-1 && sensor2!=-1)hasCalibratedRotation = calibrateRotation(front_or_side);
    // check displacement
    if(trustedSensorForDist != -1)hasCalibratedDisplacement = calibrateDisplacement(front_or_side);
    limit++;
  }while((hasCalibratedRotation || hasCalibratedDisplacement) && limit<3);
	
	return 1;
}
void Calibration::informTurn(bool right)
{
//  delay(250);
//	if(displacement_fixLater != 0)
//	{
//		if(right){
//		  motor->moveBackward(DEFAULT_RPM, float(displacement_fixLater/100.0,2));
//		}
//		else {
//		  motor->moveForward(DEFAULT_RPM, float(displacement_fixLater/100.0,2));
//		}
//		displacement_fixLater = 0;
//	}
}
void Calibration::toggleDebug()
{
	this->DEBUG = !this->DEBUG;
}
// private ---------------------------------------------------------------
bool Calibration::calibrateRotation(char front_or_side)
{
	updateReadings(WAIT);
	int sensorwidth,diff,diff2,tolerance;
	float turnDegree, toleranceScale;
	String lr_command;
	if(front_or_side == 'F')
	{
    tolerance = ROTATION_FINE_TOLERANCE_FRONT;
		toleranceScale = 1;
	}
	else if(front_or_side == 'S')
	{
    tolerance = ROTATION_FINE_TOLERANCE_SIDE;
		sensor1 = S_FR;
		sensor2 = S_BR;
		sensorwidth = SIDE_SENSOR_WIDTH;
		toleranceScale = 1;
	}
	// _________________________________________ do fine calibration
	diff = getReadings(sensor1) - getReadings(sensor2);
	bool isRight = diff > 0;
	if(abs(diff) > tolerance*toleranceScale)
	{
		//if(DEBUG)Serial.println(lr_command + SLOW_RPM);
		if(diff > 0) {
      motor->rotateLeft(SLOW_RPM, 0);
      do{
        updateReadings(IMMEDIATE);
        diff2 = getReadings(sensor1) - getReadings(sensor2);
      }while(diff2 <= diff);
      motor->stopBot();
			motor->rotateRight(SLOW_RPM, 0);
		}
		else {
      motor->rotateRight(SLOW_RPM, 0);
      do{
        updateReadings(IMMEDIATE);
        diff2 = getReadings(sensor1) - getReadings(sensor2);
      }while(diff2 >= diff);
      motor->stopBot();
			motor->rotateLeft(SLOW_RPM, 0);
		}
    do
    {
      updateReadings(IMMEDIATE);
      diff = getReadings(sensor1) - getReadings(sensor2);
    }while(abs(diff) > tolerance && (isRight == (diff>0)));
    if(DEBUG && (isRight != (diff>0)))Serial.println("Rot overshoot");
    motor->stopBot();
    return true;
	}
	else
		return false;

}

bool Calibration::calibrateDisplacement(char front_or_side)
{	
	// update readings, usually executed after rotated back to spot
  delay(200);
	updateReadings(WAIT);
	int diff;
  int tolerance_far = DISPLACEMENT_FINE_TOLERANCE;
  int tolerance_near = -DISPLACEMENT_FINE_TOLERANCE;
	float toleranceScale;
	if(front_or_side == 'F')
	{
    bool isMovingFront;
		diff = getReadings(trustedSensorForDist);
		toleranceScale = 1;
		
		// _________________________________________ do fine calibration
    isMovingFront = diff>0;
		if(diff > tolerance_far || diff < tolerance_near)
		{
			if(diff > 0) {
				motor->moveForward(SLOW_RPM, 0);
			}
			else {
				motor->moveBackward(SLOW_RPM, 0);
			}
      do
      {
        updateReadings(IMMEDIATE);
        diff = getReadings(trustedSensorForDist);
        motor->adjustSpeed(diff>0);
      }while((diff > tolerance_far+15 || diff < tolerance_near) && (isMovingFront == (diff>0))); // +15 to make robot stop earlier, else always overshoots
      if(DEBUG && (isMovingFront != (diff>0)))Serial.println("Displ overshoot");
      motor->stopBot();
      return true;
  	}
		else
			return false;
		
	}
	else if(front_or_side == 'S')
	{//if(DEBUG)Serial.println("DISPL Side");
		toleranceScale = 1;
		if(sensor1 != -1 && sensor2 != -1)displacement_fixLater = (getReadings(sensor1) + getReadings(sensor2))/2;
    else if(sensor1 != -1)displacement_fixLater = getReadings(sensor1);
    else if(sensor2 != -1)displacement_fixLater = getReadings(sensor2);
    else return false;
		
		if(displacement_fixLater > tolerance_far || displacement_fixLater < tolerance_near-5) // -5 to allow nearer, prevent turning and realizing no need to back off
		{
			motor->rotateRight(105, 90);
      sensor1 = FL;
      sensor2 = FR;
			calibrateDisplacement('F');
      sensor1 = S_FR;
      sensor2 = S_BR;
			delay(200);
			motor->rotateLeft(105, 90);
      return true;
		}
    else
    {
     return false;
    }
	}
}
// utilities
void Calibration::updateReadings(bool wait)
{
	if(wait)delay(200);
	for(int i=0; i<6; i++)
	{
	  IR_sensors[i]->takeReading(true);
	}
}
bool Calibration::sensorValid(int sensorID, int8_t closeOrFar)
{
  bool notTooClose = IR_sensors[sensorID]->reading != -900;
  bool notTooFar = IR_sensors[sensorID]->reading != 900;
  if(closeOrFar == -1)return notTooClose; // if not too close then valid
  else if(closeOrFar == 1) return notTooFar;// if not too far then valid
  else return notTooClose && notTooFar; // only valid if not too close nor too far
}
int Calibration::getReadings(int sensorIndex)
{
  return IR_sensors[sensorIndex]->reading - sensorOffsets[sensorIndex];
}

