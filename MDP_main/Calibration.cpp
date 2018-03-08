#include "IR.h"
#include "Motor.h"
#include <math.h>
#include "Calibration.h"
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
// IR sensor indexes
#define FL 0// front left
#define FM 1// front mid
#define FR 2// front right
#define S_FL 3// left front (side)
#define S_FR 4// right front (side)
#define S_BR 5// right back (side)

// IR sensor calibration trusted range
#define CALIBRATE_ROTATION_RANGE_FLO -0.15
#define CALIBRATE_ROTATION_RANGE_FHI 1.2
#define CALIBRATE_ROTATION_RANGE_SLO -0.2
#define CALIBRATE_ROTATION_RANGE_SHI 1.1

#define CALIBRATE_DISPLACEMENT_RANGE_FRONT 2.1 // SUBJ TO CHANGE (<)
#define CALIBRATE_DISPLACEMENT_RANGE_SIDE 1.5


// IR sensor calibration tolerance
#define CALIBRATE_ROTATION_TOLERANCE 0.08//(0.05~0.1)
#define CALIBRATE_DISPLACEMENT_TOLERANCE 0.1//(0.05~0.2)
/*
 * tweak up if jittering occurs (hardware cannot support such fine tuning)
 * tweak down if calibration results not accurate enough
 */

// for atan2 function
#define FRONT_SENSOR_WIDTH 16.8
#define SIDE_SENSOR_WIDTH 19.4

// public ---------------------------------------------------------------
Calibration::Calibration(IR *IR_sensors[6], Motor *motor, bool debug)
{
	this->displacement_fixNow = 0; // act - cur, (+: ahead of point, -: behind of point)
	this->displacement_fixLater = 0;// act - cur, (+: right of point -: left of point)
	for (int i=0; i<6; i++)
	{
		(this->IR_sensors)[i] = IR_sensors[i];
	}
	this->motor = motor;
  this->DEBUG = debug;
}
int Calibration::doCalibrationSet(int distInTheory, char front_or_side)
{
  int limit = 3;
  bool hasRotated = calibrateRotation(front_or_side);
  delay(50);
  while(front_or_side=='F' && !hasRotated && limit>0)
  {
    calibrateDisplacement_forRotation();
    delay(50);
    hasRotated = calibrateRotation(front_or_side);
    delay(50);
    limit--;
  }
  if(limit<=0 && DEBUG)Serial.println("--Give up trying to rotate.--");
  calibrateDisplacement(distInTheory); 
}
void Calibration::informTurn(bool right)
{
  /*port fixLater to fixNow
  * correct fixNow and clear fixLater
  */
  displacement_fixNow = displacement_fixLater * (right? 1:-1);
  displacement_fixLater = 0;
  fixDisplacement(true);
}
void Calibration::toggleDebug()
{
  this->DEBUG = !this->DEBUG;
}
// private ---------------------------------------------------------------
bool Calibration::calibrateRotation(char front_or_side)
{
  // update readings, because it may be executed after slight adjustment (to get in range)
  updateReadings();
  // call subroutine and pass in all required parameters
	if(front_or_side=='F')
    return calibrateRotation_subroutine(CALIBRATE_ROTATION_RANGE_FLO,CALIBRATE_ROTATION_RANGE_FHI,0,0.5,FL,FR,FRONT_SENSOR_WIDTH,IR_sensors[FM]->reading);
	else if(front_or_side=='S')
    return calibrateRotation_subroutine(CALIBRATE_ROTATION_RANGE_SLO,CALIBRATE_ROTATION_RANGE_SHI,0.5,1,S_FR,S_BR,SIDE_SENSOR_WIDTH,(IR_sensors[S_FR]->reading+IR_sensors[S_BR]->reading)/2);
}

void Calibration::calibrateDisplacement(int distToObstacle)
{
  /* condition to fufil before updating front displacement  (displacement_fixNow):
  *   front middle sensor indicates distance is between 0 and 1
  * condition to fufil before updating side displacement    (displacement_fixLater):
  *   2 short side sensors agree, distance is between  0 and 1
  *  
  * For front sensors, trust sensor FM
  * FOr side sensors, trust sensor S_BR
  */
  // update readings, usually executed after rotated back to spot
  updateReadings();
  if(DEBUG)Serial.println("3. ___DISPLACEMENT___");
  
  if(IR_sensors[FM]->reading<=CALIBRATE_DISPLACEMENT_RANGE_FRONT)
  {
    if(distToObstacle == -1)distToObstacle = round(IR_sensors[FM]->reading);
    displacement_fixNow = distToObstacle - IR_sensors[FM]->reading;
    if(DEBUG)Serial.print("Difference: " + String(displacement_fixNow,2));
    fixDisplacement(true);
  }
  else
  {
    if(DEBUG)Serial.println("Can't fix displacement: front sensor is too far from reliable range!");
  }
  if(IR_sensors[S_FR]->reading<=CALIBRATE_DISPLACEMENT_RANGE_SIDE && IR_sensors[S_BR]->reading<=CALIBRATE_DISPLACEMENT_RANGE_SIDE)
  {
    if(distToObstacle == -1)distToObstacle = round(IR_sensors[S_BR]->reading);
    displacement_fixLater = distToObstacle - IR_sensors[S_BR]->reading;
    if(DEBUG)Serial.println("Difference (Fix LATER): " + String(displacement_fixLater,4));
  }
  else
  {
    if(DEBUG)Serial.println("Can't fix displacement: side sensor is too far from reliable range!");
  }  
}
void Calibration::calibrateDisplacement_forRotation()
{
  // only for front sensors!
  if(DEBUG)Serial.println("1. ___DISPLACEMENT (FOR ROTATION)____");
  if(round(IR_sensors[FM]->reading)<=CALIBRATE_DISPLACEMENT_RANGE_FRONT)
  {
    if(IR_sensors[FM]->reading <= CALIBRATE_ROTATION_RANGE_FLO)
    {
      if(DEBUG)Serial.println("Need to retreat by "+String(displacement_fixNow,2));
      displacement_fixNow = CALIBRATE_ROTATION_RANGE_FLO-IR_sensors[FM]->reading;
    }
    else
    {
      if(DEBUG)Serial.println("Need to advance by "+String(displacement_fixNow,2));
      displacement_fixNow = CALIBRATE_ROTATION_RANGE_FHI-IR_sensors[FM]->reading;
    } 
    fixDisplacement(false);
  }
  else
  {
    if(DEBUG)Serial.println("Can't advance: front sensor is too far from reliable range!");
  }
}
void Calibration::fixDisplacement(bool useTolerance)
{
  // fix errors recorded by displacement_fixNow
  
  const float tolerance = useTolerance? CALIBRATE_DISPLACEMENT_TOLERANCE:0;
  if(DEBUG)Serial.println(" Tolerance: "+String(tolerance));
   if(!useTolerance)
   {
     if(displacement_fixNow > 0)
       displacement_fixNow += 0.5;
     else
       displacement_fixNow -= 0.5;
   }
  if(displacement_fixNow > tolerance)
  {
    motor->command("BACKWARD 60 " + String(abs(displacement_fixNow)));
    if(DEBUG)Serial.println("Distance is fixed by " + String(displacement_fixNow)+".");
  }
  else if(displacement_fixNow < -tolerance)
  {
    motor->command("FORWARD 60 " + String(abs(displacement_fixNow)));
    if(DEBUG)Serial.println("Distance is fixed by " + String(displacement_fixNow)+".");
  }
  else
  {
    if(DEBUG)Serial.println("Distance is within tolerance.");
  }
  displacement_fixNow = 0;
}
bool Calibration::calibrateRotation_subroutine(float guardbound_l, float guardbound_h, float scalebound_l, float scalebound_h, int sensor1, int sensor2, float sensorwidth, float dist)
{
  float tolerance, diff, turnDegree;
  // Determine tolerance values
  if(DEBUG)Serial.println("2. ___CALIBRATING ROTATION___");
  if(dist > guardbound_l && dist < guardbound_h)
  {
    if(dist > scalebound_l && dist <scalebound_h)
      tolerance = CALIBRATE_ROTATION_TOLERANCE + 0.05*((dist-scalebound_l)*2);
    else if(dist >= scalebound_h)
      tolerance = CALIBRATE_ROTATION_TOLERANCE + 0.05;
    else
      tolerance = CALIBRATE_ROTATION_TOLERANCE;

    diff = abs(IR_sensors[sensor1]->reading - IR_sensors[sensor2]->reading);
    if(DEBUG)Serial.print("Difference: "+String(diff,2)+" Tolerance: "+String(tolerance,2));
    while(diff >= tolerance)
    {
      turnDegree = atan2(diff,sensorwidth) / PI * 180;
      if(DEBUG)Serial.println("\nNeed calibrate. Turn Degree: "+String(turnDegree,2));
      turnDegree += 10;
      if(IR_sensors[sensor1]->reading < IR_sensors[sensor2]->reading)
        motor->command("ROTATE_LEFT 80 "+String(turnDegree,2));
      else
        motor->command("ROTATE_RIGHT 80 "+String(turnDegree,2));

      delay(200);
      IR_sensors[sensor1]->takeReading(true);
      IR_sensors[sensor2]->takeReading(true);
      diff = abs(IR_sensors[sensor1]->reading - IR_sensors[sensor2]->reading);
    }
    
    if(DEBUG)Serial.println("\nNo need calibrate, already straight.");
    return true;
  }
  else
  {
    if(dist > guardbound_h)
      if(DEBUG)Serial.println("\nFail to calibrate rotation, sensors too FAR from obstacle.");
    else
      if(DEBUG)Serial.println("\nFail to calibrate rotation, sensors too NEAR from obstacle.");
    return false;
  }
}
void Calibration::updateReadings()
{
  for(int i=0; i<6; i++)
    IR_sensors[i]->takeReading(true);
}

