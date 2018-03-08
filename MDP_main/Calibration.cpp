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
#define CALIBRATE_DISPLACEMENT_RANGE 1.5
// IR sensor calibration tolerance
#define CALIBRATE_ROTATION_TOLERANCE 0.08//(0.05~0.1)
#define CALIBRATE_DISPLACEMENT_TOLERANCE 0.1//(0.05~0.2)
/*
 * low tolerance: better percision but may jitter
 * high tolerance: worse precision
 */

// for atan2 function
#define FRONT_SENSOR_WIDTH 6.8
#define SIDE_SENSOR_WIDTH 19.4


Calibration::Calibration(IR *IR_sensors[6], Motor *motor)
{
	this->displacement_fixNow = 0; // act - cur, (+: ahead of point, -: behind of point)
	this->displacement_fixLater = 0;// act - cur, (+: right of point -: left of point)
	for (int i=0; i<6; i++)
	{
		(this->IR_sensors)[i] = IR_sensors[i];
	}
	this->motor = motor;
}

int Calibration::calibrateRotation(bool useFront)
{
  // update readings, because it may be executed after slight adjustment (to get in range)
  for(int i=0; i<6; i++)
  {
    IR_sensors[i]->takeReading(true);
  }
	float dist, tolerance, diff, turnDegree;
	// Determine tolerance values
	if(useFront)
	{
    Serial.println("_______________CALIBRATING ROTATION USING FRONT_________________");
    dist =  IR_sensors[FM]->reading;
		if(dist > CALIBRATE_ROTATION_RANGE_FLO && dist < CALIBRATE_ROTATION_RANGE_FHI)
		{
			if(dist > 0.0 && dist <0.5)
				tolerance = CALIBRATE_ROTATION_TOLERANCE + 0.05*(dist*2);
			else if(dist >= 0.5)
				tolerance = CALIBRATE_ROTATION_TOLERANCE + 0.05;
			else
				tolerance = CALIBRATE_ROTATION_TOLERANCE;

			diff = abs(IR_sensors[FL]->reading - IR_sensors[FR]->reading);
      Serial.print("Difference: "+String(diff,2)+" Tolerance: "+String(tolerance,2));
			if(diff < tolerance)
      {
        Serial.println("\nFail to calibrate rotation, already straight.");
        return -1;
      }
			turnDegree = atan2(diff,FRONT_SENSOR_WIDTH) / PI * 180;
      Serial.println(" Turn Degree: "+String(turnDegree,2));
			if(IR_sensors[FL]->reading < IR_sensors[FR]->reading)
				motor->command("ROTATE_LEFT 80 "+String(turnDegree,2));
			else
				motor->command("ROTATE_RIGHT 80 "+String(turnDegree,2));

			return 1;
		}
		else
		{
      if(dist > CALIBRATE_ROTATION_RANGE_FLO)
        Serial.println("\nFail to calibrate rotation, sensors too FAR from obstacle.");
      else
        Serial.println("\nFail to calibrate rotation, sensors too NEAR from obstacle.");
		  return 0;
		}
	}
	else
	{
    Serial.println("_______________CALIBRATING ROTATION USING SIDE_________________");
    dist = (IR_sensors[S_FR]->reading + IR_sensors[S_BR]->reading)/2;
		if(dist > CALIBRATE_ROTATION_RANGE_SLO && dist < CALIBRATE_ROTATION_RANGE_SHI)
		{
			if(dist > 0.5 && dist < 1.0)
				tolerance = CALIBRATE_ROTATION_TOLERANCE + 0.05*((dist-0.5)*2);
			else if(dist >= 1.0)
				tolerance = CALIBRATE_ROTATION_TOLERANCE + 0.05;
			else
				tolerance = CALIBRATE_ROTATION_TOLERANCE;

			diff = abs(IR_sensors[S_FR]->reading - IR_sensors[S_BR]->reading);
      
      Serial.print("Difference: "+String(diff,2)+" Tolerance: "+String(tolerance,2));
			if(diff < tolerance)
			{
        Serial.println(" Fail to calibrate rotation, already straight.");
			  return -1;
			}
			turnDegree = atan2(diff,SIDE_SENSOR_WIDTH) / PI * 180;
      Serial.println(" Turn Degree: "+String(turnDegree,2));
			if(IR_sensors[S_FR]->reading < IR_sensors[S_BR]->reading)
				motor->command("ROTATE_LEFT 80 "+String(turnDegree,2));
			else
				motor->command("ROTATE_RIGHT 80 "+String(turnDegree,2));

			return 1;
		}
		else
    {
      Serial.println(" Fail to calibrate rotation, sensors too far from obstacle.");
      return 0;
    }
	}
}
void Calibration::informTurn(bool right)
{
	/*port fixLater to fixNow
  * correct fixNow and clear fixLater
 	*/
	displacement_fixNow = displacement_fixLater * (right? 1:-1);
	displacement_fixLater = 0;
   Serial.println("____TRANSFERRING DISPLACEMENT _____" + String(displacement_fixNow,2));
  fixDisplacement(true);
}
void Calibration::calibrateDisplacement()
{
  /* condition to fufil before updating front displacement  (displacement_fixNow):
  *   all 3 sensors agree, distance is either 0 or 1
  * condition to fufil before updating side displacement    (displacement_fixLater):
  *   2 short side sensors agree, distance is either 0 or 1
  *  
  * For front sensors, trust sensor FM
  * FOr side sensors, trust sensor S_BR
  */
  Serial.println("____TAKING READINGS TO DETERMINE DISPLACEMENT___");
  if(round(IR_sensors[FM]->reading)<=CALIBRATE_DISPLACEMENT_RANGE)
  {
    displacement_fixNow = round(IR_sensors[FM]->reading) - IR_sensors[FM]->reading;
    Serial.println("Distance to Fix: " + String(displacement_fixNow,4));
    fixDisplacement(true);
  }
 else
 {
  Serial.println("Dist to Fix is not updated: front sensor is too far from reliable range!");
 }
  if(round(IR_sensors[S_FR]->reading)<=CALIBRATE_DISPLACEMENT_RANGE && round(IR_sensors[S_BR]->reading)<=CALIBRATE_DISPLACEMENT_RANGE)
  {
    displacement_fixLater = round(IR_sensors[S_BR]->reading) - IR_sensors[S_BR]->reading;
   Serial.println("Distance to Fix LATER: " + String(displacement_fixLater,4));
  }

  
}
void Calibration::calibrateDisplacement_forRotation()
{
  // only for front sensors!
  Serial.println("____ADJUSTING DISPLACEMENT BEFORE ROTATION____");
  if(round(IR_sensors[FM]->reading)<=CALIBRATE_DISPLACEMENT_RANGE)
  {
    if(IR_sensors[FM]->reading <= CALIBRATE_ROTATION_RANGE_FLO)
    {
      Serial.println("Need to retreat.");
      displacement_fixNow = CALIBRATE_ROTATION_RANGE_FLO-IR_sensors[FM]->reading;
    }
    else
    {
      Serial.println("Need to advance.");
      displacement_fixNow = CALIBRATE_ROTATION_RANGE_FHI-IR_sensors[FM]->reading;
    } 
    Serial.println("Distance to Fix: " + String(displacement_fixNow,4));
    fixDisplacement(false);
  }
  else
  {
    Serial.println("Dist to Fix is not updated: front sensor is too far from reliable range!");
  }
}
void Calibration::fixDisplacement(bool useTolerance)
{
  // fix errors recorded by displacement_fixNow
  
  Serial.println("________FIXING DISPLACEMENT _________________");
  const float tolerance = useTolerance? CALIBRATE_DISPLACEMENT_TOLERANCE:0;
  if(!useTolerance)
  {
    if(displacement_fixNow > 0)
      displacement_fixNow += 0.2;
    else
      displacement_fixNow -= 0.2;
  }
  if(displacement_fixNow > tolerance)
  {
    motor->command("BACKWARD 60 " + String(abs(displacement_fixNow)));
    Serial.println("Distance is fixed by " + String(displacement_fixNow)+".");
  }
  else if(displacement_fixNow < -tolerance)
  {
    motor->command("FORWARD 60 " + String(abs(displacement_fixNow)));
    Serial.println("Distance is fixed by " + String(displacement_fixNow)+".");
  }
  else
  {
    Serial.println("Distance is within tolerance.");
  }
  displacement_fixNow = 0;
}

