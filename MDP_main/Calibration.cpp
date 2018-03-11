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
#define S_FR 4// right front(side)
#define S_BR 5// right back (side)

// IR sensor calibration trusted range
#define CALIBRATE_ROTATION_RANGE_FLO -15
#define CALIBRATE_ROTATION_RANGE_FHI 120
#define CALIBRATE_ROTATION_RANGE_SLO -20
#define CALIBRATE_ROTATION_RANGE_SHI 110

#define CALIBRATE_DISPLACEMENT_RANGE_FRONT 210 // SUBJ TO CHANGE (<)
#define CALIBRATE_DISPLACEMENT_RANGE_SIDE 150


// IR sensor calibration tolerance
#define CALIBRATE_ROTATION_TOLERANCE 5//(5~10)
#define CALIBRATE_DISPLACEMENT_TOLERANCE 10//(5~20)
/*
 * tweak up if jittering occurs (hardware cannot support such fine tuning)
 * tweak down if calibration results not accurate enough
 */

// for atan2 function
#define FRONT_SENSOR_WIDTH 168
#define SIDE_SENSOR_WIDTH 194

#define TURNDEG_MIN 15
#define TURNDEG_SCALE 0.15 //(0.01~0.1) (drops from 1 until 0.6)
#define TURNDEG_INITSCALE 1.3

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
  bool sideOutOfRotationRange = false;
  delay(50);
  if(!hasRotated && front_or_side == 'S')
  {
    motor->command("ROTATE_RIGHT 105 90");
    if(IR_sensors[S_BR]->reading < IR_sensors[S_FR]->reading)
    {
      float turnDegree = atan2(IR_sensors[S_FR]->reading - IR_sensors[S_BR]->reading,SIDE_SENSOR_WIDTH) / PI * 150;
      motor->command("ROTATE_RIGHT 100 "+String(round(turnDegree)));
      //if(DEBUG)Serial.println("Turn additional "+String(turnDegree,2));
    }
    //if(DEBUG)Serial.println("Switch to Front Calibrate");
    sideOutOfRotationRange = true;
    front_or_side = 'F';
  }
  while(!hasRotated && limit>0)
  {
    calibrateDisplacement_forRotation();
    delay(50);
    hasRotated = calibrateRotation(front_or_side);
    delay(50);
    limit--;
  }
  if(limit<=0 && DEBUG)Serial.println("--Give up trying to rotate.--");

  calibrateDisplacement(distInTheory*100,front_or_side);
  if(sideOutOfRotationRange)
  {
    motor->command("ROTATE_LEFT 105 90");
  }
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
    return calibrateRotation_subroutine(CALIBRATE_ROTATION_RANGE_FLO,CALIBRATE_ROTATION_RANGE_FHI,0,50,FL,FR,FRONT_SENSOR_WIDTH,IR_sensors[FM]->reading);
	else if(front_or_side=='S')
    return calibrateRotation_subroutine(CALIBRATE_ROTATION_RANGE_SLO,CALIBRATE_ROTATION_RANGE_SHI,50,100,S_FR,S_BR,SIDE_SENSOR_WIDTH,(IR_sensors[S_FR]->reading+IR_sensors[S_BR]->reading)/2);
}

void Calibration::calibrateDisplacement(int distToObstacle, char front_or_side)
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
  //if(DEBUG)Serial.println("3. ___DISPLACEMENT___");
  if(front_or_side == 'F')
  {
    if(IR_sensors[FM]->reading<=CALIBRATE_DISPLACEMENT_RANGE_FRONT)
    {
      if(distToObstacle == -1)distToObstacle = round(IR_sensors[FM]->reading/100.0)*100;
      displacement_fixNow = distToObstacle - IR_sensors[FM]->reading;
      //if(DEBUG)Serial.print("Difference: " + String(displacement_fixNow));
      
      fixDisplacement(true);
    }
    else
    {
      //if(DEBUG)Serial.println("Can't fix displacement: front sensor is too far from reliable range!");
    }
  }
  else if(front_or_side == 'S')
  {
    if(IR_sensors[S_FR]->reading<=CALIBRATE_DISPLACEMENT_RANGE_SIDE && IR_sensors[S_BR]->reading<=CALIBRATE_DISPLACEMENT_RANGE_SIDE)
    {
      if(distToObstacle == -1)distToObstacle = round(IR_sensors[S_BR]->reading/100.0)*100;
      displacement_fixLater = distToObstacle - (IR_sensors[S_BR]->reading+IR_sensors[S_FR]->reading)/2;
      //if(DEBUG)Serial.println("Difference (Fix LATER): " + String(displacement_fixLater));
      // if unacceptable displacement, align yourself, don't wait for algo
      //HARDCODE
      if(displacement_fixLater < -40)
      {
        motor->command("ROTATE_LEFT 105 90");
        informTurn(false);
        motor->command("ROTATE_RIGHT 105 90");
        //if(DEBUG)Serial.println("Aligned TOWARDS wall without algo intervention.");
      }
      else if(displacement_fixLater > 40)
      {
        motor->command("ROTATE_RIGHT 105 90");
        informTurn(true);
        motor->command("ROTATE_LEFT 105 90");
        //if(DEBUG)Serial.println("Aligned AWAY FROM wall without algo intervention.");
      }
    }
    else
    {
      //if(DEBUG)Serial.println("Can't fix displacement: side sensor is too far from reliable range!");
    }
  }
}
void Calibration::calibrateDisplacement_forRotation()
{
  // only for front sensors!
  updateReadings();
  //if(DEBUG)Serial.println("1. ___DISPLACEMENT (FOR ROTATION)____");
  if(round(IR_sensors[FM]->reading)<=CALIBRATE_DISPLACEMENT_RANGE_FRONT)
  {
    if(IR_sensors[FM]->reading <= CALIBRATE_ROTATION_RANGE_FLO)
    {
      displacement_fixNow = CALIBRATE_ROTATION_RANGE_FLO-IR_sensors[FM]->reading;
      //if(DEBUG)Serial.println("Need to retreat by "+String(displacement_fixNow));
    }
    else if(IR_sensors[FM]->reading >= CALIBRATE_ROTATION_RANGE_FHI)
    {
      displacement_fixNow = CALIBRATE_ROTATION_RANGE_FHI-IR_sensors[FM]->reading;
      //if(DEBUG)Serial.println("Need to advance by "+String(displacement_fixNow));
    }
    fixDisplacement(false);
  }
  else
  {
    //if(DEBUG)Serial.println("Can't advance: front sensor is too far from reliable range!");
  }
}
void Calibration::fixDisplacement(bool useTolerance)
{
  // fix errors recorded by displacement_fixNow
  
  const int tolerance = useTolerance? CALIBRATE_DISPLACEMENT_TOLERANCE:0;
  //if(DEBUG)Serial.println(" Tolerance: "+String(tolerance));
  //HARDCODE
   if(!useTolerance)// SUBJ TO CHANGE
   {
     if(displacement_fixNow > 0)
       displacement_fixNow *= 1.2;
     else
       displacement_fixNow *= 1.2;
   }
  if(displacement_fixNow > tolerance)
  {
    motor->command("BACKWARD 105 " + String(float(abs(displacement_fixNow)/100.0)));
    //if(DEBUG)Serial.println("Distance is fixed by " + String(displacement_fixNow)+".");
  }
  else if(displacement_fixNow < -tolerance)
  {
    motor->command("FORWARD 105 " + String(float(abs(displacement_fixNow)/100.0)));
    //if(DEBUG)Serial.println("Distance is fixed by " + String(displacement_fixNow)+".");
  }
  else
  {
    //if(DEBUG)Serial.println("Distance is within tolerance.");
  }
  displacement_fixNow = 0;
}
bool Calibration::calibrateRotation_subroutine(int guardbound_l, int guardbound_h, int scalebound_l, int scalebound_h, int sensor1, int sensor2, int sensorwidth, int dist)
{
  int tolerance, diff;
  float turnDegree;
  float scaleDownRotation = TURNDEG_INITSCALE;
  // Determine tolerance values
  //if(DEBUG)Serial.println("2. ___CALIBRATING ROTATION___");

  //HARDCODE
  if(dist > guardbound_l && dist < guardbound_h)
  {
    if(dist > scalebound_l && dist <scalebound_h)
      tolerance = CALIBRATE_ROTATION_TOLERANCE + 5*((dist-scalebound_l)*2/100);
    else if(dist >= scalebound_h)
      tolerance = CALIBRATE_ROTATION_TOLERANCE + 5;
    else
      tolerance = CALIBRATE_ROTATION_TOLERANCE;

    diff = abs(IR_sensors[sensor1]->reading - IR_sensors[sensor2]->reading);
    //if(DEBUG)Serial.print("Difference: "+String(diff)+" Tolerance: "+String(tolerance));
    
    while(diff >= tolerance && scaleDownRotation > 0.6)
    {
      turnDegree = atan2(diff,sensorwidth) / PI * 180;
      if(turnDegree<TURNDEG_MIN)
      {
        turnDegree = TURNDEG_MIN;
      }
      turnDegree *= scaleDownRotation;
      //if(DEBUG)Serial.println("\nNeed calibrate. Turn Degree: "+String(turnDegree,2));
      scaleDownRotation -= TURNDEG_SCALE;
      String motorcommand;
      if(IR_sensors[sensor1]->reading < IR_sensors[sensor2]->reading)
      {
        motorcommand = "ROTATE_LEFT 100 "+String(turnDegree,0);
        //if(DEBUG)Serial.println(motorcommand);
        motor->command(motorcommand);
      }
      else
      {
        motorcommand = "ROTATE_RIGHT 100 "+String(turnDegree,0);
        //if(DEBUG)Serial.println(motorcommand);
        motor->command(motorcommand);
      }
      updateReadings();
      diff = abs(IR_sensors[sensor1]->reading - IR_sensors[sensor2]->reading);
    }
    
    //if(DEBUG)Serial.println("\nNo need calibrate, already straight.");
    return true;
  }
  else
  {
    //if(dist > guardbound_h)
      //if(DEBUG)Serial.println("\nFail to calibrate rotation, sensors too FAR from obstacle. Sensor: "+String(sensor1)+" "+String(dist)+" Obstacle: "+String(guardbound_h));
    //else
      //if(DEBUG)Serial.println("\nFail to calibrate rotation, sensors too NEAR from obstacle. Sensor: "+String(dist)+" Obstacle: "+String(guardbound_l));
    return false;
  }
}
void Calibration::updateReadings()
{
  delay(200);
  for(int i=0; i<6; i++)
  {
    IR_sensors[i]->takeReading(true);
    //Serial.print(String(IR_sensors[i]->reading)+"\t");
  }
  //Serial.println();
}

