#include "Calibration.h"

// IR sensor indexes
#define FM 0// front left
#define FL 1// front mid
#define FR 2// front right
#define S_FL 3// left front (side)
#define S_FR 4// right front(side)
#define S_BR 5// right back (side)

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

#define TURNDEG_SCALEDOWN 0.9 //to prevent coarse rotation from overshooting
#define TURNDISPL_SCALEDOWN 0.9 //to prevent coarse displacement from overshooting

const short DEFAULT_RPM = 105;
const short SLOW_RPM = 20;

// public ---------------------------------------------------------------
Calibration::Calibration(IR *IR_sensors[6], Motor *motor, bool debug)
{
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
	bool calibratedRotation = false;
	bool calibratedDisplacement = false;
  uint8_t limit = 0;
  // guard condition: back up if too close
  if(IR_sensors[FM]->reading == -900)
  {
    motor->moveBackward(105,1);
    updateReadings(true);
  }
	// check whether in rotate range
	/* condition to fufil before updating front displacement  (displacement_fixNow):
	*   front middle sensor indicates distance is between 0 and 1
	* condition to fufil before updating side displacement    (displacement_fixLater):
	*   2 short side sensors agree, distance is between  0 and 1
	*  
	* For front sensors, trust sensor FM
	* For side sensors, trust sensor S_BR
	*/
  do
  {
    // check rotation, fine or coarse subroutine?
    calibratedRotation = calibrateRotation(front_or_side);
    
    // check displacement, fine or corase subroutine?
    calibratedDisplacement = calibrateDisplacement(distInTheory*100, front_or_side, 0);//front_or_side=='S'?0:0);
    limit++;
  }while((calibratedRotation || calibratedDisplacement) && limit<2);
	
	return 1;
}
void Calibration::informTurn(bool right)
{
	//delay(250);
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
	updateReadings(true);
	int sensorwidth,diff,diff2,tolerance,sensor1,sensor2;
	float turnDegree, toleranceScale;
	String lr_command;
	if(front_or_side == 'F')
	{
  //if(DEBUG)Serial.println("ROT Front");
    tolerance = ROTATION_FINE_TOLERANCE_FRONT;
		// smart choice of sensors: if blind, use the other two!
		if(sensorValid(FL) && sensorValid(FR))
		{
			sensor1 = FL;
			sensor2 = FR;
      trustedSensorForDist = FM;
			sensorwidth = FRONT_SENSOR_WIDTH;
      //if(DEBUG)Serial.println("L & R");
		}
		else if(sensorValid(FL) && !sensorValid(FR) && sensorValid(FM))
		{
			// Only right sensor blind
			sensor1 = FL;
			sensor2 = FM;
      trustedSensorForDist = FL;
			sensorwidth = 9;
      //if(DEBUG)Serial.println("L & M");
		}
		else if(!sensorValid(FL) && sensorValid(FR) && sensorValid(FM))
		{
			// Only left sensor blind
			sensor1 = FM;
			sensor2 = FR;
      trustedSensorForDist = FR;
			sensorwidth = 8;
      //if(DEBUG)Serial.println("M & R");
		}
		else
		{
			//Sensors blind, abort FRONT
      trustedSensorForDist = -1;
			return false;
		}
		toleranceScale = 1;
	}
	else if(front_or_side == 'S')
	{
  //if(DEBUG)Serial.println("Rot Side");
    tolerance = ROTATION_FINE_TOLERANCE_SIDE;
		sensor1 = S_FR;
		sensor2 = S_BR;
		sensorwidth = SIDE_SENSOR_WIDTH;
		toleranceScale = 1;
	}
	// diff = IR_sensors[sensor1]->reading - IR_sensors[sensor2]->reading;
	// _________________________________________ do rough calibration
	// if(abs(diff) > ROTATION_ROUGH_TOLERANCE*toleranceScale)
	// {
	// 	turnDegree = atan2(abs(diff),sensorwidth) / PI * 180 * TURNDEG_SCALEDOWN;
	// 	//if(DEBUG)Serial.println(String(turnDegree)+lr_command + DEFAULT_RPM + String(turnDegree,0));
	// 	if(diff > 0){
	// 		motor->rotateRight(DEFAULT_RPM, turnDegree);
	// 	}
	// 	else {
	// 		motor->rotateLeft(DEFAULT_RPM, turnDegree);
	// 	}
	// }
	// delay(200);
	// _________________________________________ do fine calibration
	// Potentially can switch over to FR and FL again for case 'F'
	diff = IR_sensors[sensor1]->reading - IR_sensors[sensor2]->reading;
	bool isRight = diff > 0;
//  uint8_t counter = 0;
	if(abs(diff) > tolerance*toleranceScale)
	{
		//if(DEBUG)Serial.println(lr_command + SLOW_RPM);
		if(diff > 0) {
      motor->rotateLeft(SLOW_RPM, 0);
      do{
        updateReadings(false);
        diff2 = IR_sensors[sensor1]->reading - IR_sensors[sensor2]->reading;
      }while(diff2 <= diff);
      motor->stopBot();
			motor->rotateRight(SLOW_RPM, 0);
		}
		else {
      motor->rotateRight(SLOW_RPM, 0);
      do{
        updateReadings(false);
        diff2 = IR_sensors[sensor1]->reading - IR_sensors[sensor2]->reading;
      }while(diff2 >= diff);
      motor->stopBot();
			motor->rotateLeft(SLOW_RPM, 0);
		}
    do
    {
      updateReadings(false);
      diff = IR_sensors[sensor1]->reading - IR_sensors[sensor2]->reading;
      //if(DEBUG)Serial.println(diff);
    }while(abs(diff) > tolerance && (isRight == (diff>0)));
    //if(DEBUG)Serial.println("STOP");
    motor->stopBot();
    return true;
	}
	else
		return false;

}

bool Calibration::calibrateDisplacement(int distToObstacle, char front_or_side, int displaceOffset)
{	
	// update readings, usually executed after rotated back to spot
	updateReadings(true);
	int diff;
  int tolerance_far = DISPLACEMENT_FINE_TOLERANCE-displaceOffset;
  int tolerance_near = -DISPLACEMENT_FINE_TOLERANCE;
	float toleranceScale;
	if(front_or_side == 'F')
	{//if(DEBUG)Serial.println("DISPL Front");
    bool isMovingFront;
		diff = -distToObstacle + IR_sensors[trustedSensorForDist]->reading;
		toleranceScale = 1;
		
		// _________________________________________ do fine calibration
    isMovingFront = diff>0;
		if(diff > tolerance_far || diff < tolerance_near)
		{
//			if(DEBUG)Serial.println("CAL Front slowly");
			if(diff > 0) {
				motor->moveForward(SLOW_RPM, 0);
			}
			else {
				motor->moveBackward(SLOW_RPM, 0);
			}
      do
      {
        updateReadings(false);
        diff = -distToObstacle + IR_sensors[trustedSensorForDist]->reading;
        motor->adjustSpeed(diff>0);//Serial.println("Diff > far " + String(diff) + " > " + String(tolerance_far));
      }while((diff > tolerance_far+15 || diff < tolerance_near) && (isMovingFront == (diff>0)));
      //if(DEBUG)Serial.println(String(isMovingFront==diff<0));
      motor->stopBot();
      return true;
  	}
		else
			return false;
		
	}
	else if(front_or_side == 'S')
	{//if(DEBUG)Serial.println("DISPL Side");
		toleranceScale = 1;
		
		displacement_fixLater = -distToObstacle + (IR_sensors[S_FR]->reading + IR_sensors[S_BR]->reading)/2;
		// if(abs(displacement_fixLater) > 40)
		// {
		// 	motor->rotateLeft(105, 90);
		// 	informTurn(false);
		// 	delay(250);
		// 	motor->rotateRight(105, 90);
		// }
		if(displacement_fixLater > tolerance_far || displacement_fixLater < tolerance_near)
		{
			motor->rotateRight(105, 90);
			calibrateDisplacement(distToObstacle, 'F', displaceOffset);
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

//void Calibration::fixDisplacement(bool useTolerance)
//{
//	// fix errors recorded by displacement_fixNow
//
//	const int tolerance = useTolerance? CALIBRATE_DISPLACEMENT_TOLERANCE:0;
//	////if(DEBUG)//if(DEBUG)Serial.println(" Tolerance: "+String(tolerance));
//	//HARDCODE
//	if(!useTolerance)// SUBJ TO CHANGE
//	{
//		if(displacement_fixNow > 0)
//			displacement_fixNow *= 1.2;
//		else
//			displacement_fixNow *= 1.2;
//	}
//	if(displacement_fixNow > tolerance)
//	{
//		//if(DEBUG)Serial.println("BACKWARD 105 " + String(float(abs(displacement_fixNow)/100.0)));
//		////if(DEBUG)//if(DEBUG)Serial.println("Distance is fixed by " + String(displacement_fixNow)+".");
//	}
//	else if(displacement_fixNow < -tolerance)
//	{
//		//if(DEBUG)Serial.println("FORWARD 105 " + String(float(abs(displacement_fixNow)/100.0)));
//		////if(DEBUG)//if(DEBUG)Serial.println("Distance is fixed by " + String(displacement_fixNow)+".");
//	}
//	else
//	{
//		////if(DEBUG)//if(DEBUG)Serial.println("Distance is within tolerance.");
//	}
//	displacement_fixNow = 0;
//}
//bool Calibration::calibrateRotation_subroutine(int guardbound_l, int guardbound_h, int scalebound_l, int scalebound_h, int sensor1, int sensor2, int sensorwidth, int dist)
//{
//	int tolerance, diff;
//	float turnDegree;
//	float scaleDownRotation = TURNDEG_INITSCALE;
//	// Determine tolerance values
//	////if(DEBUG)//if(DEBUG)Serial.println("2. ___CALIBRATING ROTATION___");
//
//	//HARDCODE
//	if(dist > guardbound_l && dist < guardbound_h)
//	{
//	if(dist > scalebound_l && dist <scalebound_h)
//	  tolerance = CALIBRATE_ROTATION_TOLERANCE + 5*((dist-scalebound_l)*2/100);
//	else if(dist >= scalebound_h)
//	  tolerance = CALIBRATE_ROTATION_TOLERANCE + 5;
//	else
//	  tolerance = CALIBRATE_ROTATION_TOLERANCE;
//
//	}
//}
void Calibration::updateReadings(bool wait)
{
	if(wait)delay(200);
	for(int i=0; i<6; i++)
	{
	IR_sensors[i]->takeReading(true);
	}
	////if(DEBUG)Serial.println();
}
bool Calibration::sensorValid(int sensorID)
{
  return !(IR_sensors[sensorID]->reading == 900 || IR_sensors[sensorID]->reading == -900);
}

