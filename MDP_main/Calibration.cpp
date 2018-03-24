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

#define TURNDEG_SCALEDOWN 0.9 //to prevent coarse rotation from overshooting
#define TURNDISPL_SCALEDOWN 0.9 //to prevent coarse displacement from overshooting

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
}
int Calibration::doCalibrationSet(int distInTheory, char front_or_side, bool s1, bool s2, bool s3)
{
  // sensor choice
  if(front_or_side == 'F')
  {
    if(s1 && s3)
    {
      sensor1 = FL;
      sensor2 = FR;
      if(s2)trustedSensorForDist = FM;
      else trustedSensorForDist = FL;
    }
    else if(s1 && s2)
    {
      sensor1 = FL;
      sensor2 = FM;
      trustedSensorForDist = FL;
    }
    else if(s2 && s3)
    {
      sensor1 = FM;
      sensor2 = FR;
      trustedSensorForDist = FR;
    }
  }
  else if(front_or_side == 'S')
  {
    sensor1 = S_FR;
    sensor2 = S_BR;
    trustedSensorForDist = FL;
  }
  else
  {
    return 0;
  }
  bool hasCalibratedRotation = false;
  bool hasCalibratedDisplacement = false;
  uint8_t limit = 0;
  // guard condition: back up if too close
  if(!sensorValid(FL,true) || !sensorValid(FM,true) || !sensorValid(FR,true))
  {
    motor->moveBackward(20,0);
    do{
      updateReadings(false);
    }while(!sensorValid(FL,true) || !sensorValid(FM,true) || !sensorValid(FR,true));
    delay(250);// continue moving backwards for a while even after sensor goes valid
    motor->stopBot();
    updateReadings(true);
  }
  // guard condition: abort if one of the sensors invalid
  if(front_or_side=='F' && (!sensorValid(sensor1,false) || !sensorValid(sensor2,false)))return -1;
  // main routine: do rotation and displacement calibration until no need anymore (or exceeds limit)
  do
  {
    // check rotation, fine or coarse subroutine?
    hasCalibratedRotation = calibrateRotation(front_or_side);
    
    // check displacement, fine or corase subroutine?
    if(trustedSensorForDist != -1)hasCalibratedDisplacement = calibrateDisplacement(distInTheory*100, front_or_side);
    limit++;
  }while((hasCalibratedRotation || hasCalibratedDisplacement) && limit<3);
	
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
	int sensorwidth,diff,diff2,tolerance;
	float turnDegree, toleranceScale;
	String lr_command;
	if(front_or_side == 'F')
	{
  //if(DEBUG)Serial.println("ROT Front");
    tolerance = ROTATION_FINE_TOLERANCE_FRONT;
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
	diff = IR_sensors[sensor1]->reading - IR_sensors[sensor2]->reading;
	bool isRight = diff > 0;
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
    }while(abs(diff) > tolerance && (isRight == (diff>0)));
    if(DEBUG && (isRight != (diff>0)))Serial.println("Rot overshoot");
    motor->stopBot();
    return true;
	}
	else
		return false;

}

bool Calibration::calibrateDisplacement(int distToObstacle, char front_or_side)
{	
	// update readings, usually executed after rotated back to spot
  delay(200);
	updateReadings(true);
	int diff;
  int tolerance_far = DISPLACEMENT_FINE_TOLERANCE;
  int tolerance_near = -DISPLACEMENT_FINE_TOLERANCE;
	float toleranceScale;
	if(front_or_side == 'F')
	{
    bool isMovingFront;
		diff = -distToObstacle + IR_sensors[trustedSensorForDist]->reading;
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
        updateReadings(false);
        diff = -distToObstacle + IR_sensors[trustedSensorForDist]->reading;
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
		
		displacement_fixLater = -distToObstacle + (IR_sensors[S_FR]->reading + IR_sensors[S_BR]->reading)/2;
		if(displacement_fixLater > tolerance_far || displacement_fixLater < tolerance_near-5) // -5 to allow nearer, prevent turning and realizing no need to back off
		{
			motor->rotateRight(105, 90);
      sensor1 = FL;
      sensor2 = FR;
			calibrateDisplacement(distToObstacle, 'F');
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

void Calibration::updateReadings(bool wait)
{
	if(wait)delay(200);
	for(int i=0; i<6; i++)
	{
	IR_sensors[i]->takeReading(true);
	}
}
bool Calibration::sensorValid(int sensorID, bool near)
{
  if(!near)return !(IR_sensors[sensorID]->reading == 900 || IR_sensors[sensorID]->reading == -900);
  else return !(IR_sensors[sensorID]->reading == -900);
}

