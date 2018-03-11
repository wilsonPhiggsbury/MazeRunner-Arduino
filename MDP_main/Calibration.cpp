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

// IR sensor calibrate ROTATION trusted range, in mm
#define ROTATION_RANGE_FLO -15
#define ROTATION_RANGE_FHI 120
#define ROTATION_RANGE_SLO -20
#define ROTATION_RANGE_SHI 110
// tolerance in mm
#define ROTATION_ROUGH_TOLERANCE 25//(5~10)
#define ROTATION_FINE_TOLERANCE 10
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

#define TURNDEG_MIN 15
#define TURNDEG_SCALE 0.15 //(0.01~0.1) (drops from 1 until 0.6)
#define TURNDEG_INITSCALE 1.3

const String DEFAULT_RPM = "105 ";
const String SLOW_RPM = "20";

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
	updateReadings();
	// check whether in rotate range
	/* condition to fufil before updating front displacement  (displacement_fixNow):
	*   front middle sensor indicates distance is between 0 and 1
	* condition to fufil before updating side displacement    (displacement_fixLater):
	*   2 short side sensors agree, distance is between  0 and 1
	*  
	* For front sensors, trust sensor FM
	* For side sensors, trust sensor S_BR
	*/

	// check rotation, fine or coarse subroutine?
	if(DEBUG)Serial.println("Calibrating Rot");
	String fb_command;
	bool sideUseFrontCalibration = false;
	if(front_or_side == 'S')
	{
		// if only one eye visible, come back!! this may fix it (or may not)
		if(IR_sensors[S_FR]->reading == 900 && IR_sensors[S_BR]->reading != 900)
		{
			fb_command = "BACKWARD ";
			motor->command(fb_command + DEFAULT_RPM + "1");
		}
		else if(IR_sensors[S_FR]->reading != 900 && IR_sensors[S_BR]->reading == 900)
		{
			fb_command = "FORWARD ";
			motor->command(fb_command + DEFAULT_RPM + "1");
		}
		else if(IR_sensors[S_FR]->reading == 900 && IR_sensors[S_BR]->reading == 900)
		{
			return 0;
		}
		delay(250);
		// once fixed both eyes visibility (or may not), check if in range.
		int avgSideReading = (IR_sensors[S_FR]->reading + IR_sensors[S_BR]->reading)/2;
		if(avgSideReading < ROTATION_RANGE_SLO || avgSideReading > ROTATION_RANGE_SHI)
		{
			fb_command = "ROTATE_RIGHT ";
			motor->command(fb_command + DEFAULT_RPM + "90");
			sideUseFrontCalibration = true;
			updateReadings();
			// go down and use front calibration
		}
		else
		{
			calibrateRotation('S');
		}
	}
	if(front_or_side == 'F' || sideUseFrontCalibration)
	{
		if(IR_sensors[FM]->reading < ROTATION_RANGE_FLO)
		{
			fb_command = "BACKWARD ";
			motor->command(fb_command + DEFAULT_RPM + String(float(ROTATION_RANGE_FLO-IR_sensors[FM]->reading)/100,2));
		}
		else if(IR_sensors[FM]->reading > ROTATION_RANGE_FHI)
		{
			fb_command = "FORWARD ";
			motor->command(fb_command + DEFAULT_RPM + String(float(IR_sensors[FM]->reading-ROTATION_RANGE_FHI)/100,2));
		}
		calibrateRotation('F');
	}
	
	// check displacement, fine or corase subroutine?
	if(DEBUG)Serial.println("Calibrating Displ");
	calibrateDisplacement(distInTheory*100, sideUseFrontCalibration? 'F':front_or_side);

	delay(250);
	if(sideUseFrontCalibration)
		motor->command("ROTATE_LEFT " + DEFAULT_RPM + "90");
	return 1;
}
void Calibration::informTurn(bool right)
{
	delay(250);
	String fb_command = right? "BACKWARD ":"FORWARD ";
  if(DEBUG)Serial.println("Fix offset: "+String(displacement_fixLater));
	if(displacement_fixLater != 0)
	{
	  motor->command(fb_command + DEFAULT_RPM + String(float(displacement_fixLater/100.0),2));
    displacement_fixLater = 0;
	}
}
void Calibration::toggleDebug()
{
	this->DEBUG = !this->DEBUG;
}
// private ---------------------------------------------------------------
void Calibration::calibrateRotation(char front_or_side)
{
	// update readings, because it may be executed after slight adjustment (to get in range)
	updateReadings();
	int sensorwidth,diff,sensor1,sensor2;
	float turnDegree, toleranceScale;
	String lr_command;
	if(front_or_side == 'F')
	{
		// smart choice of sensors: if blind, use the other two!
		if(IR_sensors[FL]->reading != 900 && IR_sensors[FR]->reading != 900)
		{
			sensor1 = FL;
			sensor2 = FR;
			sensorwidth = FRONT_SENSOR_WIDTH;
      if(DEBUG)Serial.println("L & R");
		}
		else if(IR_sensors[FL]->reading != 900 && IR_sensors[FR]->reading == 900)
		{
			sensor1 = FL;
			sensor2 = FM;
			sensorwidth = 9;
      if(DEBUG)Serial.println("L & M");
		}
		else if(IR_sensors[FL]->reading == 900 && IR_sensors[FR]->reading != 900)
		{
			sensor1 = FM;
			sensor2 = FR;
			sensorwidth = 8;
      if(DEBUG)Serial.println("M & R");
		}
		else
			return;
		toleranceScale = 1;
	}
	else if(front_or_side == 'S')
	{
		sensor1 = S_FR;
		sensor2 = S_BR;
		sensorwidth = SIDE_SENSOR_WIDTH;
		toleranceScale = 1;
	}
	diff = IR_sensors[sensor1]->reading - IR_sensors[sensor2]->reading;
	if(diff > 0)// FL/S_FR longer
		lr_command = "ROTATE_RIGHT ";
	else		// FR/S_BR longer
		lr_command = "ROTATE_LEFT ";
	// _________________________________________ do rough calibration
	if(abs(diff) > ROTATION_ROUGH_TOLERANCE*toleranceScale)
	{
		turnDegree = atan2(abs(diff),sensorwidth) / PI * 162;
		if(DEBUG)Serial.println(lr_command + DEFAULT_RPM + String(turnDegree,0));
		motor->command(lr_command + DEFAULT_RPM + String(turnDegree,0));
	}
	delay(200);
	// _________________________________________ do fine calibration
	// Potentially can switch over to FR and FL again for case 'F'
	updateReadings();
	diff = IR_sensors[sensor1]->reading - IR_sensors[sensor2]->reading;
	if(diff > 0)// FL/S_FR longer
		lr_command = "ROTATE_RIGHT ";
	else    // FR/S_BR longer
		lr_command = "ROTATE_LEFT ";
	if(abs(diff) > ROTATION_FINE_TOLERANCE*toleranceScale)
	{
		if(DEBUG)Serial.println(lr_command + SLOW_RPM);
		motor->command(lr_command + SLOW_RPM);
	}
	else
		return;
	do
	{
		updateReadings();
		diff = IR_sensors[sensor1]->reading - IR_sensors[sensor2]->reading;
		if(DEBUG)Serial.println(diff);
	}while(abs(diff) > ROTATION_FINE_TOLERANCE);
	if(DEBUG)Serial.println("STOP");
	motor->command("STOP");
}

void Calibration::calibrateDisplacement(int distToObstacle, char front_or_side)
{
	
	// update readings, usually executed after rotated back to spot
	updateReadings();
	int diff;
	float toleranceScale;
	String lr_command;
	if(front_or_side == 'F')
	{
		diff = distToObstacle - IR_sensors[FM]->reading;
		toleranceScale = 1;
		if(diff < 0) // too front
			lr_command = "FORWARD ";
		else		// too back
			lr_command = "BACKWARD ";
		// _________________________________________ do rough calibration
		if(abs(diff) > DISPLACEMENT_ROUGH_TOLERANCE)
		{
			if(DEBUG)Serial.println(lr_command + DEFAULT_RPM + String(float(diff)/100.0,2));
			motor->command(lr_command + DEFAULT_RPM + String(float(diff)/100.0,2));
		}
    	delay(250);
		// _________________________________________ do fine calibration
		updateReadings();
		diff = distToObstacle - IR_sensors[FM]->reading;
		if(diff < 0) // too front
			lr_command = "FORWARD ";
		else    // too back
			lr_command = "BACKWARD ";
		if(abs(diff) > DISPLACEMENT_FINE_TOLERANCE)
		{
			if(DEBUG)Serial.println(lr_command + SLOW_RPM);
			motor->command(lr_command + SLOW_RPM);
		}
		else
			return;
		do
		{
			updateReadings();
			diff = distToObstacle - IR_sensors[FM]->reading;
			if(DEBUG)Serial.println(diff);
		}while(abs(diff) > DISPLACEMENT_FINE_TOLERANCE);
		if(DEBUG)Serial.println("STOP");
		motor->command("STOP");
	}
	else if(front_or_side == 'S')
	{
		toleranceScale = 1;
		
		displacement_fixLater = distToObstacle - (IR_sensors[S_FR]->reading + IR_sensors[S_BR]->reading)/2;
		if(abs(displacement_fixLater) > 40)
		{
			motor->command("ROTATE_LEFT 105 90");
			informTurn(false);
			delay(250);
			motor->command("ROTATE_RIGHT 105 90");
		}
	}
}

//void Calibration::fixDisplacement(bool useTolerance)
//{
//	// fix errors recorded by displacement_fixNow
//
//	const int tolerance = useTolerance? CALIBRATE_DISPLACEMENT_TOLERANCE:0;
//	//if(DEBUG)if(DEBUG)Serial.println(" Tolerance: "+String(tolerance));
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
//		if(DEBUG)Serial.println("BACKWARD 105 " + String(float(abs(displacement_fixNow)/100.0)));
//		//if(DEBUG)if(DEBUG)Serial.println("Distance is fixed by " + String(displacement_fixNow)+".");
//	}
//	else if(displacement_fixNow < -tolerance)
//	{
//		if(DEBUG)Serial.println("FORWARD 105 " + String(float(abs(displacement_fixNow)/100.0)));
//		//if(DEBUG)if(DEBUG)Serial.println("Distance is fixed by " + String(displacement_fixNow)+".");
//	}
//	else
//	{
//		//if(DEBUG)if(DEBUG)Serial.println("Distance is within tolerance.");
//	}
//	displacement_fixNow = 0;
//}
//bool Calibration::calibrateRotation_subroutine(int guardbound_l, int guardbound_h, int scalebound_l, int scalebound_h, int sensor1, int sensor2, int sensorwidth, int dist)
//{
//	int tolerance, diff;
//	float turnDegree;
//	float scaleDownRotation = TURNDEG_INITSCALE;
//	// Determine tolerance values
//	//if(DEBUG)if(DEBUG)Serial.println("2. ___CALIBRATING ROTATION___");
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
void Calibration::updateReadings()
{
	delay(200);
	for(int i=0; i<6; i++)
	{
	IR_sensors[i]->takeReading(true);
	//Serial.print(String(IR_sensors[i]->reading)+"\t");
	}
	//if(DEBUG)Serial.println();
}

