#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "IR.h"
#include "Motor.h"
#include <math.h>
const int8_t TOO_CLOSE = -1;
const int8_t CLOSE_AND_FAR = 0;
const int8_t TOO_FAR = 1;
const bool IMMEDIATE = false;
const bool WAIT = true;

class Calibration
{
	private:
		int displacement_fixLater;
    int sensorOffsets[6];
    int8_t trustedSensorForDist,sensor1,sensor2;
		IR *IR_sensors[6];
		Motor *motor;
    // main internal functions
    bool calibrateRotation(char front_or_side);
    bool calibrateDisplacement(char front_or_side);
    // subroutines & utilities
    void fixDisplacement(bool useTolerance);
    void updateReadings(bool wait, bool front, bool side);
    int getReadings(int sensorIndex);
    bool sensorValid(int sensorID, int8_t closeOrFar);
    bool DEBUG;

	public:
		Calibration(IR *IR_sensors[6], Motor *motor, bool debug);
    int doCalibrationSet(char front_or_side, int s1, int s2, int s3);
    void toggleDebug();
		void informTurn(bool right);
};
