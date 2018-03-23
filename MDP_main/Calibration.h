#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "IR.h"
#include "Motor.h"
#include <math.h>
class Calibration
{
	private:
		int displacement_fixLater;
    int8_t trustedSensorForDist,sensor1,sensor2;
		IR *IR_sensors[6];
		Motor *motor;
    // main internal functions
    bool calibrateRotation(char front_or_side);
    bool calibrateDisplacement(int distToObstacle, char front_or_side, int displaceOffset);
    // subroutines & utilities
    void fixDisplacement(bool useTolerance);
    void updateReadings(bool wait);
    bool sensorValid(int sensorID, bool near);
    bool DEBUG;

	public:
		Calibration(IR *IR_sensors[6], Motor *motor, bool debug);
    int doCalibrationSet(int distInTheory, char front_or_side, bool s1, bool s2, bool s3);
    void toggleDebug();
		void informTurn(bool right);
};
