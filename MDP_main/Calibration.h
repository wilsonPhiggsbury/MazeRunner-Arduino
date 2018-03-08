
class Calibration
{
	private:
		float displacement_fixNow;
		float displacement_fixLater;
		IR *IR_sensors[6];
		Motor *motor;
    // main internal functions
    bool calibrateRotation(char front_or_side);
    void calibrateDisplacement(int distToObstacle);
    void calibrateDisplacement_forRotation();
    // subroutines & utilities
    void fixDisplacement(bool useTolerance);
    bool calibrateRotation_subroutine(float guardbound_l, float guardbound_h, float scalebound_l, float scalebound_h, int sensor1, int sensor2, float sensorwidth, float dist);
    void updateReadings();
    bool DEBUG;

	public:
		Calibration(IR *IR_sensors[6], Motor *motor, bool debug);
    int doCalibrationSet(int distInTheory, char front_or_side);
    void toggleDebug();

		void informTurn(bool right);
};
