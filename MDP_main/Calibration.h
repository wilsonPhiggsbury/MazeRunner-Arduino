
class Calibration
{
	private:
		int displacement_fixNow;
		int displacement_fixLater;
		IR *IR_sensors[6];
		Motor *motor;
    // main internal functions
    void calibrateRotation(char front_or_side);
    void calibrateDisplacement(int distToObstacle, char front_or_side);
    void calibrateDisplacement_forRotation();
    // subroutines & utilities
    void fixDisplacement(bool useTolerance);
    bool calibrateRotation_subroutine(int guardbound_l, int guardbound_h, int scalebound_l, int scalebound_h, int sensor1, int sensor2, int sensorwidth, int dist);
    void updateReadings(bool wait);
    bool DEBUG;

	public:
		Calibration(IR *IR_sensors[6], Motor *motor, bool debug);
    int doCalibrationSet(int distInTheory, char front_or_side);
    void toggleDebug();
		void informTurn(bool right);
};
