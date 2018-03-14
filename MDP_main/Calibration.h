
class Calibration
{
	private:
		int displacement_fixNow;
		int displacement_fixLater;
		IR *IR_sensors[6];
		Motor *motor;
    // main internal functions
    bool calibrateRotation(char front_or_side);
    bool calibrateDisplacement(int distToObstacle, char front_or_side, int displaceOffset);
    // subroutines & utilities
    void fixDisplacement(bool useTolerance);
    void updateReadings(bool wait);
    bool DEBUG;

	public:
		Calibration(IR *IR_sensors[6], Motor *motor, bool debug);
    int doCalibrationSet(int distInTheory, char front_or_side);
    void toggleDebug();
		void informTurn(bool right);
};
