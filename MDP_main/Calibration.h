
class Calibration
{
	private:
		float displacement_fixNow;
		float displacement_fixLater;
		IR *IR_sensors[6];
		Motor *motor;
    void fixDisplacement(bool useTolerance);

	public:
		Calibration(IR *IR_sensors[6], Motor *motor);
		int calibrateRotation(bool useFront);
		void calibrateDisplacement();
    void calibrateDisplacement_forRotation();
		void informTurn(bool right);
};
