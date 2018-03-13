#include "DualVNH5019MotorShield.h"

const int NUM_SAMPLES = 17;
const int TPR = 2249; //Tick Per Rotation
const int TPR_new = 520; //Tick Per Rotation
const int CPC = 298; //Count Per Cell
const float CPD = 4.42511574074; //Count Per Degree
const float RPM_CONVERSION = 120/(TPR*0.000001);
const float CELL_SIZE = 10.0; //cm
const float WHEEL_DIAMETER = 6.0; //cm
const float BASE_DIAMETER = 17.0; //cm
const float Pi = 3.14159;

//PID constant for E1
const float k1_e1 = 0.03;
const float k2_e1 = -0.03;
const float k3_e1 = 0.03;
//PID constant for E2
const float k1_e2 = 0.03;
const float k2_e2 = -0.03;
const float k3_e2 = 0.03;

//Motor Charac
const float E1M = 0.31851837820234463;
const float E1C = -12.16942732305689;
const float E2M = 0.3283140251845417;
const float E2C = -16.30316431466205;

//Encoder offsets
const float e1_offset = 0;
const float e2_offset = 10;

//rotation time offset
const float rotate_r_m = 0.44444444444;
const float rotate_r_c = -20;
const float rotate_l_m = 0.44444444444;
const float rotate_l_c = -20;

//distance time offset
const float dis_time_m = 56.25;
const float dis_time_c = -56.25;

class Motor
{
    private:
        DualVNH5019MotorShield md;
        int E1A;
        int E1B;
        int E2A;
        int E2B;
        float desired_rpm;
        float input_rpm_e1;
        float input_rpm_e2;
        unsigned int e1a_readings[NUM_SAMPLES];
        unsigned int e1b_readings[NUM_SAMPLES];
        unsigned int e2a_readings[NUM_SAMPLES];
        unsigned int e2b_readings[NUM_SAMPLES];
        float error_e1;
        float error_e2;
        float last_error_e1;
        float last_error_e2;
        float last_last_error_e1;
        float last_last_error_e2;
        long commandPeriod;
        
    public:
        bool isRunning;
        String motor_status;
        Motor(int E1A, int E1B, int E2A, int E2B);
        void command(String command);
        int rpmToSpeed(float rpm, boolean isRight);
        void adjustSpeed(bool isForward);
        unsigned int takeMedian(unsigned int nums[]);
        void moveForward(float input_rpm);
        void moveBackward(float input_rpm);
        float getRpm(unsigned int readings[]);
        String getSubString(String data, char separator, int index);
        long getMoveTime(float rpm, float num_cell);
        long getRotateTime(float rpm, float degree, bool isRight);
        long getPeriod(String full_command);
        void resetError();
};

static long tick = 0;
void incrementTick();
