#include "DualVNH5019MotorShield.h"

//Pins
const int E1A = 3; //right
const int E1B = 5;
const int E2A = 11; //left
const int E2B = 13;

const int NUM_SAMPLES = 9;
const int TPR = 2249; //Tick Per Rotation
const uint8_t CPC = 130; //Count Per Cell
const uint8_t CPR = 193; //Count Per Right angle
const float RPM_CONVERSION = 120/(TPR*0.000001);

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
const float e2_offset = 3; // inc means make left motor slower

//command
const char COMM_FORWARD = 'F';
const char COMM_BACKWARD = 'B';
const char COMM_ROTATE_R = 'R';
const char COMM_ROTATE_L = 'L';

class Motor
{
    private:
        DualVNH5019MotorShield md;
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
        
    public:
        bool isRunning;
        char motor_status;
        Motor();
        int rpmToSpeed(float rpm, boolean isRight);
        void adjustSpeed(bool isForward);
        unsigned int takeMedian(unsigned int nums[]);
        void moveForward(float input_rpm, float cell_num);
        void moveBackward(float input_rpm, float cell_num);
        void rotateRight(float input_rpm, float degree);
        void rotateLeft(float input_rpm, float degree);
        void stopBot();
        float getRpm(unsigned int readings[]);
        uint8_t getRotateTime(float rpm, float degree, bool isRight);
        void resetError();
        unsigned long getCorrection(int num_cells);
};

volatile static uint8_t tick = 0;
static uint8_t half_tick = 0;
void incrementTick();
