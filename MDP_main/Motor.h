#include "DualVNH5019MotorShield.h"
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
const int NUM_SAMPLES = 9;
const int TPR = 2249; //Tick Per Rotation
const uint8_t CPC = 130; //298 //Count Per Cell
const uint8_t CPR = 193; //Count Per Right angle
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
const float e2_offset = 0; // inc means make left motor slower

//rotation time offset
//const float rotate_r_m = 0.44444444444;
//const float rotate_r_c = -20;
//const float rotate_l_m = 0.44444444444;
//const float rotate_l_c = -20;

//distance time offset
//const float dis_time_m = 56.25;
//const float dis_time_c = -56.25;

//command
const char COMM_FORWARD = 'F';
const char COMM_BACKWARD = 'B';
const char COMM_ROTATE_R = 'R';
const char COMM_ROTATE_L = 'L';

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
        char motor_status;
        Motor(int E1A, int E1B, int E2A, int E2B);
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
};

volatile static uint8_t tick = 0;
static uint8_t half_tick = 0;
void incrementTick();
