#include "DualVNH5019MotorShield.h"

//Pins
const int E1A = 3; //right
const int E1B = 5;
const int E2A = 11; //left
const int E2B = 13;

const int NUM_SAMPLES = 7;
const int TPR = 2249; //Tick Per Rotation
const float RPM_CONVERSION = 120/(TPR*0.000001);

// Distance params - change these values if distance travelled is inaccurate
const int CPC_F[17] = {265, 600, 900, 1100, 1500, 780};
const int CPC_B[17] = {265, 600, 900, 1100, 1500, 780};
// Orientation params - change these values if the bot is not runnning straight
const float e1_offset = 2; // inc means make right motor slower
const float e2_offset = 0; // inc means make left motor slower
// Rotation params - change these values if the rotation is not 90 degrees
const int CPR = 400; // Count Per Right angle
const int ROTATE_OFFSET_RIGHT = -20; // inc means rotate right more
const int ROTATE_OFFSET_LEFT = -5; // inc means rotate left more

//PID constant for E1
const float k1_e1 = 0.04;
const float k2_e1 = -0.04;
const float k3_e1 = 0.04;
//PID constant for E2
const float k1_e2 = 0.04;
const float k2_e2 = -0.04;
const float k3_e2 = 0.04;

//Motor Charac
const float F_E1M = 0.343694246634824;
const float F_E1C = -12.532550242437424;
const float F_E2M = 0.3591186170212765;
const float F_E2C = -18.768609042553216;

const float B_E1M = -0.3828751411202779;
const float B_E1C = -14.782471016066019;
const float B_E2M = -0.36466075770733813;
const float B_E2C = -15.973117455492877;

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
        int rpmToSpeed(float rpm, boolean isRight, boolean isForward);
        void adjustSpeed(bool isForward);
        unsigned int takeMedian(unsigned int nums[]);
        void moveForward(float input_rpm, int cell_num);
        void moveBackward(float input_rpm, int cell_num);
        void rotateRight(float input_rpm, float degree);
        void rotateLeft(float input_rpm, float degree);
        void stopBot();
        float getRpm(unsigned int readings[]);
        int getRotateTime(float rpm, float degree, bool isRight);
        void resetError();
        void moveForwardCustom(float input_rpm, int desiredTick);
};

volatile static uint8_t tick_MSB = 0;
volatile static uint8_t tick_LSB = 0;
void incrementTick();
