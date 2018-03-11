#define SAMPLES 71
// IR sensor indexes
#define FL 0// front left
#define FM 1// front mid
#define FR 2// front right
#define S_FL 3// left front (side)
#define S_FR 4// right front(side)
#define S_BR 5// right back (side)

#define START -50
#define INTERVAL 50
#define FL_LIMIT 300
#define FM_LIMIT 300
#define FR_LIMIT 300
#define S_FL_LIMIT 400
#define S_FR_LIMIT 200
#define S_BR_LIMIT 200

class IR
{
    private:
        int id;
        // Observation-based values
        const int FL_offsets[(FL_LIMIT-START)/INTERVAL+1] = {-50,0,50,100,150,200,250,300}; // 8
        const int FM_offsets[(FM_LIMIT-START)/INTERVAL+1] = {-50,0,50,100,150,200,250,300}; // 8
        const int FR_offsets[(FR_LIMIT-START)/INTERVAL+1] = {-50,0,50,100,150,200,250,300}; // 8
        const int S_FL_offsets[(S_FL_LIMIT-START)/INTERVAL+1] = {-50,0,50,100,150,200,250,300,350,400}; // 10
        const int S_FR_offsets[(S_FR_LIMIT-START)/INTERVAL+1] = {-50,0,50,100,150,200}; // 6
        const int S_BR_offsets[(S_BR_LIMIT-START)/INTERVAL+1] = {-50,0,50,100,150,200}; // 6
        int fitCurve();
        int correction();
        int takeMedian(int nums[]);
        static int scaleByInterval(int variableToScale, int readings[], int start, int interval, int limit);
    		static int logisticFit(int x, float A1, float A2, float x0, float p);
        static int cubicFit(int x, float A, float B, float C, float D);

//    		static float power2Fit(float x, float xc, float A, float pl);
//    		static float log3p1Fit(float x, float a, float b, float c);

    public:
    	   int reading;
        IR(int index);
        float takeReading(bool convertToDist);
};
