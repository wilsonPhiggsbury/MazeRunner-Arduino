#define SAMPLES 71

class IR
{
    private:
        int id;
        
        int fitCurve();
        int correction();
        int takeMedian(int nums[]);
        static int scaleByInterval(int variableToScale, int readings[], int start, int interval);
    		static int logisticFit(int x, float A1, float A2, float x0, float p);
        static int cubicFit(int x, float A, float B, float C, float D);

//    		static float power2Fit(float x, float xc, float A, float pl);
//    		static float log3p1Fit(float x, float a, float b, float c);

    public:
    	   int reading;
        IR(int index);
        float takeReading(bool convertToDist);
};
