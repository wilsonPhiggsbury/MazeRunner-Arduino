#define SAMPLES 71

class IR
{
    private:
        int id;
        
        float fitCurve();
        float correction();
        float takeMedian(float nums[]);
    		static float logisticFit(float x, float A1, float A2, float x0, float p);
        static float IR::cubicFit(float x, float A, float B, float C, float D);

//    		static float power2Fit(float x, float xc, float A, float pl);
//    		static float log3p1Fit(float x, float a, float b, float c);

    public:
    	float reading;
        IR(int index);
        float takeReading(bool convertToDist);
};
