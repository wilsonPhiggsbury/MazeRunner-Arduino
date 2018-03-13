#define SAMPLES 71
// IR sensor indexes
#define FL 1// front left
#define FM 0// front mid
#define FR 2// front right
#define S_FL 3// left front (side)
#define S_FR 4// right front(side)
#define S_BR 5// right back (side)

#define START -50
#define INTERVAL 50
#define FL_LIMIT 350
#define FM_LIMIT 300
#define FR_LIMIT 350
#define S_FL_LIMIT 800
#define S_FR_LIMIT 300
#define S_BR_LIMIT 300

class IR
{
    private:
        int id;
        // Observation-based values
         const int FL_offsets[8] = {-42,4,84,147,200,229,242,249}; // 8
         const int FM_offsets[9] = {-50,3,57,107,158,213,267,300,350}; // 8
         const int FR_offsets[9] = {-30,3,52,105,159,209,265,300,350}; // 8
         const int S_FL_offsets[18] = {-50,12,71,124,169,207,255,309,367,438,475,540,582,637,666,727,765,830}; // 10
         const int S_FR_offsets[8] = {-42,-2,46,99,141,193,248,283}; // 8
         const int S_BR_offsets[8] = {-25,-1,49,103,151,199,243,284}; // 8
//         int readingConsts_fine[6][26] = {
//            {652, 632, 609, 571, 544, 522, 505, 490, 474, 463, 449, 436, 426, 417, 407, 397, 390, 384, 374, 367, 360, 356, 350, 343,0,0},
//            {597, 569, 544, 521, 498, 471, 454, 436, 421, 409, 393, 381, 373, 358, 348, 337, 325, 317, 310, 301, 293, 288, 283, 275,0,0},
//            {651, 647, 636, 609, 588, 565, 547, 532, 517, 502, 492, 475, 463, 452, 438, 422, 411, 403, 392, 380, 372, 362, 355, 347,0,0},
//            {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
//            {515, 494, 468, 452, 435, 420, 404, 393, 375, 364, 352, 340, 332, 324, 316, 303, 299, 291, 280, 275, 266, 263, 262, 253, 249, 245},
//            {656, 629, 592, 570, 543, 521, 497, 481, 459, 442, 423, 407, 396, 384, 371, 360, 351, 339, 330, 324, 314, 310, 302, 294, 290, 286}
//        };
//         int readingConsts_coarse[6][10] = {
//            {258, 246, 234, 228, 221, 215, 187, 170, 147, 140},
//            {247, 227, 203, 189, 173, 161, 151, 139, 134, 126},
//            {320, 293, 268, 250, 234, 219, 206, 198, 190, 184},
//            {0,0,0,0,0,0,0,0,0,0},
//            {220, 201, 182, 169, 157, 145, 137, 132, 123, 115},
//            {256, 231, 214, 198, 185, 172, 164, 156, 145, 141}            
//        };
        int fitCurve();
        int correction();
        int takeMedian(int nums[]);
        static int scaleByInterval(int variableToScale, int readings[], int start, int end, int interval);
    	static int logisticFit(int x, float A1, float A2, float x0, float p);
        static int cubicFit(int x, float A, float B, float C, float D);

//    		static float power2Fit(float x, float xc, float A, float pl);
//    		static float log3p1Fit(float x, float a, float b, float c);

    public:
    	   int reading;
        IR(int index);
        float takeReading(bool convertToDist);
};
