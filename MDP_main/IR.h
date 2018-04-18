#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define SAMPLES 101
// IR sensor indexes
#define FL 1// front left
#define FM 0// front mid
#define FR 2// front right
#define S_FL 3// left front (side)
#define S_FR 4// right front(side)
#define S_BR 5// right back (side)

// TODO: obtain more samples of long range readings

#define FL_START -30
#define FL_END 250
#define FL_INTERVAL 10
#define FM_START -30
#define FM_END 250
#define FM_INTERVAL 10
#define FR_START -30
#define FR_END 250
#define FR_INTERVAL 10
#define SIDE_START -40
#define SIDE_END 210
#define SIDE_INTERVAL 10
#define LONG_START -25
#define LONG_END 450
#define LONG_INTERVAL 25

//
//const int FL_offsets[8]PROGMEM = {-45,0,84,147,200,229,242,249}; // 8
//const int FM_offsets[9]PROGMEM = {-45,10,57,107,158,213,267,300,350}; // 8
//const int FR_offsets[9]PROGMEM = {-45,15,52,105,159,209,265,300,350}; // 8
//const int S_FL_offsets[17]PROGMEM = {12,71,124,169,207,255,309,367,438,475,540,582,637,666,727,765,830}; // 10
//const int S_FR_offsets[8]PROGMEM = {-49,8,56,99,141,193,248,283}; // 8
//const int S_BR_offsets[8]PROGMEM = {-32,9,59,103,151,199,243,284}; // 8
//const int FL_offsets_fine[21]PROGMEM = {-50,-45,-40,-35,-30,-25,-20,-15,-10,-5,0,5,10,15,20,25,30,35,40,45,50};
//const int FM_offsets_fine[21]PROGMEM = {-50,-45,-40,-35,-30,-25,-20,-15,-10,-5,0,5,10,15,20,25,30,35,40,45,50};
//const int FR_offsets_fine[21]PROGMEM = {-50,-45,-40,-35,-30,-25,-20,-15,-10,-5,0,5,10,15,20,25,30,35,40,45,50};
//const int S_FR_offsets_fine[21]PROGMEM = {-50,-45,-40,-35,-30,-25,-20,-15,-10,-5,0,5,10,15,20,25,30,35,40,45,50};
//const int S_BR_offsets_fine[21]PROGMEM = {-50,-45,-40,-35,-30,-25,-20,-15,-10,-5,0,5,10,15,20,25,30,35,40,45,50};
// const int FL_offsets[8]PROGMEM = {50,100,150,200,250,300};
// const int FM_offsets[9]PROGMEM = {50,100,150,200,250,300,350};
// const int FR_offsets[9]PROGMEM = {50,100,150,200,250,300,350};
// const int S_FL_offsets[18]PROGMEM = {-50,0,50,100,150,200,250,300,350,450,500,550,600,650,700,750,800,850};
// const int S_FR_offsets[8]PROGMEM = {50,100,150,200,250,300};
// const int S_BR_offsets[8]PROGMEM = {50,100,150,200,250,300};
//                                 -10 -5  0   5   10  15  20  25  30  35  40  45  50  55  60  65  70  75  80  85  90  95  100 105 110 115 120 125 130 135 140 145 150 155 160 165 170 175 180 185 190 195 200 205 210 215 220 225 230 235 240 245 250 255 260 265 270 275 280 285 290 295 300 305 310 

const int FL_offsets[]PROGMEM = {645,574,529,487,446,415,385,362,343,328,316,291,283,267,254,246,230,223,213,205,197,189,180,177,173,165,160,163,151};
const int FM_offsets[]PROGMEM = {651,606,566,533,458,425,394,371,348,332,315,294,281,267,255,244,235,222,215,206,201,189,185,181,174,164,160,158,181};
const int FR_offsets[]PROGMEM = {636,601,564,538,501,471,448,425,400,382,369,345,329,323,302,291,278,272,263,256,251,242,236,226,222,214,206,206,202};
const int S_FR_offsets[]PROGMEM = {486,445,416,393,359,336,322,301,287,270,259,253,239,226,214,210,200,193,190,183,174,167,161,156,152,146};
const int S_BR_offsets[]PROGMEM = {654,621,565,507,472,456,403,377,355,334,335,310,290,275,267,254,241,233,227,219,214,206,197,194,191,181};
const int S_FL_offsets[20]PROGMEM = {584,553,514,496,464,431,408,376,352,330,310,300,280,261,253,239,230,218,210,200};
//checklist const int S_FL_offsets[11]PROGMEM = {288,257,236,213,197,185,174,159,154,141,138};
//S_FL: for LR2{610,596,577,550,524,494,459,428,403,374,349,333,314,294,281,269,256,244};
static int cubicFit(int x, float A, float B, float C, float D);
class IR
{
    private:
        int id;
        int fitCurve();
        int lookUptable();
        int scaleByInterval(int variableToScale, const int readings[], int start, int end, int interval);
//		static int logisticFit(int x, float A1, float A2, float x0, float p);
 	
//    	static float power2Fit(float x, float xc, float A, float pl);
//    	static float log3p1Fit(float x, float a, float b, float c);

    public:
    	int reading;
        IR(int index);
        float takeReading(bool convertToDist);
};
