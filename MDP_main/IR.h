#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define SAMPLES 61
// IR sensor indexes
#define FL 1// front left
#define FM 0// front mid
#define FR 2// front right
#define S_FL 3// left front (side)
#define S_FR 4// right front(side)
#define S_BR 5// right back (side)

// TODO: obtain more samples of long range readings

#define FL_START -10
#define FL_END 310
#define FL_INTERVAL 5
#define FM_START -10
#define FM_END 310
#define FM_INTERVAL 5
#define FR_START -10
#define FR_END 310
#define FR_INTERVAL 5
#define SIDE_START -40
#define SIDE_END 270
#define SIDE_INTERVAL 5
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
const int FL_offsets[65]PROGMEM = {633,597,567,552,518,498,480,465,458,445,415,398,383,375,366,358,346,337,327,318,308,301,294,289,283,279,275,267,262,255,251,242,239,235,227,221,215,213,210,205,204,200,194,190,186,184,182,178,174,174,170,167,165,163,160,158,153,153,148,148,144,144,140,140,140};
const int FM_offsets[65]PROGMEM = {579,550,530,513,489,465,450,434,418,403,391,381,368,356,343,336,327,315,307,300,290,287,281,276,268,265,260,253,248,240,235,231,227,224,220,210,207,205,202,198,196,195,193,191,185,181,178,177,175,173,171,169,167,165,163,161,157,155,152,151,150,149,148,148,148};
const int FR_offsets[65]PROGMEM = {652,636,622,603,583,564,550,535,520,508,493,477,462,450,435,421,410,402,390,378,373,366,353,344,340,333,325,316,313,304,301,295,293,285,280,275,270,265,257,255,251,248,246,241,238,237,234,230,227,225,222,220,218,214,210,206,204,202,200,198,197,197,197,197,197};
const int S_FR_offsets[63]PROGMEM = {517,498,472,456,437,421,405,391,375,362,347,339,331,318,315,302,294,286,278,274,266,262,257,254,245,242,238,233,230,222,218,216,214,210,205,200,197,194,187,185,183,180,178,176,170,168,167,165,164,162,160,156,159,156,154,151,149,145,143,142,141,140,135};
const int S_BR_offsets[63]PROGMEM = {659,640,601,580,549,524,497,478,465,436,417,405,398,382,370,359,349,338,330,319,315,310,301,295,290,282,274,270,265,262,254,248,244,240,236,231,229,224,220,217,212,207,205,203,202,200,197,196,193,191,188,186,185,183,180,178,177,175,173,171,168,165,162};
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
