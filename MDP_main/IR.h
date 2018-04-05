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

#define FL_START -20
#define FL_END 250
#define FL_INTERVAL 10
#define FM_START -30
#define FM_END 250
#define FM_INTERVAL 10
#define FR_START -20
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

const int FL_offsets[]PROGMEM = {674,635,589,532,489,452,421,396,378,357,328,312,296,285,268,259,247,240,230,219,209,205,197,196,185,172,172,168};
const int FM_offsets[]PROGMEM = {672,631,564,522,477,473,410,388,362,341,324,335,289,272,266,250,243,236,229,218,210,202,197,189,181,172,173,169,164};
const int FR_offsets[]PROGMEM = {670,651,614,573,542,512,500,457,428,408,383,367,346,335,322,306,297,289,281,274,262,256,247,243,235,226,222,218};
const int S_FR_offsets[]PROGMEM = {586,520,477,438,406,377,351,328,310,290,279,264,254,243,238,222,216,205,198,195,186,177,175,170,163,159};
const int S_BR_offsets[]PROGMEM = {670,652,598,556,492,451,416,385,362,342,327,306,298,283,272,258,247,236,230,220,217,210,203,197,194,186};
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
