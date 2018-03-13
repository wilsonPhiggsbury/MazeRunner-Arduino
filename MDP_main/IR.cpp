#include "IR.h"
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

IR::IR(int id)
{
	this->id = id;
}

float IR::takeReading(bool convertToDist)
{
//  disable a reading on purpose
//  if(this->id == 3)
//  {
//    this->reading = 0;
//    return 0;
//  }
	int voltageMedians[SAMPLES];
	for(int j=0; j<SAMPLES; j++)
  {
	  voltageMedians[j] = analogRead(this->id);
  }
  this->reading = takeMedian(voltageMedians);
	if(convertToDist)
	{
		this->reading = fitCurve();
		this->reading = correction();
	}
	return this->reading;
} 

int IR::takeMedian(int nums[SAMPLES])
{
  // insertion sort the nums array
	for(int i=1; i<SAMPLES; i++)
	{
		int tmp;
		// swap values until previous value is not larger than next value
		int j = i;
		while(j!=0 && nums[j] < nums[j-1])
		{
		  tmp = nums[j];
		  nums[j] = nums[j-1];
		  nums[j-1] = tmp;
		  j--;
		}
	}
	// insertion sort done
	// take the middle value
	return nums[(SAMPLES-1)/2];
}
int IR::fitCurve()
{
	int dist,bound;
	int x = this->reading;
	switch(this->id)
	{
	case FL:
   bound = FL_LIMIT;
	 dist = logisticFit(x, 262.81293, -35.58077, 322.95448, 3.54856);
	break;
	case FM:
   bound = FM_LIMIT;
	 dist = logisticFit(x, 2072.83017, -93.91462, 41.48998, 1.22671);
	break;
	case FR:
   bound = FR_LIMIT;
	 dist = logisticFit(x, 4508.49721, -90.55705, 34.42585, 1.35516);
	break;
	case S_FL:
	dist = cubicFit(x, 1799.43651, -9.76627, 0.02037, -1.51172e-5);
	break;
	case S_FR:
   bound = S_FR_LIMIT;
   dist = logisticFit(x, 2698.87551, -121.92001, 29.03353, 1.16081);
	 //dist = cubicFit(x, 815.83317, -5.8258, 0.01505, -1.40109e-5);
	break;
	case S_BR:
   bound = S_BR_LIMIT;
	 dist = logisticFit(x, 10554.28817, -92.81836, 10.60316, 1.22066);
	 //dist = cubicFit(x, 985.90138, -6.45298, 0.01508, -1.23553e-5);
	break;
	}
 return dist;
//   bound += 50;
//	 if(dist>=bound)
//	 	return 900;
//	 else
	 	
//	int n;Serial.println("HERE");
//	if(this->reading >= readingConsts_fine[this->id][0])
//		return 900;//too near
//	for(n=1;n<coarseReadingsBorder;n++){
//
//		if (this->reading >= readingConsts_fine[this->id][n]){
//			return -20+n*5;
//		}
//	}
//	for(n=0;n<samplestaken;n++){
//
//		if (this->reading >= readingConsts_coarse[this->id][n]){
//			return 100+n*25;
//		}
//	}
//	return 900;//too far
}
int IR::correction()
{
  int correctedVal = this->reading;
  if(this->reading == 900)
    return 900;
	
	switch(this->id)
	{
    case FL:
    correctedVal = scaleByInterval(correctedVal,FL_offsets,START,FL_LIMIT,INTERVAL);
    break;
    case FM:
    correctedVal = scaleByInterval(correctedVal,FM_offsets,START,FM_LIMIT,INTERVAL);
    break;
    case FR:
    correctedVal = scaleByInterval(correctedVal,FR_offsets,START,FR_LIMIT,INTERVAL);
    break;
		case S_FL:
    correctedVal = scaleByInterval(correctedVal,S_FL_offsets,START,S_FL_LIMIT,INTERVAL);
		break;
		case S_FR:
    correctedVal = scaleByInterval(correctedVal,S_FR_offsets,START,S_FR_LIMIT,INTERVAL);
		break;
		case S_BR:
    correctedVal = scaleByInterval(correctedVal,S_BR_offsets,START,S_BR_LIMIT,INTERVAL);
		break;
	}
	return correctedVal;
}
static int IR::scaleByInterval(int variableToScale, int readings[], int start, int end, int interval)
{
  int maxIndex = (end-start)/interval + 1; // 8 for -50~300 + 50, 4 for 0~300 +100
  int i;
//  // Actual Versus Ideal
//  // readings[5] stands for : ACTUAL values at range -121, 5, 34, ..., 218
//  // to generate IDEAL numbers, use i*50: ideal ranges -50, 0, 50, ..., 300
//  // get the offset, required to scale by how much?
//  // proceed to determine which range the ACTUAL value falls in [-50,0] or [0,50] or ... or [250,300]
  for(i=0; i<maxIndex-1; i++)
  {
    if(variableToScale >= readings[i] && variableToScale < readings[i+1])
    {
      variableToScale = map(variableToScale, readings[i], readings[i+1], (i-1)*interval, i*interval);
      return variableToScale;
    }
  }
  return 900;
}
static int IR::logisticFit(int x, float A1, float A2, float x0, float p)
{
	return int(A2 + (A1-A2)/(1+pow(float(x)/x0,p)));
}
static int IR::cubicFit(int x, float A, float B, float C, float D)
{
	return int(A + B*x + C*pow(x,2) + D*pow(x,3));
}
// static float IR::power2Fit(float x, float xc, float A, float pl)
// {
// 	return A*pow(abs(x-xc),pl);
// }
// static float IR::log3p1Fit(float x, float a, float b, float c)
// {
// 	return a - b*(log(x+c));
// }



//array frontn is the voltage readings for the front sensors
// i is sensor#, n is voltage stored

	


