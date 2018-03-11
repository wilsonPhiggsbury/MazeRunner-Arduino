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
	float voltageMedians[SAMPLES];
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

float IR::takeMedian(float nums[SAMPLES])
{
  // insertion sort the nums array
	for(int i=1; i<SAMPLES; i++)
	{
		float tmp;
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
float IR::fitCurve()
{
	float dist,bound;
	float x = this->reading;
	switch(this->id)
	{
	case 0: // L front
	bound = 350;
	dist = logisticFit(x, 3980.64967, -60.66538, 24.73956, 1.35595);
	break;
	case 1: // M front
	bound = 250;
	dist = logisticFit(x, 332.47702, -30.99956, 210.32544, 2.92285);
	break;
	case 2: // R front
	bound = 350;
	dist = logisticFit(x, 839.38251, -111.55843, 133.99694, 1.39961);
	break;
	case 3: // R back (side, short)
	bound = 350;
	dist = cubicFit(x, 1965.88511, -10.15601, 0.01991, -1.40721e-5);
	break;
	case 4: // R front (side, short)
	bound = 250;
	dist = cubicFit(x, 815.83317, -5.8258, 0.01505, -1.40109e-5);
	// dist = logisticFit(x, 67.28591, -1.10478, 0.06223, 1.17884);
	break;
	case 5:
	bound = 250; // L front (side, long)
	dist = cubicFit(x, 985.90138, -6.45298, 0.01508, -1.23553e-5);
	break;
	}

	if(dist>=bound)
		return 900;
	else
		return dist;
}
float IR::correction()
{
  if(this->reading == 900)
    return 900;
	int correctedVal = this->reading;
	switch(this->id)
	{
		case 0:// offset
			if(correctedVal<100)
				correctedVal += 10*(this->reading/100);
			else
				correctedVal += 10;
		break;
		case 1: // scale up only in range [0,1]
			correctedVal += (50*this->reading/300);
		break;
		case 2: // scale down
//			if(correctedVal>200)
//				correctedVal += 50*(this->reading-200);
      correctedVal += 3;
		break;
		case 3:
       
		break;
		case 4:
      correctedVal -= 3;
		break;
		case 5:
		  correctedVal -= 4;
		break;
	}
	return correctedVal;
}
static float IR::logisticFit(float x, float A1, float A2, float x0, float p)
{
	return (A2 + (A1-A2)/(1+pow(x/x0,p)));
}
static float IR::cubicFit(float x, float A, float B, float C, float D)
{
	return (A + B*x + C*pow(x,2) + D*pow(x,3));
}
// static float IR::power2Fit(float x, float xc, float A, float pl)
// {
// 	return A*pow(abs(x-xc),pl);
// }
// static float IR::log3p1Fit(float x, float a, float b, float c)
// {
// 	return a - b*(log(x+c));
// }
