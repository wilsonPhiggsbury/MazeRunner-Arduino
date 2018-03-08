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
	  voltageMedians[j] = analogRead(this->id)*5.0/1024.0;
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
	bound = 3.5;
	dist = logisticFit(x, 229.46929, -0.70569, 0.02349, 1.22789);
	break;
	case 1: // M front
	bound = 3.5;
	dist = logisticFit(x, 3.32477, -0.31000, 1.02698, 2.92285);
	break;
	case 2: // R front
	bound = 3.5;
	dist = logisticFit(x, 8.39383, -1.11558, 0.65428, 1.39961);
	break;
	case 3: // R back (side, short)
	bound = 3.5;
	dist = cubicFit(x, 19.65885, -20.7995, 8.34963, -1.20879);
	break;
	case 4: // R front (side, short)
	bound = 2.5;
	dist = logisticFit(x, 14.56058, -0.97428, 0.30062, 1.45322);
	// dist = logisticFit(x, 67.28591, -1.10478, 0.06223, 1.17884);
	break;
	case 5:
	bound = 2.5; // L front (side, long)
	dist = logisticFit(x, 154.43877, -0.66568, 0.07746, 1.59525);
	break;
	}

	if(dist>=bound)
		return 9;
	else
		return dist;
}
float IR::correction()
{
  if(this->reading == 9)
    return 9;
	float correctedVal = this->reading;
	switch(this->id)
	{
		case 0:// offset
			if(correctedVal<1)
				correctedVal += 0.1*this->reading;
			else
				correctedVal += 0.1;
		break;
		case 1: // scale up only in range [0,1]
			correctedVal += (0.5*this->reading/3);
		break;
		case 2: // scale down
//			if(correctedVal>2)
//				correctedVal += 0.5*(this->reading-2);
      correctedVal += 0.03;
		break;
		case 3:
       
		break;
		case 4:
      correctedVal -= 0.03;
		break;
		case 5:
		  correctedVal -= 0.04;
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
