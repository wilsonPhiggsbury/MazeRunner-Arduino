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
	float voltageMedians[6][SAMPLES];
	for(int i=0; i<6; i++)
	{
	  for(int j=0; j<SAMPLES; j++)
	  {
		  voltageMedians[i][j] = analogRead(this->id)*5.0/1024.0;
		  delayMicroseconds(50);
	  }
	  delay(5);
	  this->reading = takeMedian(voltageMedians[i]);
	  if(convertToDist)
	  {
	  	this->reading = fitCurve();
	  	correction();
	  }
	}
	return this->reading;
} 

float IR::takeMedian(float nums[])
{
	int size = sizeof(nums)/sizeof(nums[0]);
		// insertion sort the nums array
	for(int i=1; i<size; i++)
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
	switch(this->id)
	{
	case 0: // L front
	bound = 3.5;
	dist = logisticFit(this->reading, 147.2596, -0.88058, 0.03274, 1.17774);
	break;
	case 1: // M front
	bound = 2.4;
	dist = power2Fit(this->reading, 2.60905, 0.51431, 2.47468);
	break;
	case 2: // R front
	bound = 3.5;
	dist = logisticFit(this->reading, 163.14261, -0.94237, 0.04718, 1.25732);
	break;
	case 3: // L front (side, long)
	bound = 4;
	dist = log3p1Fit(this->reading, 3.5882, 3.66031, -0.33237);
	/* below is poly3fit, an alternative
	 *  dist = 17.0795 - 18.16677*x + 7.56588*pow(x,2) - 1.15652*pow(X,3);
	 */
	break;
	case 4: // R front (side, short)
	bound = 3.5;
	dist = logisticFit(this->reading, 135.99865, -1.23964, 0.02665, 1.10135);
	break;
	case 5:
	bound = 2.5; // R back (side, short)
	dist = logisticFit(this->reading, 19.4419, -0.62827, 0.31499, 1.72727);
	/* below is real plot, above uses "quick plot" to define range 
	 *  bound = 3.5;
	 *  dist = logisticFit(value, 905.58779, -0.07398, 0.11493, 2.82832);
	 */
	break;
	}

	if(dist>bound)
		return 90;
	else
		return dist;
}
float IR::correction()
{
	switch(this->id)
	{
		case 0:// scale down
		// needs scaling at extreme ranges (2~3), gets blind outside 3.5
		if(this->reading>2)
		{
			this->reading *= 1-(this->reading-2)/15;
		}
		break;
		case 1: // scale up
		if(this->reading>2)
		{
			this->reading *= 1+(this->reading-2)*1.153; 
		}
		case 2: // scale down
		if(this->reading>2)
		{
			this->reading *= 1-(this->reading-2)/20;
		}
		break;
		case 3:

		break;
		case 4:

		break;
		case 5:

		break;
	}
}
static float IR::logisticFit(float x, float A1, float A2, float x0, float p)
{
	return (A2 + (A1-A2)/(1+pow(x/x0,p)));
}
static float IR::power2Fit(float x, float xc, float A, float pl)
{
	return A*pow(abs(x-xc),pl);
}
static float IR::log3p1Fit(float x, float a, float b, float c)
{
	return a - b*(log(x+c));
}
