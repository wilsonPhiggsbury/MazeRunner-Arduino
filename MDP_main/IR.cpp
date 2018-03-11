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
	dist = logisticFit(x, 3980.64967, -60.66538, 24.73956, 1.35595);
	break;
	case FM:
  bound = FM_LIMIT;
	dist = logisticFit(x, 332.47702, -30.99956, 210.32544, 2.92285);
	break;
	case FR:
  bound = FR_LIMIT;
	dist = logisticFit(x, 839.38251, -111.55843, 133.99694, 1.39961);
	break;
	case S_FL:
  bound = S_FL_LIMIT;
	dist = 900;//cubicFit(x, 1965.88511, -10.15601, 0.01991, -1.40721e-5);
	break;
	case S_FR:
  bound = S_FR_LIMIT;
	dist = cubicFit(x, 815.83317, -5.8258, 0.01505, -1.40109e-5);
	// dist = logisticFit(x, 67.28591, -1.10478, 0.06223, 1.17884);
	break;
	case S_BR:
  bound = S_BR_LIMIT;
	dist = cubicFit(x, 985.90138, -6.45298, 0.01508, -1.23553e-5);
	break;
	}
  bound += 50;
	if(dist>=bound)
		return 900;
	else
		return dist;
}
int IR::correction()
{
  int correctedVal = this->reading;
  if(this->reading == 900)
    return 900;
	
  Serial.print(this->id);
  Serial.print(". "+String(correctedVal));
	switch(this->id)
	{
    case FL:
    correctedVal = scaleByInterval(correctedVal,FL_offsets,START,INTERVAL,FL_LIMIT);
    break;
    case FM:
    correctedVal = scaleByInterval(correctedVal,FM_offsets,START,INTERVAL,FM_LIMIT);
    break;
    case FR:
    correctedVal = scaleByInterval(correctedVal,FR_offsets,START,INTERVAL,FR_LIMIT);
    break;
		case S_FL:
    correctedVal = scaleByInterval(correctedVal,S_FL_offsets,START,INTERVAL,S_FL_LIMIT);
		break;
		case S_FR:
    correctedVal = scaleByInterval(correctedVal,S_FR_offsets,START,INTERVAL,S_FR_LIMIT);
		break;
		case S_BR:
    correctedVal = scaleByInterval(correctedVal,S_BR_offsets,START,INTERVAL,S_BR_LIMIT);
		break;
	}
  Serial.println(">>"+String(correctedVal));
	return correctedVal;
}
static int IR::scaleByInterval(int variableToScale, int readings[], int start, int interval, int limit)
{
  int maxIndex = (limit-start)/interval + 1; // 8 for -50~300 + 50, 4 for 0~300 +100
  int offset[maxIndex];
  int i;
  // Actual Versus Ideal
  // readings[5] stands for : ACTUAL values at range -121, 5, 34, ..., 218
  // to generate IDEAL numbers, use i*50: ideal ranges -50, 0, 50, ..., 300
  // get the offset, required to scale by how much?
  for(i=0;i<maxIndex;i++)
  {
    offset[i] = i*interval + start - readings[i];
    //offset[i] = readings[i];
  }
  // proceed to determine which range the ACTUAL value falls in [-50,0] or [0,50] or ... or [250,300]
  int variableIdealDist = start;
  
  i = 0;
  while(variableToScale>=variableIdealDist)
  {
    variableIdealDist += interval;
    i++;
    if(i>maxIndex-1)return -1000;
  }
  // compute offset, comes in form of (offset + offset_transition*scale)
  if(i==maxIndex-1)variableToScale += (offset[i]);
  else variableToScale += (offset[i]) + (offset[i+1]-offset[i])*(variableToScale-(variableIdealDist/interval-1));
//  if(variableToScale < 0)
//  {
//    variableToScale += (offset[0])+(offset[1]-offset[0])*(variableToScale-(-100))/100;
//  }
//  else if(variableToScale<100)
//  {
//    variableToScale += (offset[1])+(offset[2]-offset[1])*(variableToScale-0)/100;
//  }
//  else if(variableToScale<200)
//  {
//    variableToScale += (offset[2])+(offset[3]-offset[3])*(variableToScale-100)/100;
//  }
//  else if(variableToScale<300)
//  {
//    variableToScale += (offset[3])+(offset[4]-offset[3])*(variableToScale-200)/100;
//  }
//  else
//  {
//    variableToScale += (offset[4]);
//  }
  return variableToScale;
}
static int IR::logisticFit(int x, float A1, float A2, float x0, float p)
{
	return int(A2 + (A1-A2)/(1+pow(float(x)/x0,p)));
}
static int IR::cubicFit(int x, float A, float B, float C, float D)
{
	return int(A + B*float(x) + C*pow(float(x),2) + D*pow(float(x),3));
}
// static float IR::power2Fit(float x, float xc, float A, float pl)
// {
// 	return A*pow(abs(x-xc),pl);
// }
// static float IR::log3p1Fit(float x, float a, float b, float c)
// {
// 	return a - b*(log(x+c));
// }
