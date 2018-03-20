#include "IR.h"


IR::IR(int id)
{
	this->id = id;
}

float IR::takeReading(bool convertToDist)
{
	int voltageMedians[SAMPLES];
	for(int j=0; j<SAMPLES; j++)
	{
		voltageMedians[j] = analogRead(this->id);
	}
	this->reading = takeMedian(voltageMedians);
	if(convertToDist)
	{
		this->reading = lookUptable();
	}
	return this->reading;
} 

int IR::lookUptable()
{
  int voltage = this->reading;
  switch(this->id)
  {
    case FL:
    voltage = scaleByInterval(voltage,FL_offsets,FRONT_START,FRONT_END,FRONT_INTERVAL);
    break;

    case FM:
    voltage = scaleByInterval(voltage,FM_offsets,FRONT_START,FRONT_END,FRONT_INTERVAL);
    break;

    case FR:
	voltage = scaleByInterval(voltage,FR_offsets,FRONT_START,FRONT_END,FRONT_INTERVAL);
    break;

    case S_FL:
    voltage = scaleByInterval(voltage,S_FL_offsets,LONG_START,LONG_END,LONG_INTERVAL);
//    voltage = scaleByInterval(voltage,S_FL_offsets,30,80,5);
   
    break;

    case S_FR:
    voltage = scaleByInterval(voltage,S_FR_offsets,SIDE_START,SIDE_END,SIDE_INTERVAL);
    break;

    case S_BR:
    voltage = scaleByInterval(voltage,S_BR_offsets,SIDE_START,SIDE_END,SIDE_INTERVAL);
    break;
  }
  return voltage;
}
int IR::scaleByInterval(int voltageToScale, const int voltageSamples[], int startDist, int endDist, int interval)
{
	int maxIndex = (endDist-startDist)/interval + 1; // 8 for -50~300 + 50, 4 for 0~300 +100
	int i;
//  // Actual Versus Ideal
//  // voltageSamples[5] stands for : ACTUAL values at range -121, 5, 34, ..., 218
//  // to generate IDEAL numbers, use i*50: ideal ranges -50, 0, 50, ..., 300
//  // get the offset, required to scale by how much?
//  // proceed to determine which range the ACTUAL value falls in [-50,0] or [0,50] or ... or [250,300]
  if(voltageToScale > pgm_read_word(voltageSamples+i))
    return -900;;
  for(i=0; i<maxIndex-1; i++)
  {
    int thisVal = pgm_read_word(voltageSamples+i); // voltageSamples[i]
    int nextVal = pgm_read_word(voltageSamples+i+1); // voltageSamples[i+1]
          // if(this->id == FM)Serial.println("Input: "+String(voltageToScale)+" Scaling from "+String(thisVal)+"-"+String(nextVal)+" to "+String(i*interval+startDist)+"-"+String((i+1)*interval+startDist));

    if(voltageToScale <= thisVal && voltageToScale >= nextVal)
    {
      voltageToScale = map(voltageToScale, thisVal, nextVal, i*interval+startDist, (i+1)*interval+startDist);
      return voltageToScale;
    }
  }
  return 900;
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

static int cubicFit(int x, float A, float B, float C, float D)
{
	return int(A + B*float(x) + C*pow(float(x),2) + D*pow(float(x),3));
}

