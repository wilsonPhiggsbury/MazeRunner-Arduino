#include "IR.h"


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
    voltage = scaleByInterval(voltage,FL_offsets,FL_START,FL_END,FL_INTERVAL);
    break;

    case FM:
    voltage = scaleByInterval(voltage,FM_offsets,FM_START,FM_END,FM_INTERVAL);
    break;

    case FR:
	voltage = scaleByInterval(voltage,FR_offsets,FR_START,FR_END,FR_INTERVAL);
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
  // Actual Versus Ideal
  // voltageSamples[5] stands for : ACTUAL values at range -121, 5, 34, ..., 218
  // to generate IDEAL numbers, use i*50: ideal ranges -50, 0, 50, ..., 300
  // get the offset, required to scale by how much?
  // proceed to determine which range the ACTUAL value falls in [-50,0] or [0,50] or ... or [250,300]
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
//      Serial.println("i: "+String(i)+" End boundary: "+String((i+1)*interval+startDist));
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

// int IR::fitCurve()
// {
//   int dist,bound;
//   int x = this->reading;
//   switch(this->id)
//   {
//     case FL:
//     bound = FL_LIMIT;
//     dist = logisticFit(x, 262.81293, -35.58077, 322.95448, 3.54856);
//   //  if(dist<0)
//   //    dist = cubicFit(x, 2016.17638, -10.27134, 0.0175, -1.0087e-5);
//     dist = scaleByInterval(dist,FL_offsets,START,FL_LIMIT,INTERVAL);      
//     break;
//     case FM:
//     bound = FM_LIMIT;
//     dist = logisticFit(x, 2072.83017, -93.91462, 41.48998, 1.22671);
//   //  if(dist<0)
//   //    dist = cubicFit(x, 3939.21375, -19.38187, 0.03189, -1.76112e-5);
//     dist = scaleByInterval(dist,FM_offsets,START,FM_LIMIT,INTERVAL);
//     break;
//     case FR:
//     bound = FR_LIMIT;
//     dist = logisticFit(x, 4508.49721, -90.55705, 34.42585, 1.35516);
//   //  if(dist<0)
//   //    dist = cubicFit(x,4309.69231, -20.23017, 0.0319, -1.69584e-5);
//     dist = scaleByInterval(dist,FR_offsets,START,FR_LIMIT,INTERVAL);
//     break;
//     case S_FL:
//     dist = cubicFit(x, 1799.43651, -9.76627, 0.02037, -1.51172e-5);
//     dist = scaleByInterval(dist,S_FL_offsets,START,S_FL_LIMIT,INTERVAL);
//     break;
//     case S_FR:
//      bound = S_FR_LIMIT;
//      dist = logisticFit(x, 2698.87551, -121.92001, 29.03353, 1.16081);
//      //dist = cubicFit(x, 815.83317, -5.8258, 0.01505, -1.40109e-5);
//     dist = scaleByInterval(dist,S_FR_offsets,START,S_FR_LIMIT,INTERVAL);
//     break;
//     case S_BR:
//      bound = S_BR_LIMIT;
//      dist = logisticFit(x, 10554.28817, -92.81836, 10.60316, 1.22066);
//      //dist = cubicFit(x, 985.90138, -6.45298, 0.01508, -1.23553e-5);
//     dist = scaleByInterval(dist,S_BR_offsets,START,S_BR_LIMIT,INTERVAL);
//     break;
//   }
//  return dist;
// }
// static int IR::logisticFit(int x, float A1, float A2, float x0, float p)
// {
// 	return int(A2 + (A1-A2)/(1+pow(float(x)/x0,p)));
// }
static int cubicFit(int x, float A, float B, float C, float D)
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



//array frontn is the voltage readings for the front sensors
// i is sensor#, n is voltage stored

	


