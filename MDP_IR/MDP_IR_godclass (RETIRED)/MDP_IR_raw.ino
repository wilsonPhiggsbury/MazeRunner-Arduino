#define READINGS 31


const int READINGS_HALF = (READINGS-1)/2;
const int sensors[6] = {A0,A1,A2,A3,A4,A5};
float voltageMedians[6][READINGS];
float readings[6];
void setup() {
  Serial.begin(9600);
}

void loop() {
  
  if(Serial.available())
  {
    Serial.read();
    // do reading
    for(int i=0; i<6; i++)
    {
      for(int j=0; j<READINGS; j++)
      {
        voltageMedians[i][j] = analogRead(sensors[i])*5.0/1024.0;
      }
      delay(5);
      readings[i] = takeMedian(voltageMedians[i]);
    }
    // print values, seperated by "tab tab newline tab tab newline"
    for(int i=0; i<6; i++)
    {
      float fittedDist = fit(readings[i],i);
      Serial.print(String(readings[i],2));
      if(i==2 || i==5)Serial.print("\n");
      else Serial.print("\t");
      
    }
  }
  
  delay(200); // slow down serial port
}
float takeMedian(float nums[READINGS])
{
  // insertion sort the nums array
  for(int i=1; i<READINGS; i++)
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
  return nums[READINGS_HALF];
}
float fit(float value, int index)
{
  float dist,bound;
  switch(index)
  {
    case 0: // L front
    bound = 3.5;
    dist = logisticFit(value, 147.2596, -0.88058, 0.03274, 1.17774);
    break;
    case 1: // M front
    bound = 2.4;
    dist = power2Fit(value, 2.60905, 0.51431, 2.47468);
    break;
    case 2: // R front
    bound = 3.5;
    dist = logisticFit(value, 163.14261, -0.94237, 0.04718, 1.25732);
    break;
    case 3: // L front (side, long)
    bound = 4;
    dist = log3p1Fit(value, 3.5882, 3.66031, -0.33237);
    /* below is poly3fit, an alternative
     *  dist = 17.0795 - 18.16677*x + 7.56588*pow(x,2) - 1.15652*pow(X,3);
     */
    break;
    case 4: // R front (side, short)
    bound = 3.5;
    dist = logisticFit(value, 135.99865, -1.23964, 0.02665, 1.10135);
    break;
    case 5:
    bound = 2.5; // R back (side, short)
    dist = logisticFit(value, 19.4419, -0.62827, 0.31499, 1.72727);
    /* below is real plot, above uses "quick plot" to define range 
     *  bound = 3.5;
     *  dist = logisticFit(value, 905.58779, -0.07398, 0.11493, 2.82832);
     */
    break;
  }
  dist = correction(dist, index);
  
  if(dist>bound)
    return 100.00;
  else
    return dist;
}
float logisticFit(float x, float A1, float A2, float x0, float p)
{
  return (A2 + (A1-A2)/(1+pow(x/x0,p)));
}
float power2Fit(float x, float xc, float A, float pl)
{
  return A*pow(abs(x-xc),pl);
}
float log3p1Fit(float x, float a, float b, float c)
{
  return a - b*(log(x+c));
}
float correction(float value, int index)
{
  switch(index)
  {
    case 0:// scale down
    // needs scaling at extreme ranges (2~3), gets blind outside 3.5
    if(value>2)
    {
      value *= 1-(value-2)/15; 
    }
    break;
    case 1: // scale up
    if(value>2)
    {
      value *= 1+(value-2)*1.153; 
    }
    case 2: // scale down
    if(value>2)
    {
      value *= 1-(value-2)/20;
    }
    break;
  }
  return value;
}

