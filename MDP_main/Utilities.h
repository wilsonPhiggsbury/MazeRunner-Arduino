#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
class Utilities
{
  private:

  public:
    Utilities();
    static int takeMedian(int nums[], int samples);
    static String getSubString(String data, char separator, int index);
};
