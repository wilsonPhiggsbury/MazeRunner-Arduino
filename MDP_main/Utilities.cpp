#include "Utilities.h"

Utilities::Utilities()
{
}
int Utilities::takeMedian(int nums[], int samples)
{
    int size = sizeof(nums)/sizeof(nums[0]);
        // insertion sort the nums array
    for(int i=1; i<size; i++)
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
    return nums[(samples-1)/2];
}

