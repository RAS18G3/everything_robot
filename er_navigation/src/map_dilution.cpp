// possibly merge with map_reader?
#ifndef DILUTION_MAGN
#define DILUTION_MAGN 10
#endif

#include "map_dilution.h"

nav_msgs::OccupancyGrid diluteMap(nav_msgs::OccupancyGrid &oldMap)
{
  // mean filter, WIP
  int x, y;
  int dil_magn = DILUTION_MAGN;
  int xMax = oldMap.info.height;
  int yMax = oldMap.info.width;

std::vector<double>  = std::vector<double>(oldMap.data);

  for (x = 0; x < xMax; x++)
  {
    for (y = 0; y < yMax; y++)
    {
      newMap.data[y*newMap.info.width + x] = (int) sum;
    }
  }

}
