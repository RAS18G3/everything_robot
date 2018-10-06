#ifndef MAP_READER_H
#define MAP_READER_H

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"

#include <cmath>
#include <string>
#include <fstream>
#include <sstream>

class MapReader {
public:
  MapReader(std::string path_to_map);
  ~MapReader();

  nav_msgs::OccupancyGrid occupancy_grid() const;

private:
  struct Wall {
    double x_start, x_end, y_start, y_end;

    Wall(double xs, double xe, double ys, double ye) : x_start(xs), x_end(xe), y_start(ys), y_end(ye) {};
  };

  std::vector<Wall> walls;
};


#endif
