#ifndef OCCUPANCY_GRID_UTILS_H
#define OCCUPANCY_GRID_UTILS_H

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"

#include <cmath>

void bresenham_line(nav_msgs::OccupancyGrid &occupancy_grid, double x_start, double x_end, double y_start, double y_end);

#endif
