#ifndef OCCUPANCY_GRID_UTILS_H
#define OCCUPANCY_GRID_UTILS_H

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"

#include <cmath>

// TODO: add namespace

void draw_line(nav_msgs::OccupancyGrid &occupancy_grid, double x_start, double x_end, double y_start, double y_end, int8_t value=100);
int8_t&  at(nav_msgs::OccupancyGrid &occupancy_grid, int x, int y);
int sign(int x);

#endif
